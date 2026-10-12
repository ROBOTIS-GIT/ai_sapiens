// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/sensor_handles/joint_states_sensor_handle.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

YAML::Node carry_observations(bool dual)
{
  YAML::Node config;
  for (const auto * name : {"base_ang_vel", "projected_gravity", "velocity_commands",
      "joint_pos_rel", "joint_vel_rel", "last_action", "mode_command"})
  {
    config[name]["history_length"] = std::string(name) == "mode_command" ? 1 : 4;
  }
  if (dual) {
    config["applied_torque"]["history_length"] = 5;
    config["applied_torque"]["scale"] = 0.01;
    config["joint_pos_rel_long"]["history_length"] = 20;
    config["applied_torque_long"]["history_length"] = 20;
    config["applied_torque_long"]["scale"] = 0.01;
  }
  return config;
}

TEST(DualHistoryObservation, LayoutMappingScalingAndHistoryMatchBundle)
{
  SharedControlData data;
  data.resize(23, 23);
  PolicyJointContext joints;
  for (size_t i = 0; i < 23; ++i) {
    joints.policy_joint_names.push_back("joint_" + std::to_string(22 - i));
    joints.policy_to_controller.push_back(22 - i);
    data.sensors.joint_pos[i] = 0.1f * i;
    data.output.default_joint_pos[i] = 0.01f * i;
    data.sensors.joint_effort[i] = 10.0f + i;
  }
  ObservationManager legacy(carry_observations(false), &data, joints, nullptr);
  ObservationManager dual(carry_observations(true), &data, joints, nullptr);
  ASSERT_EQ(legacy.get_observation_size(), 318u);
  ASSERT_EQ(dual.get_observation_size(), 1353u);
  auto obs = dual.compute();
  for (size_t frame = 0; frame < 20; ++frame) {
    EXPECT_NEAR(obs[433 + frame * 23], 1.98, 1e-5);
    EXPECT_NEAR(obs[893 + frame * 23], 0.32, 1e-5);
  }
  for (int step = 1; step <= 25; ++step) {
    for (size_t j = 0; j < 23; ++j) {
      data.sensors.joint_pos[j] = step + 0.1f * j;
      data.sensors.joint_effort[j] = 100.0f * step + j;
    }
    obs = dual.compute();
    const auto old = legacy.compute();
    for (size_t j = 0; j < old.size(); ++j) {
      EXPECT_FLOAT_EQ(obs[j], old[j]);
    }
  }
  for (size_t j = 0; j < 23; ++j) {
    const float controller = static_cast<float>(22 - j);
    for (int frame = 0; frame < 5; ++frame) {
      EXPECT_NEAR(obs[318 + frame * 23 + j], 21 + frame + 0.01f * controller, 1e-5);
    }
    for (int frame = 0; frame < 20; ++frame) {
      EXPECT_NEAR(obs[433 + frame * 23 + j], 6 + frame + 0.09f * controller, 1e-5);
      EXPECT_NEAR(obs[893 + frame * 23 + j], 6 + frame + 0.01f * controller, 1e-5);
    }
  }
  data.sensors.joint_effort.setConstant(7.0f);
  dual.reset();
  obs = dual.compute();
  for (size_t i = 893; i < 1353; ++i) {
    EXPECT_NEAR(obs[i], 0.07, 1e-6);
  }
}

class JointEffortFeedbackTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(JointEffortFeedbackTest, ReordersEffortAndClearsMissingEntries)
{
  using namespace std::chrono_literals;
  SharedControlData data;
  data.resize(2, 2);
  auto node = std::make_shared<rclcpp::Node>("joint_effort_feedback_test");
  JointStatesSensorHandle handle(node, &data.sensors, &data.requests, 5.0,
    {"left", "right"}, "/test_dual_history/joints");
  auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(
    "/test_dual_history/joints", 10);
  auto send = [&](const std::vector<double> & effort, double marker) {
      sensor_msgs::msg::JointState msg;
      msg.name = {"right", "left"};
      msg.position = {marker, -marker};
      msg.velocity = {0, 0};
      msg.effort = effort;
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      do {
        publisher->publish(msg);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(5ms);
        handle.update(node->now());
        if (data.sensors.joint_pos[1] == marker) {return true;}
      } while (std::chrono::steady_clock::now() < deadline);
      return false;
    };
  ASSERT_TRUE(send({20, 10}, 1));
  EXPECT_FLOAT_EQ(data.sensors.joint_effort[0], 10);
  EXPECT_FLOAT_EQ(data.sensors.joint_effort[1], 20);
  ASSERT_TRUE(send({}, 2));
  EXPECT_TRUE(data.sensors.joint_effort.isZero());
  ASSERT_TRUE(send({30}, 3));
  EXPECT_FLOAT_EQ(data.sensors.joint_effort[0], 0);
  EXPECT_FLOAT_EQ(data.sensors.joint_effort[1], 30);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
