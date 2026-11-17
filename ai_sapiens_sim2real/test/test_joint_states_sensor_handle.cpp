// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <chrono>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "ai_sapiens_sim2real/sensor_handles/joint_states_sensor_handle.hpp"

using namespace ai_sapiens_sim2real;
using namespace std::chrono_literals;

class JointStatesSensorHandleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("joint_mapping_test");
    group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    handle = std::make_unique<JointStatesSensorHandle>(node, &sensors, &requests, 10.0,
      std::vector<std::string>{"left", "right"}, "/test/joint_mapping", options);
    publisher = node->create_publisher<sensor_msgs::msg::JointState>("/test/joint_mapping", 10);
    inputs = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    inputs->add_callback_group(group, node->get_node_base_interface());

    for (int i = 0; i < 200 && publisher->get_subscription_count() == 0; ++i) {
      inputs->spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(publisher->get_subscription_count(), 0U);
  }

  void TearDown() override
  {
    inputs.reset();
    handle.reset();
    publisher.reset();
    group.reset();
    node.reset();
    rclcpp::shutdown();
  }

  void send(
    std::vector<std::string> names, std::vector<double> positions,
    std::vector<double> velocities)
  {
    sensor_msgs::msg::JointState msg;
    msg.name = std::move(names);
    msg.position = std::move(positions);
    msg.velocity = std::move(velocities);
    for (int i = 0; i < 10; ++i) {
      publisher->publish(msg);
      inputs->spin_some();
      std::this_thread::sleep_for(2ms);
    }
    handle->update(node->now());
  }

  void flush_logs()
  {
    for (int i = 0; i < 10; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(2ms);
    }
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr group;
  SensorData sensors;
  ModeRequests requests;
  std::unique_ptr<JointStatesSensorHandle> handle;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> inputs;
};

TEST_F(JointStatesSensorHandleTest, ReusesMappingAndRebuildsForReorderingAndDuplicates)
{
  send({"left", "right"}, {1, 2}, {3, 4});
  ASSERT_TRUE(handle->is_ready());
  EXPECT_FLOAT_EQ(sensors.joint_pos[0], 1);
  send({"left", "right"}, {5, 6}, {7, 8});
  EXPECT_FLOAT_EQ(sensors.joint_pos[0], 5);
  EXPECT_FLOAT_EQ(sensors.joint_vel[1], 8);

  send({"extra", "right", "left"}, {9, 10, 11}, {12, 13, 14});
  EXPECT_FLOAT_EQ(sensors.joint_pos[0], 11);
  EXPECT_FLOAT_EQ(sensors.joint_pos[1], 10);
  EXPECT_FLOAT_EQ(sensors.joint_vel[0], 14);

  send({"left", "right", "left"}, {15, 16, 17}, {18, 19, 20});
  EXPECT_FLOAT_EQ(sensors.joint_pos[0], 17);
  EXPECT_FLOAT_EQ(sensors.joint_vel[0], 20);
}

TEST_F(JointStatesSensorHandleTest, RejectsIncompleteSamplesWithoutReplacingAcceptedValues)
{
  send({}, {}, {});
  EXPECT_FALSE(handle->is_ready());
  send({"left"}, {1}, {2});
  EXPECT_FALSE(handle->is_ready());
  send({"left", "right"}, {1, 2}, {3, 4});
  ASSERT_TRUE(handle->is_ready());

  send({"left", "right"}, {5, 6}, {7});
  EXPECT_FLOAT_EQ(sensors.joint_pos[0], 1);
  EXPECT_FLOAT_EQ(sensors.joint_vel[1], 4);
  send({}, {}, {});
  EXPECT_FLOAT_EQ(sensors.joint_pos[0], 1);
  send({"right", "left"}, {8, 9}, {10, 11});
  EXPECT_FLOAT_EQ(sensors.joint_pos[0], 9);
  EXPECT_FLOAT_EQ(sensors.joint_vel[1], 10);
}

TEST_F(JointStatesSensorHandleTest, MissingJointWarningIsDeferredAndThrottledFromStartup)
{
  testing::internal::CaptureStderr();
  send({"left"}, {1}, {2});
  EXPECT_TRUE(testing::internal::GetCapturedStderr().empty());

  testing::internal::CaptureStderr();
  flush_logs();
  const auto first = testing::internal::GetCapturedStderr();
  EXPECT_NE(first.find("1/2 joints complete"), std::string::npos);
  EXPECT_NE(first.find("Missing/incomplete: right"), std::string::npos);
  EXPECT_NE(first.find("detected_at="), std::string::npos);
  EXPECT_NE(first.find("log_delay="), std::string::npos);
  EXPECT_EQ(first.find("Joint 'right'"), std::string::npos);

  testing::internal::CaptureStderr();
  send({"left"}, {3}, {4});
  flush_logs();
  EXPECT_EQ(testing::internal::GetCapturedStderr().find("Ignoring incomplete"), std::string::npos);

  std::this_thread::sleep_for(1010ms);
  send({"left"}, {3}, {4});
  testing::internal::CaptureStderr();
  flush_logs();
  EXPECT_NE(testing::internal::GetCapturedStderr().find("Ignoring incomplete"), std::string::npos);
}

TEST_F(JointStatesSensorHandleTest, DiagnosticsSnapshotPreservesMissingDetailsAndReportsReadyOnce)
{
  const std::vector<std::string> names{"j0", "j1", "j2", "j3", "j4", "j5", "j6", "j7", "j8", "j9"};
  JointStatesDiagnostics diagnostics(node, "/test/diagnostics", names);
  sensor_msgs::msg::JointState msg;
  std::vector<size_t> indices(names.size(), std::numeric_limits<size_t>::max());
  indices[0] = 0;
  msg.position = {1.0};

  testing::internal::CaptureStderr();
  diagnostics.record_incomplete(msg, indices, 0);
  diagnostics.record_ready();
  EXPECT_TRUE(testing::internal::GetCapturedStderr().empty());

  // The pending diagnostic owns its snapshot, independent of subsequent input changes.
  indices.assign(names.size(), 0);
  msg.velocity = {2.0};
  testing::internal::CaptureStderr();
  flush_logs();
  const auto output = testing::internal::GetCapturedStderr();
  EXPECT_NE(output.find("0/10 joints complete"), std::string::npos);
  EXPECT_NE(output.find(
      "j0(missing position/velocity), j1, j2, j3, j4, j5, j6, j7, ..."), std::string::npos);
  EXPECT_NE(output.find("all 10 joints received"), std::string::npos);

  testing::internal::CaptureStderr();
  diagnostics.record_ready();
  flush_logs();
  EXPECT_EQ(testing::internal::GetCapturedStderr().find("all 10 joints received"),
    std::string::npos);
}
