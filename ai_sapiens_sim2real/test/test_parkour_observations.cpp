// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <numeric>
#include <cmath>
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"
using namespace ai_sapiens_sim2real;
TEST(Parkour, TermMajorHistoryDepthTailAndOnnx)
{
  const std::string asset = PARKOUR_ASSET_DIR;
  Sim2RealConfig config(asset + "/params/sim2real.yaml");
  SharedControlData shared;
  shared.resize(23, 23);
  PolicyJointContext joints;
  joints.policy_joint_names = config.policy_joints();
  joints.policy_to_controller.resize(23);
  std::iota(joints.policy_to_controller.begin(), joints.policy_to_controller.end(), 0);
  for (int j = 0; j < 23; ++j) {
    shared.output.default_joint_pos[j] = config.joint_properties().default_position[j];
    shared.sensors.joint_pos[j] = shared.output.default_joint_pos[j];
  }
  ObservationManager observations(config.observations(), &shared, joints, nullptr);
  for (int tick = 0; tick < 8; ++tick) {
    shared.sensors.angular_velocity.setConstant(tick);
    shared.mode.velocity_commands.setConstant(tick * 0.1f);
    shared.sensors.joint_vel.setConstant(tick);
    shared.policy.last_action.assign(23, tick * 0.01f);
    shared.sensors.depth_history.fill(tick * 0.1f);
    auto obs = observations.compute();
    ASSERT_EQ(obs.size(), 5232U);
    if (tick != 7) {continue;}
    std::vector<float> expected;
    for (int t = 0; t < 8; ++t) {
      expected.insert(expected.end(), 3, t * 0.25f);
    }
    for (int t = 0; t < 8; ++t) {
      expected.insert(expected.end(), {0, 0, -1});
    }
    for (int t = 0; t < 8; ++t) {
      expected.insert(expected.end(), 3, t * 0.1f);
    }
    expected.insert(expected.end(), 23 * 8, 0);
    for (int t = 0; t < 8; ++t) {
      expected.insert(expected.end(), 23, t * 0.05f);
    }
    for (int t = 0; t < 8; ++t) {
      expected.insert(expected.end(), 23, t * 0.01f);
    }
    expected.insert(expected.end(), 4608, 0.7f);
    ASSERT_EQ(expected.size(), obs.size());
    for (size_t i = 0; i < obs.size(); ++i) {
      ASSERT_NEAR(obs[i], expected[i], 1e-7) << i;
    }
    OnnxInference inference(asset + "/exported/policy.onnx");
    std::unordered_map<std::string, std::vector<float>> actual_input{{"obs", obs}};
    std::unordered_map<std::string, std::vector<float>> expected_input{{"obs", expected}};
    const auto actual = inference.run(actual_input), golden = inference.run(expected_input);
    ASSERT_EQ(actual.size(), 23U);
    for (size_t j = 0; j < actual.size(); ++j) {
      ASSERT_TRUE(std::isfinite(actual[j])); EXPECT_NEAR(actual[j], golden[j], 1e-5);
    }
  }
}
