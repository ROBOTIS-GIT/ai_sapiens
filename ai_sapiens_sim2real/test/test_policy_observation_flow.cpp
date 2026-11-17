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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>

#include "ai_sapiens_sim2real/policy/mimic/mimic_policy_runtime.hpp"

using namespace ai_sapiens_sim2real;

namespace
{
class PolicyObservationFlow : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<rclcpp::Node>("policy_observation_flow_test");
    auto pattern = (std::filesystem::temp_directory_path() / "observation-flow-XXXXXX").string();
    ASSERT_NE(::mkdtemp(pattern.data()), nullptr);
    directory = pattern;
    data.resize(2, 2);
    data.sensors.joint_pos << .4f, -.7f;
    data.sensors.is_imu_usable = true;
    data.mode.velocity_source = VelocitySource::Api;
  }

  void TearDown() override
  {
    runtime.reset();
    ObservationRegistry::get_registry().erase("flow_test_joint_pos");
    std::filesystem::remove_all(directory);
  }

  Sim2RealConfig write_config(const YAML::Node & config)
  {
    const auto path = directory / "sim2real.yaml";
    std::ofstream(path) << config;
    return Sim2RealConfig(path);
  }

  void create_phase_policy(const YAML::Node & phase, int history_length)
  {
    auto yaml = YAML::LoadFile(std::string(COMMON_POLICY_TEST_DATA_DIR) + "/sim2real.yaml");
    yaml["observations"] = YAML::Node(YAML::NodeType::Map);
    yaml["observations"]["gait_phase"] = YAML::Clone(phase);
    // Reuse the existing 21-input model: 18 phase values plus the current command.
    yaml["observations"]["gait_phase"]["history_length"] = history_length;
    yaml["observations"]["velocity_commands"] = YAML::Node(YAML::NodeType::Map);
    const auto config = write_config(yaml);
    runtime = std::make_unique<PolicyRuntime>(node, "phase_order",
        std::string(COMMON_POLICY_TEST_DATA_DIR) + "/policy.onnx", config,
        config.policy_joints(), &data);
    runtime->enter();
  }

  PolicyUpdateResult tick(double seconds = .02)
  {
    return runtime->update(rclcpp::Duration::from_seconds(seconds));
  }

  void expect_observed_velocity(float expected)
  {
    ASSERT_EQ(data.policy.input.size(), 21U);
    EXPECT_FLOAT_EQ(data.policy.input[18], expected);
    EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), expected);
  }

  rclcpp::Node::SharedPtr node;
  std::filesystem::path directory;
  SharedControlData data;
  std::unique_ptr<PolicyRuntime> runtime;
};
}  // namespace

TEST_F(PolicyObservationFlow, AdaptivePhaseUsesCurrentCommandOnTheSameTick)
{
  const auto phase_path = std::filesystem::path(__FILE__).parent_path() /
    "data/adaptive_phase/phase_contract.yaml";
  const auto phase = YAML::LoadFile(phase_path.string())["gait_phase"];
  data.api.velocity_command_raw.x() = .2f;
  create_phase_policy(phase, 3);
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), .2f);  // Entry history uses current command.
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  EXPECT_FLOAT_EQ(data.policy.phase_state.phase, 0.f);
  expect_observed_velocity(.2f);

  data.api.velocity_command_raw.x() = 1.f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  const float expected_period = .8f + (1.f - std::exp(-.02f / .25f)) * (1.16f - .8f);
  EXPECT_NEAR(data.policy.phase_state.period, expected_period, 1e-6);
  EXPECT_NEAR(data.policy.phase_state.phase, .02f / expected_period, 1e-6);
  expect_observed_velocity(1.f);

  data.api.velocity_command_raw.x() = .2f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  EXPECT_LT(data.policy.phase_state.period, expected_period);
  expect_observed_velocity(.2f);
}

TEST_F(PolicyObservationFlow, CommandResetPhaseUsesPreviousCommandWhileObservationUsesCurrent)
{
  data.api.velocity_command_raw.x() = .5f;
  create_phase_policy(YAML::Load("params: {period: 0.8, command_threshold: 0.1}"), 9);
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), 0.f);  // Preserve legacy entry ordering.
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  EXPECT_FLOAT_EQ(data.policy.phase_state.phase, 0.f);
  expect_observed_velocity(.5f);

  data.api.velocity_command_raw.x() = 0.f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(data.policy.phase_state.phase, .02f / .8f, 1e-6);
  expect_observed_velocity(0.f);

  data.api.velocity_command_raw.x() = .5f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  EXPECT_FLOAT_EQ(data.policy.phase_state.phase, 0.f);
  expect_observed_velocity(.5f);

  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(data.policy.phase_state.phase, .02f / .8f, 1e-6);
}

TEST_F(PolicyObservationFlow, MimicCompletionStopsObservationAndActionUpdatesAndEpisodeTime)
{
  auto observation_calls = std::make_shared<size_t>(0);
  ObservationRegistry::register_observation("flow_test_joint_pos",
    [observation_calls](const ObservationContext & context, const YAML::Node &) {
      ++*observation_calls;
      return std::vector<float>{context.shared.sensors.joint_pos[0],
      context.shared.sensors.joint_pos[1]};
    });
  auto yaml = YAML::LoadFile(std::string(HISTORY_POLICY_TEST_DATA_DIR) + "/sim2real.yaml");
  yaml["observations"] = YAML::Load("flow_test_joint_pos: {history_length: 4}");
  const auto config = write_config(yaml);
  MotionPlayback playback;
  const auto motion = std::filesystem::path(__FILE__).parent_path() / "data/common/motion.csv";
  playback.reference = std::make_shared<MotionReference>(motion.string(), 50.f,
    std::vector<std::string>{"left_hip_roll_joint", "right_hip_roll_joint", "waist_yaw_joint"});
  playback.time_end = .02;
  runtime = std::make_unique<MimicPolicyRuntime>(node, "completion_test",
      std::string(HISTORY_POLICY_TEST_DATA_DIR) + "/policy.onnx", config,
      config.policy_joints(), &data, std::move(playback), "Damping");
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);  // Sample the endpoint once.
  EXPECT_TRUE(data.requests.state_name.empty());

  const auto calls_at_endpoint = *observation_calls;
  const auto input_at_endpoint = data.policy.input;
  const auto action_at_endpoint = data.policy.last_action;
  const auto target_at_endpoint = runtime->target_command().position;
  const auto time_at_endpoint = data.policy.episode_time;
  data.sensors.joint_pos.setConstant(std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(data.requests.state_name, "Damping");
    EXPECT_EQ(*observation_calls, calls_at_endpoint);  // No new sample can enter history.
    EXPECT_EQ(data.policy.input, input_at_endpoint);
    EXPECT_EQ(data.policy.last_action, action_at_endpoint);
    EXPECT_EQ(runtime->target_command().position, target_at_endpoint);
    EXPECT_FLOAT_EQ(data.policy.episode_time, time_at_endpoint);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_FALSE(data.requests.damping);  // Completion is a state request, not an action fault.
  }
}
