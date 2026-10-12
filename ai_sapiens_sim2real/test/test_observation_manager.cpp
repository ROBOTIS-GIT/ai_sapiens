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
#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

using ai_sapiens_sim2real::ObservationManager;
using ai_sapiens_sim2real::PolicyJointContext;
using ai_sapiens_sim2real::SharedControlData;

namespace
{

YAML::Node two_term_history_config()
{
  YAML::Node observations;
  observations["base_ang_vel"]["history_length"] = 2;
  observations["velocity_commands"]["history_length"] = 2;
  return observations;
}

void expect_vector_near(const std::vector<float> & actual, const std::vector<float> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], 1e-6f) << "index " << i;
  }
}

}  // namespace

TEST(ObservationManager, DefaultHistoryLayoutRemainsTermMajor)
{
  SharedControlData shared;
  shared.sensors.angular_velocity = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  shared.mode.velocity_commands = Eigen::Vector3f(10.0f, 20.0f, 30.0f);
  ObservationManager manager(two_term_history_config(), &shared, PolicyJointContext{}, nullptr);

  shared.sensors.angular_velocity = Eigen::Vector3f(4.0f, 5.0f, 6.0f);
  shared.mode.velocity_commands = Eigen::Vector3f(40.0f, 50.0f, 60.0f);

  expect_vector_near(
    manager.compute(),
    {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f,
      10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f});
}

TEST(ObservationManager, DwaqHistoryIsFrameMajorAndUsesCurrentCommandInEveryFrame)
{
  SharedControlData shared;
  shared.sensors.angular_velocity = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  shared.mode.velocity_commands = Eigen::Vector3f(10.0f, 20.0f, 30.0f);
  auto observations = two_term_history_config();
  observations["use_gym_history"] = true;
  observations["replace_velocity_command_history"] = true;
  ObservationManager manager(observations, &shared, PolicyJointContext{}, nullptr);

  shared.sensors.angular_velocity = Eigen::Vector3f(4.0f, 5.0f, 6.0f);
  shared.mode.velocity_commands = Eigen::Vector3f(40.0f, 50.0f, 60.0f);

  expect_vector_near(
    manager.compute(),
    {1.0f, 2.0f, 3.0f, 40.0f, 50.0f, 60.0f,
      4.0f, 5.0f, 6.0f, 40.0f, 50.0f, 60.0f});
}

TEST(ObservationManager, GymHistoryRequiresEqualTermHistoryLengths)
{
  auto observations = two_term_history_config();
  observations["use_gym_history"] = true;
  observations["velocity_commands"]["history_length"] = 3;
  SharedControlData shared;

  EXPECT_THROW(
    ObservationManager(observations, &shared, PolicyJointContext{}, nullptr),
    std::runtime_error);
}

TEST(ObservationManager, DwaqGaitPhaseUsesFourValuesAndZerosWhenStopped)
{
  YAML::Node observations;
  observations["gait_phase"]["params"]["command_threshold"] = 0.1;
  observations["gait_phase"]["params"]["offset"] = 0.5;
  SharedControlData shared;
  shared.policy.gait_phase = 0.25f;
  ObservationManager manager(observations, &shared, PolicyJointContext{}, nullptr);

  expect_vector_near(manager.compute(), {0.0f, 0.0f, 1.0f, 1.0f});

  shared.mode.velocity_commands.x() = 0.2f;
  expect_vector_near(manager.compute(), {1.0f, -1.0f, 0.0f, 0.0f});
}

TEST(ObservationManager, DwaqPolicyProducesFiveFramesOfEightyTwoValues)
{
  YAML::Node observations;
  observations["use_gym_history"] = true;
  observations["replace_velocity_command_history"] = true;
  for (const auto * term_name : {
    "base_ang_vel", "projected_gravity", "velocity_commands", "joint_pos",
    "joint_vel", "actions", "gait_phase"})
  {
    observations[term_name]["history_length"] = 5;
  }
  observations["gait_phase"]["params"]["command_threshold"] = 0.1;
  observations["gait_phase"]["params"]["period"] = 0.8;
  observations["gait_phase"]["params"]["offset"] = 0.5;

  SharedControlData shared;
  shared.resize(23, 23);
  PolicyJointContext joints;
  for (size_t index = 0; index < 23; ++index) {
    joints.policy_joint_names.push_back("joint_" + std::to_string(index));
    joints.policy_to_controller.push_back(index);
  }

  ObservationManager manager(observations, &shared, joints, nullptr);
  shared.mode.velocity_commands = Eigen::Vector3f(0.2f, -0.1f, 0.3f);
  const auto input = manager.compute();

  constexpr size_t kFrameSize = 82;
  constexpr size_t kHistoryLength = 5;
  constexpr size_t kCommandOffset = 6;
  ASSERT_EQ(input.size(), kFrameSize * kHistoryLength);
  for (size_t frame = 0; frame < kHistoryLength; ++frame) {
    EXPECT_FLOAT_EQ(input[frame * kFrameSize + kCommandOffset], 0.2f);
    EXPECT_FLOAT_EQ(input[frame * kFrameSize + kCommandOffset + 1], -0.1f);
    EXPECT_FLOAT_EQ(input[frame * kFrameSize + kCommandOffset + 2], 0.3f);
  }
}
