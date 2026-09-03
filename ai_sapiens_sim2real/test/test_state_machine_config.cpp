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

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/config/root_config_validator.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_machine_config.hpp"
#include "ai_sapiens_sim2real/mode_runtime/mode_state_machine.hpp"

using ai_sapiens_sim2real::ModeStateMachine;
using ai_sapiens_sim2real::RootConfig;
using ai_sapiens_sim2real::RootConfigValidator;
using ai_sapiens_sim2real::SelectorsConfig;
using ai_sapiens_sim2real::SharedControlData;
using ai_sapiens_sim2real::StateBehaviorsConfig;
using ai_sapiens_sim2real::StateMachineConfig;
using ai_sapiens_sim2real::TeleopConditionsConfig;

namespace
{
struct TestStateMachineConfig
{
  SharedControlData shared_data;
  StateBehaviorsConfig state_behaviors;
  TeleopConditionsConfig teleop_conditions;
  SelectorsConfig selectors;
  StateMachineConfig state_machine;

  explicit TestStateMachineConfig(const std::string & yaml)
  {
    shared_data.joint_map.controller_joint_names = {"joint"};
    const auto node = YAML::Load(yaml);
    state_behaviors = StateBehaviorsConfig::from_yaml(node, shared_data);
    teleop_conditions = TeleopConditionsConfig::from_yaml(node);
    selectors = SelectorsConfig::from_yaml(node);
    state_machine = StateMachineConfig::from_yaml(node);
    RootConfigValidator(
      state_machine,
      state_behaviors,
      teleop_conditions,
      selectors).validate();
  }
};

ModeStateMachine make_runtime(const TestStateMachineConfig & config)
{
  return ModeStateMachine(
    config.state_machine,
    config.state_behaviors,
    config.teleop_conditions,
    config.selectors);
}

TestStateMachineConfig load_config(const std::string & yaml)
{
  return TestStateMachineConfig(yaml);
}

std::filesystem::path write_temp_config(const std::string & yaml)
{
  static int index = 0;
  const auto path = std::filesystem::temp_directory_path() /
    ("ai_sapiens_sim2real_root_config_test_" + std::to_string(++index) + ".yaml");

  std::ofstream file(path);
  file << yaml;
  return path;
}

std::string root_config_with_authority(const std::string & authority)
{
  return
    R"(
robot_joint_order: [joint]
teleop_input:
  plugin: test_plugin
  config: test_plugin.yaml
teleop_conditions:
  DampingRequested:
    input_code: 1
)"
    +
    authority +
    R"(
state_machine:
  initial: Damping
  states:
    Damping:
      run: damping
    Velocity:
      run: velocity_policy
    MimicSquat:
      run: mimic_run
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
  velocity_policy:
    kind: policy
    asset: test/velocity
  mimic_run:
    kind: mimic
    asset: test/mimic
)";
}

std::string root_config_with_teleop_timeouts(const std::string & timeout_fields)
{
  auto yaml = root_config_with_authority(
    R"(
authority:
  api_entry:
    allowed_from_states: [Damping, Velocity]
  default_velocity_state: Velocity
)"
  );
  const std::string marker = "  config: test_plugin.yaml\n";
  const auto marker_position = yaml.find(marker);
  if (marker_position == std::string::npos) {
    throw std::logic_error("teleop input config marker is missing from the test fixture");
  }
  yaml.insert(marker_position + marker.size(), timeout_fields);
  return yaml;
}

void write_test_plugin_config()
{
  const auto path = std::filesystem::temp_directory_path() / "test_plugin.yaml";
  std::ofstream file(path);
  file << "{}\n";
}

void expect_teleop_timeouts_rejected(
  const std::string & timeout_fields,
  const std::string & expected_error)
{
  write_test_plugin_config();
  RootConfig root_config(
    write_temp_config(root_config_with_teleop_timeouts(timeout_fields)));

  try {
    static_cast<void>(root_config.operator_command_input_options());
    FAIL() << "Expected teleop timeout configuration to be rejected";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(expected_error), std::string::npos)
      << error.what();
  }
}

void expect_root_config_rejected(const std::string & authority)
{
  EXPECT_THROW(RootConfig(write_temp_config(root_config_with_authority(authority))),
      std::runtime_error);
}
}  // namespace


TEST(RootConfig, ReadsAuthorityConfig)
{
  const auto path =
    write_temp_config(
    R"(
robot_joint_order: [joint]
teleop_input:
  plugin: test_plugin
  config: test_plugin.yaml
teleop_conditions:
  DampingRequested:
    input_code: 1
authority:
  api_entry:
    warmup_duration: 2.5
    velocity_neutral_threshold: 0.2
    allowed_from_states: [Damping, Velocity]
  default_velocity_state: Velocity
state_machine:
  initial: Damping
  states:
    Damping:
      run: damping
    ReadyPose:
      run: damping
    Velocity:
      run: velocity_policy
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
  velocity_policy:
    kind: policy
    asset: test/velocity
)");

  RootConfig root_config(path);
  const auto & authority_config = root_config.authority_config();

  ASSERT_TRUE(authority_config.api_entry_warmup_duration);
  ASSERT_TRUE(authority_config.api_entry_velocity_neutral_threshold);
  EXPECT_DOUBLE_EQ(*authority_config.api_entry_warmup_duration, 2.5);
  EXPECT_FLOAT_EQ(*authority_config.api_entry_velocity_neutral_threshold, 0.2f);
  EXPECT_EQ(
    authority_config.api_entry_allowed_from_states,
    std::vector<std::string>({"Damping", "Velocity"}));
  EXPECT_EQ(authority_config.default_velocity_state, "Velocity");
}

TEST(RootConfig, TeleopVelocityTimeoutDefaultsToInputTimeout)
{
  write_test_plugin_config();
  RootConfig root_config(
    write_temp_config(root_config_with_teleop_timeouts("  timeout: 0.7\n")));

  const auto options = root_config.operator_command_input_options();

  EXPECT_DOUBLE_EQ(options.teleop_input_timeout, 0.7);
  EXPECT_DOUBLE_EQ(options.teleop_vel_command_timeout, 0.7);
}

TEST(RootConfig, ReadsExplicitTeleopVelocityTimeout)
{
  write_test_plugin_config();
  RootConfig root_config(
    write_temp_config(
      root_config_with_teleop_timeouts(
        "  vel_command_timeout: 0.1\n"
        "  timeout: 0.2\n")));

  const auto options = root_config.operator_command_input_options();

  EXPECT_DOUBLE_EQ(options.teleop_input_timeout, 0.2);
  EXPECT_DOUBLE_EQ(options.teleop_vel_command_timeout, 0.1);
}

TEST(RootConfig, RejectsInvalidTeleopTimeoutsWithConfigPaths)
{
  expect_teleop_timeouts_rejected(
    "  timeout: 0.0\n", "teleop_input.timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  timeout: -0.1\n", "teleop_input.timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  timeout: .nan\n", "teleop_input.timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  timeout: .inf\n", "teleop_input.timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  vel_command_timeout: 0.0\n  timeout: 1.0\n",
    "teleop_input.vel_command_timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  vel_command_timeout: -0.1\n  timeout: 1.0\n",
    "teleop_input.vel_command_timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  vel_command_timeout: .nan\n  timeout: 1.0\n",
    "teleop_input.vel_command_timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  vel_command_timeout: .inf\n  timeout: 1.0\n",
    "teleop_input.vel_command_timeout must be finite and positive");
  expect_teleop_timeouts_rejected(
    "  vel_command_timeout: 1.1\n  timeout: 1.0\n",
    "teleop_input.vel_command_timeout must not exceed teleop_input.timeout");
}

TEST(AuthorityConfigValidation, AcceptsZeroWarmupAndNeutralThreshold)
{
  const auto path = write_temp_config(
    root_config_with_authority(
      R"(
authority:
  api_entry:
    warmup_duration: 0.0
    velocity_neutral_threshold: 0.0
    allowed_from_states: [Damping, Velocity]
  default_velocity_state: Velocity
)"));

  EXPECT_NO_THROW((void)RootConfig{path});
}

TEST(AuthorityConfigValidation, RejectsInvalidNumericValues)
{
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    warmup_duration: -0.1
    allowed_from_states: [Damping]
  default_velocity_state: Velocity
)");
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    warmup_duration: .nan
    allowed_from_states: [Damping]
  default_velocity_state: Velocity
)");
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    velocity_neutral_threshold: -0.1
    allowed_from_states: [Damping]
  default_velocity_state: Velocity
)");
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    velocity_neutral_threshold: .inf
    allowed_from_states: [Damping]
  default_velocity_state: Velocity
)");
}

TEST(AuthorityConfigValidation, RejectsEmptyOrDuplicateEntryStates)
{
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    allowed_from_states: []
  default_velocity_state: Velocity
)");
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    allowed_from_states: [Damping, Damping]
  default_velocity_state: Velocity
)");
}

TEST(AuthorityConfigValidation, RequiresNonMimicPolicyDefaultVelocityState)
{
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    allowed_from_states: [Damping]
  default_velocity_state: Damping
)");
  expect_root_config_rejected(
    R"(
authority:
  api_entry:
    allowed_from_states: [Damping]
  default_velocity_state: MimicSquat
)");
}

TEST(ModeStateMachine, DoesNotRequireMimicParentState)
{
  auto config =
    load_config(
    R"(
teleop_conditions:
  DampingRequested:
    input_code: 1
state_machine:
  initial: Damping
  states:
    Damping:
      run: damping
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
)");

  auto runtime = make_runtime(config);
  EXPECT_FALSE(runtime.is_mimic_state("Damping"));
  EXPECT_FALSE(runtime.is_mimic_state("Mimic"));
}

TEST(ModeStateMachine, TreatsStandaloneMimicBehaviorAsMimicState)
{
  auto config =
    load_config(
    R"(
teleop_conditions:
  DampingRequested:
    input_code: 1
state_machine:
  initial: Damping
  states:
    Damping:
      run: damping
    SoloMotion:
      run: solo_motion
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
  solo_motion:
    kind: mimic
    asset: mimic/solo
)");

  auto runtime = make_runtime(config);
  EXPECT_TRUE(runtime.behavior_for_state("SoloMotion").mimic);
  EXPECT_TRUE(runtime.is_mimic_state("SoloMotion"));
}

TEST(ModeStateMachine, FollowsSelectorTargetAndParentStateChain)
{
  auto config =
    load_config(
    R"(
teleop_conditions:
  MotionRequested:
    input_code: 4
selectors:
  motion_selector:
    table:
      200: Squat
state_machine:
  initial: Velocity
  states:
    Velocity:
      run: damping
      transitions:
        - when: MotionRequested
          select: motion_selector
    MotionGroup:
      abstract: true
    Squat:
      parent: MotionGroup
      run: squat
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
  squat:
    kind: mimic
    asset: mimic/squat
)");

  auto runtime = make_runtime(config);
  EXPECT_TRUE(runtime.is_mimic_state("Squat"));
  EXPECT_FALSE(runtime.is_mimic_state("Velocity"));
}

TEST(StateMachineConfigValidation, RejectsSelectorTargetMissingState)
{
  EXPECT_THROW(load_config(
    R"(
teleop_conditions:
  MotionRequested:
    input_code: 4
selectors:
  motion_selector:
    table:
      200: MissingState
state_machine:
  initial: Velocity
  states:
    Velocity:
      run: damping
      transitions:
        - when: MotionRequested
          select: motion_selector
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
)"),
    std::runtime_error);
}

TEST(StateMachineConfigValidation, RejectsMissingParentState)
{
  EXPECT_THROW(load_config(
    R"(
teleop_conditions:
  DampingRequested:
    input_code: 1
state_machine:
  initial: Damping
  states:
    Damping:
      run: damping
    Squat:
      parent: MotionGroup
      run: damping
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
)"),
    std::runtime_error);
}

TEST(StateMachineConfigValidation, RejectsNonAbstractParentState)
{
  EXPECT_THROW(load_config(
    R"(
teleop_conditions:
  DampingRequested:
    input_code: 1
state_machine:
  initial: Damping
  states:
    Damping:
      run: damping
    MotionGroup:
      run: damping
    Squat:
      parent: MotionGroup
      run: damping
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
)"),
    std::runtime_error);
}

TEST(StateMachineConfigValidation, RejectsUnsupportedBehaviorKind)
{
  EXPECT_THROW(load_config(
    R"(
teleop_conditions:
  DampingRequested:
    input_code: 1
state_machine:
  initial: Damping
  states:
    Damping:
      run: bad_behavior
state_behaviors:
  bad_behavior:
    kind: unsupported
)"),
    std::runtime_error);
}
