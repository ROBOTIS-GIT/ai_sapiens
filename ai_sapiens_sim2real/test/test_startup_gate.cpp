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

#include <cstdint>
#include <string>

#include "ai_sapiens_sim2real/config/root_config_validator.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_machine_config.hpp"
#include "ai_sapiens_sim2real/mode_runtime/mode_state_machine.hpp"

using ai_sapiens_sim2real::ModeStateMachine;
using ai_sapiens_sim2real::ModeFsmTeleopInput;
using ai_sapiens_sim2real::RootConfigValidator;
using ai_sapiens_sim2real::SelectorsConfig;
using ai_sapiens_sim2real::SharedControlData;
using ai_sapiens_sim2real::StateBehaviorsConfig;
using ai_sapiens_sim2real::StateMachineConfig;
using ai_sapiens_sim2real::TeleopConditionsConfig;

namespace
{
// Condition names the startup gate checks; mirror the ModeController constants
// kDampingConditionName / kStartupReadyPoseConditionName.
constexpr const char * kDamping = "DampingRequested";
constexpr const char * kReadyPose = "ReadyPoseRequested";

struct TestRootSectionsConfig
{
  SharedControlData shared_data;
  StateBehaviorsConfig state_behaviors;
  TeleopConditionsConfig teleop_conditions;
  SelectorsConfig selectors;
  StateMachineConfig state_machine;

  explicit TestRootSectionsConfig(const std::string & yaml)
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

// input_code values match config/k1_config.yaml teleop_conditions.
TestRootSectionsConfig make_config()
{
  return TestRootSectionsConfig(
    R"(
teleop_conditions:
  DampingRequested:
    input_code: 1
  ReadyPoseRequested:
    input_code: 2
  VelocityRequested:
    input_code: 3
  MimicRequested:
    input_code: 4
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
}

ModeFsmTeleopInput input(uint16_t code, bool available = true)
{
  ModeFsmTeleopInput in;
  in.available = available;
  in.input_code = code;
  return in;
}

// Mirrors ModeController::is_startup_teleop_input_safe: the gate releases only
// when the live input matches the Damping or ReadyPose condition.
bool startup_input_is_safe(const ModeStateMachine & runtime, const ModeFsmTeleopInput & in)
{
  return runtime.does_teleop_input_match_condition(kDamping, in) ||
         runtime.does_teleop_input_match_condition(kReadyPose, in);
}
}  // namespace

TEST(StartupGate, DampingPositionReleasesGate)
{
  auto config = make_config();
  ModeStateMachine runtime(
    config.state_machine,
    config.state_behaviors,
    config.teleop_conditions,
    config.selectors);
  EXPECT_TRUE(startup_input_is_safe(runtime, input(1)));
}

TEST(StartupGate, ReadyPosePositionReleasesGate)
{
  auto config = make_config();
  ModeStateMachine runtime(
    config.state_machine,
    config.state_behaviors,
    config.teleop_conditions,
    config.selectors);
  EXPECT_TRUE(startup_input_is_safe(runtime, input(2)));
}

TEST(StartupGate, VelocityMimicAndNeutralHoldGate)
{
  auto config = make_config();
  ModeStateMachine runtime(
    config.state_machine,
    config.state_behaviors,
    config.teleop_conditions,
    config.selectors);
  EXPECT_FALSE(startup_input_is_safe(runtime, input(3)));  // VelocityRequested
  EXPECT_FALSE(startup_input_is_safe(runtime, input(4)));  // MimicRequested
  EXPECT_FALSE(startup_input_is_safe(runtime, input(0)));  // no condition
}

TEST(StartupGate, UnavailableInputNeverReleasesGate)
{
  auto config = make_config();
  ModeStateMachine runtime(
    config.state_machine,
    config.state_behaviors,
    config.teleop_conditions,
    config.selectors);
  // A stale watchdog must hold the gate even if the last code was a safe one.
  EXPECT_FALSE(startup_input_is_safe(runtime, input(1, /*available=*/false)));
  EXPECT_FALSE(startup_input_is_safe(runtime, input(2, /*available=*/false)));
}
