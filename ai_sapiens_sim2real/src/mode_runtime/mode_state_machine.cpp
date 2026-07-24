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
// Author: Kiwoong Park

#include "ai_sapiens_sim2real/mode_runtime/mode_state_machine.hpp"

namespace ai_sapiens_sim2real
{

ModeStateMachine::ModeStateMachine(
  const StateMachineConfig & state_machine,
  const StateBehaviorsConfig & state_behaviors,
  const TeleopConditionsConfig & teleop_conditions,
  const SelectorsConfig & selectors)
: state_machine_(&state_machine),
  state_behaviors_(&state_behaviors),
  teleop_conditions_(&teleop_conditions),
  selectors_(&selectors)
{
}

void ModeStateMachine::reset(
  const StateMachineConfig & state_machine,
  const StateBehaviorsConfig & state_behaviors,
  const TeleopConditionsConfig & teleop_conditions,
  const SelectorsConfig & selectors)
{
  state_machine_ = &state_machine;
  state_behaviors_ = &state_behaviors;
  teleop_conditions_ = &teleop_conditions;
  selectors_ = &selectors;
}

std::optional<StateEntryRequest> ModeStateMachine::resolve_state_request_with_trigger_rules(
  const std::string & active_state,
  const ModeFsmTeleopInput & current_teleop_input,
  const ModeFsmTeleopInput & previous_teleop_input) const
{
  return resolve_state_request_from_state(
    active_state,
    current_teleop_input,
    [&](const ModeTransition & transition) {
      return is_transition_triggered_by_teleop_input(
        transition, current_teleop_input, previous_teleop_input);
    });
}

std::optional<StateEntryRequest> ModeStateMachine::resolve_state_request_by_level_match(
  const std::string & active_state,
  const ModeFsmTeleopInput & current_teleop_input) const
{
  return resolve_state_request_from_state(
    active_state,
    current_teleop_input,
    [&](const ModeTransition & transition) {
      return is_transition_matched_by_current_teleop_input(transition, current_teleop_input);
    });
}

std::optional<StateEntryRequest> ModeStateMachine::resolve_state_request_from_state_name(
  const std::string & /*active_state*/,
  const std::string & target_state) const
{
  const auto state = find_state(target_state);
  if (state == nullptr || state->abstract) {
    return std::nullopt;
  }

  return StateEntryRequest{target_state, behavior_kind_for_state(target_state)};
}

bool ModeStateMachine::does_teleop_input_match_condition(
  const std::string & condition_name,
  const ModeFsmTeleopInput & current_teleop_input) const
{
  const auto condition = find_condition(condition_name);
  if (condition == nullptr) {
    return false;
  }

  return does_teleop_input_match_condition(*condition, current_teleop_input);
}

const std::string & ModeStateMachine::initial_state_name() const
{
  return require_state_machine().initial_state_name();
}

const ModeState * ModeStateMachine::find_state(const std::string & state_name) const
{
  return require_state_machine().find_state(state_name);
}

std::vector<std::string> ModeStateMachine::concrete_state_names() const
{
  return require_state_machine().concrete_state_names();
}

std::string ModeStateMachine::parent_state_name(const std::string & state_name) const
{
  const auto state = find_state(state_name);
  return state == nullptr ? std::string{} : state->parent;
}

std::string ModeStateMachine::behavior_name_for_state(const std::string & state_name) const
{
  const auto state = find_state(state_name);
  return state == nullptr ? std::string{} : state->run;
}

bool ModeStateMachine::is_mimic_state(const std::string & state_name) const
{
  const auto state = find_state(state_name);
  if (state == nullptr) {
    return false;
  }

  return require_state_behaviors().require_behavior(state->run).mimic;
}

bool ModeStateMachine::is_transition_triggered_by_teleop_input(
  const ModeTransition & transition,
  const ModeFsmTeleopInput & current_teleop_input,
  const ModeFsmTeleopInput & previous_teleop_input) const
{
  return is_teleop_condition_satisfied_by(transition, current_teleop_input, previous_teleop_input);
}

bool ModeStateMachine::is_transition_matched_by_current_teleop_input(
  const ModeTransition & transition,
  const ModeFsmTeleopInput & current_teleop_input) const
{
  const auto condition = find_condition_for_transition(transition);
  return condition != nullptr &&
         does_teleop_input_match_condition(*condition, current_teleop_input);
}

const TeleopCondition * ModeStateMachine::find_condition_for_transition(
  const ModeTransition & transition) const
{
  return find_condition(transition.condition_name);
}

bool ModeStateMachine::is_teleop_condition_satisfied_by(
  const ModeTransition & transition,
  const ModeFsmTeleopInput & current_teleop_input,
  const ModeFsmTeleopInput & previous_teleop_input) const
{
  const auto condition = find_condition_for_transition(transition);
  if (condition == nullptr ||
    !does_teleop_input_match_condition(*condition, current_teleop_input))
  {
    return false;
  }
  if (transition.trigger == ModeConditionTrigger::Edge) {
    return is_teleop_condition_newly_satisfied_by(*condition, previous_teleop_input);
  }

  return true;
}

bool ModeStateMachine::is_teleop_condition_newly_satisfied_by(
  const TeleopCondition & condition,
  const ModeFsmTeleopInput & previous_teleop_input) const
{
  // The previous tick must be a valid sample that did not satisfy the
  // condition: a held switch, a boot-time position, or a change spanning a
  // teleop dropout never counts as a fresh edge.
  return previous_teleop_input.available &&
         !does_teleop_input_match_condition(condition, previous_teleop_input);
}

bool ModeStateMachine::does_teleop_input_match_condition(
  const TeleopCondition & condition,
  const ModeFsmTeleopInput & current_teleop_input) const
{
  if (!current_teleop_input.available) {
    return false;
  }

  return current_teleop_input.input_code == condition.input_code;
}

std::optional<std::string> ModeStateMachine::resolve_selector_target_for_code(
  const std::string & selector_name,
  uint16_t selector_code) const
{
  return require_selectors().find_target_state_for_selector_code(selector_name, selector_code);
}

const StateBehavior & ModeStateMachine::behavior_for_state(const std::string & state_name) const
{
  const auto & state = require_state(state_name);
  return require_state_behaviors().require_behavior(state.run);
}

std::string ModeStateMachine::policy_for_state(const std::string & state_name) const
{
  const auto & behavior = behavior_for_state(state_name);
  if (behavior.policy_name.empty()) {
    throw std::runtime_error("State '" + state_name + "' does not run a policy behavior");
  }

  return behavior.policy_name;
}

BehaviorKind ModeStateMachine::behavior_kind_for_state(const std::string & state_name) const
{
  return behavior_for_state(state_name).kind;
}

const StateMachineConfig & ModeStateMachine::require_state_machine() const
{
  if (state_machine_ == nullptr) {
    throw std::runtime_error("ModeStateMachine is not configured");
  }

  return *state_machine_;
}

const StateBehaviorsConfig & ModeStateMachine::require_state_behaviors() const
{
  if (state_behaviors_ == nullptr) {
    throw std::runtime_error("ModeStateMachine is not configured");
  }

  return *state_behaviors_;
}

const TeleopConditionsConfig & ModeStateMachine::require_teleop_conditions() const
{
  if (teleop_conditions_ == nullptr) {
    throw std::runtime_error("ModeStateMachine is not configured");
  }

  return *teleop_conditions_;
}

const SelectorsConfig & ModeStateMachine::require_selectors() const
{
  if (selectors_ == nullptr) {
    throw std::runtime_error("ModeStateMachine is not configured");
  }

  return *selectors_;
}

const ModeState & ModeStateMachine::require_state(const std::string & state_name) const
{
  const auto state = find_state(state_name);
  if (state == nullptr) {
    throw std::runtime_error("Unknown state: " + state_name);
  }

  return *state;
}

const TeleopCondition * ModeStateMachine::find_condition(
  const std::string & condition_name) const
{
  return require_teleop_conditions().find_condition(condition_name);
}

}  // namespace ai_sapiens_sim2real
