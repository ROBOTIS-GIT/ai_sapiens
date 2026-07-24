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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__MODE_STATE_MACHINE_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__MODE_STATE_MACHINE_HPP_

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/config/root_sections/selectors_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_behaviors_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_machine_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/teleop_conditions_config.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_command.hpp"

namespace ai_sapiens_sim2real
{

struct StateEntryRequest
{
  std::string name;
  BehaviorKind behavior_kind;
};

struct ModeFsmTeleopInput
{
  bool available{false};
  uint16_t input_code{0};
  uint16_t selector_code{0};
};

class ModeStateMachine
{
public:
  ModeStateMachine() = default;

  ModeStateMachine(
    const StateMachineConfig & state_machine,
    const StateBehaviorsConfig & state_behaviors,
    const TeleopConditionsConfig & teleop_conditions,
    const SelectorsConfig & selectors);
  void reset(
    const StateMachineConfig & state_machine,
    const StateBehaviorsConfig & state_behaviors,
    const TeleopConditionsConfig & teleop_conditions,
    const SelectorsConfig & selectors);

  // Evaluate transitions with their configured trigger semantics. An edge
  // transition fires only on the tick the input newly satisfies the condition
  // (rising edge vs the previous tick); a held switch never re-triggers.
  std::optional<StateEntryRequest> resolve_state_request_with_trigger_rules(
    const std::string & active_state,
    const ModeFsmTeleopInput & current_teleop_input,
    const ModeFsmTeleopInput & previous_teleop_input) const;
  // Ignore edge requirements: map the current input level to a state request
  // (used for handoff, kill-switch, and startup checks).
  std::optional<StateEntryRequest> resolve_state_request_by_level_match(
    const std::string & active_state,
    const ModeFsmTeleopInput & current_teleop_input) const;
  std::optional<StateEntryRequest> resolve_state_request_from_state_name(
    const std::string & active_state,
    const std::string & target_state) const;
  bool does_teleop_input_match_condition(
    const std::string & condition_name,
    const ModeFsmTeleopInput & current_teleop_input) const;

  // Read-only FSM queries used by services, status, and mode execution.
  const std::string & initial_state_name() const;
  const ModeState * find_state(const std::string & state_name) const;
  std::vector<std::string> concrete_state_names() const;
  std::string parent_state_name(const std::string & state_name) const;
  const StateBehavior & behavior_for_state(const std::string & state_name) const;
  std::string behavior_name_for_state(const std::string & state_name) const;
  std::string policy_for_state(const std::string & state_name) const;
  BehaviorKind behavior_kind_for_state(const std::string & state_name) const;
  bool is_mimic_state(const std::string & state_name) const;

private:
  template<typename TransitionPredicate>
  std::optional<StateEntryRequest> resolve_state_request_from_state(
    const std::string & state_name,
    const ModeFsmTeleopInput & current_teleop_input,
    TransitionPredicate is_transition_triggered) const
  {
    const auto & state = require_state(state_name);
    for (const auto & transition : state.transitions) {
      if (!is_transition_triggered(transition)) {
        continue;
      }

      const auto target = resolve_transition_target(transition, current_teleop_input);
      if (target && !target->empty()) {
        return StateEntryRequest{*target, behavior_kind_for_state(*target)};
      }
    }

    if (!state.parent.empty()) {
      return resolve_state_request_from_state(
        state.parent, current_teleop_input, is_transition_triggered);
    }

    return std::nullopt;
  }

  std::optional<std::string> resolve_transition_target(
    const ModeTransition & transition,
    const ModeFsmTeleopInput & current_teleop_input) const
  {
    if (!transition.target_state.empty()) {
      return transition.target_state;
    }
    if (transition.selector_name.empty()) {
      return std::nullopt;
    }

    // Selectors read the live teleop selector code.
    return resolve_selector_target_for_code(
      transition.selector_name, current_teleop_input.selector_code);
  }

  bool is_transition_triggered_by_teleop_input(
    const ModeTransition & transition,
    const ModeFsmTeleopInput & current_teleop_input,
    const ModeFsmTeleopInput & previous_teleop_input) const;
  bool is_transition_matched_by_current_teleop_input(
    const ModeTransition & transition,
    const ModeFsmTeleopInput & current_teleop_input) const;

  // Teleop condition predicates.
  const TeleopCondition * find_condition_for_transition(
    const ModeTransition & transition) const;
  bool is_teleop_condition_satisfied_by(
    const ModeTransition & transition,
    const ModeFsmTeleopInput & current_teleop_input,
    const ModeFsmTeleopInput & previous_teleop_input) const;
  bool is_teleop_condition_newly_satisfied_by(
    const TeleopCondition & condition,
    const ModeFsmTeleopInput & previous_teleop_input) const;
  bool does_teleop_input_match_condition(
    const TeleopCondition & condition,
    const ModeFsmTeleopInput & current_teleop_input) const;

  // Selector expansion.
  std::optional<std::string> resolve_selector_target_for_code(
    const std::string & selector_name,
    uint16_t selector_code) const;

  // Config accessors.
  const StateMachineConfig & require_state_machine() const;
  const StateBehaviorsConfig & require_state_behaviors() const;
  const TeleopConditionsConfig & require_teleop_conditions() const;
  const SelectorsConfig & require_selectors() const;
  const ModeState & require_state(const std::string & state_name) const;
  const TeleopCondition * find_condition(const std::string & condition_name) const;

  const StateMachineConfig * state_machine_{nullptr};
  const StateBehaviorsConfig * state_behaviors_{nullptr};
  const TeleopConditionsConfig * teleop_conditions_{nullptr};
  const SelectorsConfig * selectors_{nullptr};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__MODE_STATE_MACHINE_HPP_
