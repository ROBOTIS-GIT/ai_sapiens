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

#include "ai_sapiens_sim2real/config/root_config_validator.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace ai_sapiens_sim2real
{

RootConfigValidator::RootConfigValidator(
  const StateMachineConfig & state_machine,
  const StateBehaviorsConfig & state_behaviors,
  const TeleopConditionsConfig & teleop_conditions,
  const SelectorsConfig & selectors)
: state_machine_(state_machine),
  state_behaviors_(state_behaviors),
  teleop_conditions_(teleop_conditions),
  selectors_(selectors)
{
}

RootConfigValidator::RootConfigValidator(
  const StateMachineConfig & state_machine,
  const StateBehaviorsConfig & state_behaviors,
  const TeleopConditionsConfig & teleop_conditions,
  const SelectorsConfig & selectors,
  const AuthorityConfig & authority)
: RootConfigValidator(
    state_machine,
    state_behaviors,
    teleop_conditions,
    selectors)
{
  authority_ = &authority;
}

void RootConfigValidator::validate() const
{
  validate_state_machine_links();
  validate_authority_links();
}

void RootConfigValidator::validate_state_machine_links() const
{
  require_initial_state_is_non_abstract();
  require_states_have_known_behaviors();
  require_transitions_use_known_conditions();
  require_transitions_use_known_selectors();
  require_transition_targets_are_non_abstract();
  require_selector_targets_are_non_abstract();
  require_parent_states_are_abstract();
  require_parent_chains_have_no_cycles();
}

void RootConfigValidator::validate_authority_links() const
{
  if (authority_ == nullptr) {
    return;
  }

  require_api_entry_numbers_are_valid();
  require_api_entry_source_states_are_non_abstract();
  require_api_entry_source_states_are_unique();
  require_existing_non_abstract_state(
    authority_->default_velocity_state,
    "authority.default_velocity_state");
  require_default_velocity_state_is_non_mimic_policy();
}

void RootConfigValidator::require_initial_state_is_non_abstract() const
{
  require_existing_non_abstract_state(
    state_machine_.initial_state_name(),
    "state_machine.initial");
}

void RootConfigValidator::require_states_have_known_behaviors() const
{
  state_machine_.for_each_state(
    [&](const auto & name, const auto & state) {
      if (!state.abstract) {
        require_known_behavior(state.run, state_path(name) + ".run");
      }
    });
}

void RootConfigValidator::require_transitions_use_known_conditions() const
{
  for_each_transition(
    [&](const auto & state_name, const auto & transition, auto index) {
      require_known_condition(
        transition.condition_name,
        transition_condition_path(state_name, index));
    });
}

void RootConfigValidator::require_transitions_use_known_selectors() const
{
  for_each_transition(
    [&](const auto & state_name, const auto & transition, auto index) {
      if (!transition.selector_name.empty()) {
        require_known_selector(
          transition.selector_name,
          transition_selector_path(state_name, index));
      }
    });
}

void RootConfigValidator::require_transition_targets_are_non_abstract() const
{
  for_each_transition(
    [&](const auto & state_name, const auto & transition, auto index) {
      if (!transition.target_state.empty()) {
        require_existing_non_abstract_state(
          transition.target_state,
          transition_to_path(state_name, index));
      }
    });
}

void RootConfigValidator::require_selector_targets_are_non_abstract() const
{
  for_each_selector_target(
    [&](
      const auto & selector_name,
      auto selector_code,
      const auto & target_state) {
      require_existing_non_abstract_state(
        target_state,
        selector_table_entry_path(selector_name, selector_code));
    });
}

void RootConfigValidator::require_parent_states_are_abstract() const
{
  state_machine_.for_each_state(
    [&](const auto & name, const auto & state) {
      require_parent_state_is_abstract(name, state);
    });
}

void RootConfigValidator::require_parent_chains_have_no_cycles() const
{
  state_machine_.for_each_state(
    [&](const auto & name, const auto & state) {
      require_parent_chain_has_no_cycle(name, state);
    });
}

void RootConfigValidator::require_parent_state_is_abstract(
  const std::string & state_name,
  const ModeState & state) const
{
  if (state.parent.empty()) {
    return;
  }

  const auto path = parent_state_path(state_name);
  const auto * parent = require_state(state.parent, path);
  require_abstract_state(state.parent, *parent, path);
}

void RootConfigValidator::require_parent_chain_has_no_cycle(
  const std::string & state_name,
  const ModeState & state) const
{
  std::unordered_set<std::string> visited{state_name};
  std::string parent_name = state.parent;

  while (!parent_name.empty()) {
    if (!visited.insert(parent_name).second) {
      throw std::runtime_error(
        state_path(state_name) + " has a cyclic parent chain through '" + parent_name + "'");
    }

    const auto parent = state_machine_.find_state(parent_name);
    parent_name = parent == nullptr ? std::string{} : parent->parent;
  }
}

void RootConfigValidator::require_api_entry_source_states_are_non_abstract() const
{
  if (authority_->api_entry_allowed_from_states.empty()) {
    throw std::runtime_error("authority.api_entry.allowed_from_states must not be empty");
  }

  for (std::size_t i = 0; i < authority_->api_entry_allowed_from_states.size(); ++i) {
    require_existing_non_abstract_state(
      authority_->api_entry_allowed_from_states[i],
      "authority.api_entry.allowed_from_states[" + std::to_string(i) + "]");
  }
}

void RootConfigValidator::require_api_entry_numbers_are_valid() const
{
  if (authority_->api_entry_warmup_duration) {
    const double duration = *authority_->api_entry_warmup_duration;
    if (!std::isfinite(duration) || duration < 0.0) {
      throw std::runtime_error(
        "authority.api_entry.warmup_duration must be finite and non-negative");
    }
  }

  if (authority_->api_entry_velocity_neutral_threshold) {
    const float threshold = *authority_->api_entry_velocity_neutral_threshold;
    if (!std::isfinite(threshold) || threshold < 0.0F) {
      throw std::runtime_error(
        "authority.api_entry.velocity_neutral_threshold must be finite and non-negative");
    }
  }
}

void RootConfigValidator::require_api_entry_source_states_are_unique() const
{
  std::unordered_set<std::string> seen_states;
  for (std::size_t i = 0; i < authority_->api_entry_allowed_from_states.size(); ++i) {
    const auto & state_name = authority_->api_entry_allowed_from_states[i];
    if (!seen_states.insert(state_name).second) {
      throw std::runtime_error(
        "authority.api_entry.allowed_from_states[" + std::to_string(i) +
        "] duplicates state '" + state_name + "'");
    }
  }
}

void RootConfigValidator::require_default_velocity_state_is_non_mimic_policy() const
{
  const auto * state = require_state(
    authority_->default_velocity_state,
    "authority.default_velocity_state");
  const auto & behavior = state_behaviors_.require_behavior(state->run);
  if (behavior.kind != BehaviorKind::Policy || behavior.mimic) {
    throw std::runtime_error(
      "authority.default_velocity_state must reference a non-mimic policy state");
  }
}

void RootConfigValidator::for_each_transition(
  const std::function<void(
    const std::string &,
    const ModeTransition &,
    std::size_t)> & visitor) const
{
  state_machine_.for_each_state(
    [&](const auto & name, const auto & state) {
      for (std::size_t i = 0; i < state.transitions.size(); ++i) {
        visitor(name, state.transitions[i], i);
      }
    });
}

void RootConfigValidator::for_each_selector_target(
  const std::function<void(
    const std::string &,
    uint16_t,
    const std::string &)> & visitor) const
{
  selectors_.for_each_selector(
    [&](const auto & selector_name, const auto & selector) {
      for (const auto & [selector_code, target_state] : selector) {
        visitor(selector_name, selector_code, target_state);
      }
    });
}

std::string RootConfigValidator::state_path(const std::string & state_name) const
{
  return "state_machine.states." + state_name;
}

std::string RootConfigValidator::transition_condition_path(
  const std::string & state_name,
  std::size_t transition_index) const
{
  return state_path(state_name) + ".transitions[" + std::to_string(transition_index) + "].when";
}

std::string RootConfigValidator::transition_selector_path(
  const std::string & state_name,
  std::size_t transition_index) const
{
  return state_path(state_name) + ".transitions[" + std::to_string(transition_index) + "].select";
}

std::string RootConfigValidator::transition_to_path(
  const std::string & state_name,
  std::size_t transition_index) const
{
  return state_path(state_name) + ".transitions[" + std::to_string(transition_index) + "].to";
}

std::string RootConfigValidator::selector_table_entry_path(
  const std::string & selector_name,
  uint16_t selector_value) const
{
  return "selectors." + selector_name + ".table[" + std::to_string(selector_value) + "]";
}

std::string RootConfigValidator::parent_state_path(const std::string & state_name) const
{
  return state_path(state_name) + ".parent";
}

void RootConfigValidator::require_known_behavior(
  const std::string & behavior_name,
  const std::string & path) const
{
  if (behavior_name.empty()) {
    throw std::runtime_error(path + " is required");
  }

  if (!state_behaviors_.has_behavior(behavior_name)) {
    throw std::runtime_error(path + " references unknown state_behavior '" + behavior_name + "'");
  }
}

void RootConfigValidator::require_known_condition(
  const std::string & condition_name,
  const std::string & path) const
{
  if (!teleop_conditions_.has_condition(condition_name)) {
    throw std::runtime_error(
      path + " references unknown teleop condition '" + condition_name + "'");
  }
}

void RootConfigValidator::require_known_selector(
  const std::string & selector_name,
  const std::string & path) const
{
  if (!selectors_.has_selector(selector_name)) {
    throw std::runtime_error(path + " references unknown selector '" + selector_name + "'");
  }
}

void RootConfigValidator::require_existing_non_abstract_state(
  const std::string & state_name,
  const std::string & path) const
{
  const auto * state = require_state(state_name, path);
  if (state->abstract) {
    throw std::runtime_error(path + " references abstract state '" + state_name + "'");
  }
}

void RootConfigValidator::require_abstract_state(
  const std::string & state_name,
  const ModeState & state,
  const std::string & path) const
{
  if (!state.abstract) {
    throw std::runtime_error(path + " references non-abstract state '" + state_name + "'");
  }
}

const ModeState * RootConfigValidator::require_state(
  const std::string & state_name,
  const std::string & path) const
{
  const auto state = state_machine_.find_state(state_name);
  if (state == nullptr) {
    throw std::runtime_error(path + " references unknown state '" + state_name + "'");
  }

  return state;
}

}  // namespace ai_sapiens_sim2real
