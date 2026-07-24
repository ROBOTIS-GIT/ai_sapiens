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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__ROOT_CONFIG_VALIDATOR_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__ROOT_CONFIG_VALIDATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "ai_sapiens_sim2real/config/authority_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/selectors_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_behaviors_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_machine_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/teleop_conditions_config.hpp"

namespace ai_sapiens_sim2real
{

// Validates references that cross root config sections. Individual section
// parsers validate their own syntax; this class checks that the parsed sections
// agree with each other.
class RootConfigValidator
{
public:
  RootConfigValidator(
    const StateMachineConfig & state_machine,
    const StateBehaviorsConfig & state_behaviors,
    const TeleopConditionsConfig & teleop_conditions,
    const SelectorsConfig & selectors);

  RootConfigValidator(
    const StateMachineConfig & state_machine,
    const StateBehaviorsConfig & state_behaviors,
    const TeleopConditionsConfig & teleop_conditions,
    const SelectorsConfig & selectors,
    const AuthorityConfig & authority);

  void validate() const;

private:
  void validate_state_machine_links() const;
  void validate_authority_links() const;

  void require_initial_state_is_non_abstract() const;

  void require_states_have_known_behaviors() const;

  void require_transitions_use_known_conditions() const;
  void require_transitions_use_known_selectors() const;
  void require_transition_targets_are_non_abstract() const;
  void require_selector_targets_are_non_abstract() const;

  void require_parent_states_are_abstract() const;
  void require_parent_chains_have_no_cycles() const;

  void require_parent_state_is_abstract(
    const std::string & state_name,
    const ModeState & state) const;
  void require_parent_chain_has_no_cycle(
    const std::string & state_name,
    const ModeState & state) const;

  void require_api_entry_numbers_are_valid() const;
  void require_api_entry_source_states_are_non_abstract() const;
  void require_api_entry_source_states_are_unique() const;
  void require_default_velocity_state_is_non_mimic_policy() const;

  void for_each_transition(
    const std::function<void(
      const std::string &,
      const ModeTransition &,
      std::size_t)> & visitor) const;
  void for_each_selector_target(
    const std::function<void(
      const std::string &,
      uint16_t,
      const std::string &)> & visitor) const;

  std::string state_path(const std::string & state_name) const;
  std::string transition_condition_path(
    const std::string & state_name,
    std::size_t transition_index) const;
  std::string transition_selector_path(
    const std::string & state_name,
    std::size_t transition_index) const;
  std::string transition_to_path(
    const std::string & state_name,
    std::size_t transition_index) const;
  std::string selector_table_entry_path(
    const std::string & selector_name,
    uint16_t selector_value) const;
  std::string parent_state_path(const std::string & state_name) const;

  void require_known_behavior(
    const std::string & behavior_name,
    const std::string & path) const;
  void require_known_condition(
    const std::string & condition_name,
    const std::string & path) const;
  void require_known_selector(
    const std::string & selector_name,
    const std::string & path) const;
  void require_existing_non_abstract_state(
    const std::string & state_name,
    const std::string & path) const;
  void require_abstract_state(
    const std::string & state_name,
    const ModeState & state,
    const std::string & path) const;
  const ModeState * require_state(
    const std::string & state_name,
    const std::string & path) const;

  const StateMachineConfig & state_machine_;
  const StateBehaviorsConfig & state_behaviors_;
  const TeleopConditionsConfig & teleop_conditions_;
  const SelectorsConfig & selectors_;
  const AuthorityConfig * authority_{nullptr};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__ROOT_CONFIG_VALIDATOR_HPP_
