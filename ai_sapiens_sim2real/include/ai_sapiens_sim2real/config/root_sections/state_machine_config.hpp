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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__STATE_MACHINE_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__STATE_MACHINE_CONFIG_HPP_

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

enum class ModeConditionTrigger
{
  Level,
  Edge,
};

// States form a lightweight FSM. Abstract states share transitions with children.
struct ModeTransition
{
  std::string condition_name;
  ModeConditionTrigger trigger{ModeConditionTrigger::Level};
  std::string target_state;
  std::string selector_name;
};

struct ModeState
{
  std::string name;
  std::string parent;
  std::string run;
  bool abstract{false};
  std::vector<ModeTransition> transitions;
};

class StateMachineConfig
{
public:
  StateMachineConfig() = default;
  StateMachineConfig(
    std::string initial_state_name,
    std::unordered_map<std::string, ModeState> states);

  static StateMachineConfig from_yaml(const YAML::Node & config);

  const std::string & initial_state_name() const;
  const ModeState * find_state(const std::string & state_name) const;
  const ModeState & require_state(const std::string & state_name) const;
  bool has_state(const std::string & state_name) const;
  std::vector<std::string> concrete_state_names() const;
  void for_each_state(
    const std::function<void(const std::string &, const ModeState &)> & visitor) const;

private:
  struct TransitionDestination
  {
    std::string target_state;
    std::string selector_name;
  };

  YAML::Node read_state_machine_node(const YAML::Node & config) const;
  std::unordered_map<std::string, ModeState> read_state_configs(
    const YAML::Node & states_node) const;
  ModeState read_state_config(
    const YAML::Node & name_node,
    const YAML::Node & node) const;
  std::vector<ModeTransition> read_state_transitions(
    const YAML::Node & transitions_node,
    const std::string & state_path) const;
  ModeTransition read_transition_config(
    const YAML::Node & transition_node,
    const std::string & path) const;
  ModeConditionTrigger read_transition_trigger(
    const YAML::Node & trigger_node,
    const std::string & path) const;
  TransitionDestination read_transition_destination(
    const YAML::Node & node,
    const std::string & path) const;
  ModeConditionTrigger parse_condition_trigger(
    const std::string & trigger,
    const std::string & path) const;
  std::string read_optional_string(const YAML::Node & node) const;
  bool read_optional_bool(const YAML::Node & node) const;

  std::string initial_state_name_;
  std::unordered_map<std::string, ModeState> states_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__STATE_MACHINE_CONFIG_HPP_
