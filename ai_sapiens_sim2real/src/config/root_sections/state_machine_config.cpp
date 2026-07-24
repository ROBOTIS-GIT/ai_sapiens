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

#include "ai_sapiens_sim2real/config/root_sections/state_machine_config.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

#include "ai_sapiens_sim2real/config/config_utils.hpp"

namespace ai_sapiens_sim2real
{

StateMachineConfig::StateMachineConfig(
  std::string initial_state_name,
  std::unordered_map<std::string, ModeState> states)
: initial_state_name_(std::move(initial_state_name)),
  states_(std::move(states))
{
}

StateMachineConfig StateMachineConfig::from_yaml(const YAML::Node & config)
{
  StateMachineConfig state_machine;
  const auto machine = state_machine.read_state_machine_node(config);

  state_machine.initial_state_name_ =
    read_required_yaml_string(machine["initial"], "state_machine.initial");
  state_machine.states_ = state_machine.read_state_configs(machine["states"]);

  return state_machine;
}

const std::string & StateMachineConfig::initial_state_name() const
{
  return initial_state_name_;
}

const ModeState * StateMachineConfig::find_state(const std::string & state_name) const
{
  const auto state = states_.find(state_name);
  if (state == states_.end()) {
    return nullptr;
  }

  return &state->second;
}

const ModeState & StateMachineConfig::require_state(const std::string & state_name) const
{
  const auto state = find_state(state_name);
  if (state == nullptr) {
    throw std::runtime_error("Unknown state: " + state_name);
  }

  return *state;
}

bool StateMachineConfig::has_state(const std::string & state_name) const
{
  return find_state(state_name) != nullptr;
}

std::vector<std::string> StateMachineConfig::concrete_state_names() const
{
  std::vector<std::string> names;
  names.reserve(states_.size());
  for (const auto & [name, state] : states_) {
    if (!state.abstract) {
      names.push_back(name);
    }
  }

  std::sort(names.begin(), names.end());
  return names;
}

void StateMachineConfig::for_each_state(
  const std::function<void(const std::string &, const ModeState &)> & visitor) const
{
  for (const auto & [name, state] : states_) {
    visitor(name, state);
  }
}

YAML::Node StateMachineConfig::read_state_machine_node(const YAML::Node & config) const
{
  const auto machine = config["state_machine"];
  require_yaml_map(machine, "state_machine");

  return machine;
}

std::unordered_map<std::string, ModeState> StateMachineConfig::read_state_configs(
  const YAML::Node & states_node) const
{
  require_yaml_map(states_node, "state_machine.states");

  std::unordered_map<std::string, ModeState> states;
  states.reserve(states_node.size());

  for (const auto & state_entry : states_node) {
    const auto state = read_state_config(state_entry.first, state_entry.second);
    states[state.name] = state;
  }

  return states;
}

ModeState StateMachineConfig::read_state_config(
  const YAML::Node & name_node,
  const YAML::Node & node) const
{
  ModeState state;
  state.name = name_node.as<std::string>();
  state.parent = read_optional_string(node["parent"]);
  state.run = read_optional_string(node["run"]);
  state.abstract = read_optional_bool(node["abstract"]);

  const std::string state_path = "state_machine.states." + state.name;
  state.transitions = read_state_transitions(node["transitions"], state_path);

  return state;
}

std::vector<ModeTransition> StateMachineConfig::read_state_transitions(
  const YAML::Node & transitions_node,
  const std::string & state_path) const
{
  if (yaml_node_is_missing(transitions_node)) {
    return {};
  }

  require_yaml_sequence(transitions_node, state_path + ".transitions");

  std::vector<ModeTransition> transitions;
  transitions.reserve(transitions_node.size());

  for (std::size_t i = 0; i < transitions_node.size(); ++i) {
    transitions.push_back(
      read_transition_config(
        transitions_node[i],
        state_path + ".transitions[" + std::to_string(i) + "]"));
  }

  return transitions;
}

ModeTransition StateMachineConfig::read_transition_config(
  const YAML::Node & transition_node,
  const std::string & path) const
{
  require_yaml_map(transition_node, path);

  ModeTransition transition;
  transition.condition_name =
    read_required_yaml_string(transition_node["when"], path + ".when");
  transition.trigger = read_transition_trigger(transition_node["trigger"], path);

  const auto destination = read_transition_destination(transition_node, path);
  transition.target_state = destination.target_state;
  transition.selector_name = destination.selector_name;

  return transition;
}

ModeConditionTrigger StateMachineConfig::read_transition_trigger(
  const YAML::Node & trigger_node,
  const std::string & path) const
{
  if (yaml_node_is_missing(trigger_node)) {
    return ModeConditionTrigger::Level;
  }

  if (!trigger_node.IsScalar()) {
    throw std::runtime_error(path + ".trigger must be a scalar");
  }

  return parse_condition_trigger(trigger_node.as<std::string>(), path + ".trigger");
}

StateMachineConfig::TransitionDestination StateMachineConfig::read_transition_destination(
  const YAML::Node & node,
  const std::string & path) const
{
  if (node["to"]) {
    return TransitionDestination{node["to"].as<std::string>(), {}};
  }

  if (node["select"]) {
    return TransitionDestination{{}, node["select"].as<std::string>()};
  }

  throw std::runtime_error(path + " must define to or select");
}

ModeConditionTrigger StateMachineConfig::parse_condition_trigger(
  const std::string & trigger,
  const std::string & path) const
{
  if (trigger == "level") {
    return ModeConditionTrigger::Level;
  }
  if (trigger == "edge") {
    return ModeConditionTrigger::Edge;
  }

  throw std::runtime_error(path + " must be level or edge");
}

std::string StateMachineConfig::read_optional_string(const YAML::Node & node) const
{
  return yaml_node_is_missing(node) ? std::string{} : node.as<std::string>();
}

bool StateMachineConfig::read_optional_bool(const YAML::Node & node) const
{
  return yaml_node_is_missing(node) ? false : node.as<bool>();
}

}  // namespace ai_sapiens_sim2real
