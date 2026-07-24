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

#include "ai_sapiens_sim2real/config/root_sections/state_behaviors_config.hpp"

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

#include "ai_sapiens_sim2real/config/config_utils.hpp"

namespace ai_sapiens_sim2real
{

StateBehaviorsConfig::StateBehaviorsConfig(
  std::unordered_map<std::string, StateBehavior> behaviors)
: behaviors_(std::move(behaviors))
{
}

StateBehaviorsConfig StateBehaviorsConfig::from_yaml(
  const YAML::Node & config,
  const SharedControlData & shared_data)
{
  StateBehaviorsConfig state_behaviors;
  state_behaviors.behaviors_ =
    state_behaviors.read_behavior_configs(config["state_behaviors"], shared_data);

  return state_behaviors;
}

const StateBehavior * StateBehaviorsConfig::find_behavior(
  const std::string & behavior_name) const
{
  const auto behavior = behaviors_.find(behavior_name);
  if (behavior == behaviors_.end()) {
    return nullptr;
  }

  return &behavior->second;
}

const StateBehavior & StateBehaviorsConfig::require_behavior(
  const std::string & behavior_name) const
{
  const auto behavior = find_behavior(behavior_name);
  if (behavior == nullptr) {
    throw std::runtime_error("Unknown state_behavior: " + behavior_name);
  }

  return *behavior;
}

bool StateBehaviorsConfig::has_behavior(const std::string & behavior_name) const
{
  return find_behavior(behavior_name) != nullptr;
}

namespace
{
struct BehaviorKindSpec
{
  const char * name;
  BehaviorKind kind;
  bool mimic;
};

constexpr std::array<BehaviorKindSpec, 4> kBehaviorKindSpecs{{
  {"damping", BehaviorKind::Damping, false},
  {"posture", BehaviorKind::Posture, false},
  {"policy", BehaviorKind::Policy, false},
  {"mimic", BehaviorKind::Policy, true},
}};
}  // namespace

std::unordered_map<std::string, StateBehavior> StateBehaviorsConfig::read_behavior_configs(
  const YAML::Node & behaviors_node,
  const SharedControlData & shared_data) const
{
  require_yaml_map(behaviors_node, "state_behaviors");

  std::unordered_map<std::string, StateBehavior> behaviors;
  behaviors.reserve(behaviors_node.size());

  for (const auto & item : behaviors_node) {
    const auto name = item.first.as<std::string>();
    behaviors[name] = read_behavior_config(name, item.second, shared_data);
  }

  return behaviors;
}

StateBehavior StateBehaviorsConfig::read_behavior_config(
  const std::string & name,
  const YAML::Node & node,
  const SharedControlData & shared_data) const
{
  const std::string behavior_path = "state_behaviors." + name;
  auto behavior = read_behavior_kind(node, behavior_path);

  return read_behavior_payload(
    std::move(behavior),
    node,
    behavior_path,
    name,
    shared_data);
}

StateBehavior StateBehaviorsConfig::read_behavior_kind(
  const YAML::Node & node,
  const std::string & behavior_path) const
{
  const auto kind_name = read_required_yaml_string(node["kind"], behavior_path + ".kind");
  return parse_behavior_kind(kind_name);
}

StateBehavior StateBehaviorsConfig::read_behavior_payload(
  StateBehavior behavior,
  const YAML::Node & node,
  const std::string & behavior_path,
  const std::string & name,
  const SharedControlData & shared_data) const
{
  if (behavior.kind == BehaviorKind::Damping) {
    behavior.damping = read_joint_map(node["damping"], behavior_path + ".damping", shared_data);
    return behavior;
  }

  if (behavior.kind == BehaviorKind::Posture) {
    return read_posture_behavior(
      std::move(behavior),
      node,
      behavior_path,
      shared_data);
  }

  behavior.policy_name = name;
  require_policy_location(node, behavior_path);
  return behavior;
}

StateBehavior StateBehaviorsConfig::read_posture_behavior(
  StateBehavior behavior,
  const YAML::Node & node,
  const std::string & behavior_path,
  const SharedControlData & shared_data) const
{
  behavior.duration = read_posture_duration(node, behavior_path, behavior.duration);
  behavior.stiffness = read_joint_map(
    node["stiffness"],
    behavior_path + ".stiffness",
    shared_data);
  behavior.damping = read_joint_map(
    node["damping"],
    behavior_path + ".damping",
    shared_data);
  behavior.target_position = read_joint_map(
    node["target_position"],
    behavior_path + ".target_position",
    shared_data);

  return behavior;
}

double StateBehaviorsConfig::read_posture_duration(
  const YAML::Node & node,
  const std::string & behavior_path,
  double default_duration) const
{
  const auto duration_node = node["duration"];
  const double duration =
    yaml_node_is_missing(duration_node) ? default_duration : duration_node.as<double>();

  if (duration <= 0.0) {
    throw std::runtime_error(behavior_path + ".duration must be positive");
  }

  return duration;
}

void StateBehaviorsConfig::require_policy_location(
  const YAML::Node & node,
  const std::string & behavior_path) const
{
  if (!has_policy_location(node)) {
    throw std::runtime_error(
      behavior_path + " requires asset, or both policy_path and sim2real_yaml_path");
  }
}

bool StateBehaviorsConfig::has_policy_location(const YAML::Node & node) const
{
  return !yaml_node_is_missing(node["asset"]) ||
         (!yaml_node_is_missing(node["policy_path"]) &&
         !yaml_node_is_missing(node["sim2real_yaml_path"]));
}

std::vector<float> StateBehaviorsConfig::read_joint_map(
  const YAML::Node & node,
  const std::string & path,
  const SharedControlData & shared_data) const
{
  require_yaml_map(node, path);

  const auto & controller_joint_names = shared_data.joint_map.controller_joint_names;
  const auto controller_joint_count = controller_joint_names.size();

  if (node["values"]) {
    return read_joint_values_sequence(node["values"], path, controller_joint_count);
  }

  return read_joint_values_by_name(node, path, shared_data);
}

std::vector<float> StateBehaviorsConfig::read_joint_values_sequence(
  const YAML::Node & values_node,
  const std::string & path,
  std::size_t controller_joint_count) const
{
  if (!values_node.IsSequence()) {
    throw std::runtime_error(path + ".values must be a sequence");
  }

  std::vector<float> values;
  for (const auto & item : values_node) {
    if (item.IsSequence()) {
      const auto row = item.as<std::vector<float>>();
      values.insert(values.end(), row.begin(), row.end());
    } else {
      values.push_back(item.as<float>());
    }
  }

  if (values.size() != controller_joint_count) {
    throw std::runtime_error(
      path + ".values has " + std::to_string(values.size()) +
      " entries, expected " + std::to_string(controller_joint_count));
  }

  return values;
}

std::vector<float> StateBehaviorsConfig::read_joint_values_by_name(
  const YAML::Node & node,
  const std::string & path,
  const SharedControlData & shared_data) const
{
  const auto & controller_joint_names = shared_data.joint_map.controller_joint_names;
  std::vector<float> values(controller_joint_names.size(), 0.0f);

  for (std::size_t i = 0; i < controller_joint_names.size(); ++i) {
    const auto & joint_name = controller_joint_names[i];
    const auto value = node[joint_name];
    if (yaml_node_is_missing(value)) {
      throw std::runtime_error(path + " is missing joint '" + joint_name + "'");
    }

    values[i] = value.as<float>();
  }

  return values;
}

StateBehavior StateBehaviorsConfig::parse_behavior_kind(const std::string & kind) const
{
  for (const auto & spec : kBehaviorKindSpecs) {
    if (kind == spec.name) {
      StateBehavior behavior;
      behavior.kind = spec.kind;
      behavior.mimic = spec.mimic;
      return behavior;
    }
  }

  throw std::runtime_error(
    "Unsupported behavior kind: " + kind +
    "; expected one of " + supported_behavior_kind_names());
}

std::string StateBehaviorsConfig::supported_behavior_kind_names() const
{
  std::string names;
  for (const auto & spec : kBehaviorKindSpecs) {
    if (!names.empty()) {
      names += ", ";
    }

    names += spec.name;
  }

  return names;
}

}  // namespace ai_sapiens_sim2real
