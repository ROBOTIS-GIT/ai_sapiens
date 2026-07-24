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

#include "ai_sapiens_sim2real/config/sim2real_config.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "ai_sapiens_sim2real/config/config_utils.hpp"

namespace ai_sapiens_sim2real
{

namespace
{

struct JointProperty
{
  float default_position;
  float stiffness;
  float damping;
  PositionLimit position_limit;
};

std::vector<std::string> read_policy_joint_order(const YAML::Node & config)
{
  auto joints = read_string_sequence(config, "policy_joints");
  validate_unique_names(joints, "policy_joints");
  return joints;
}

float read_required_float(
  const YAML::Node & node, const std::string & key,
  const std::string & joint_name)
{
  const std::string value_path = "joint_properties." + joint_name + "." + key;
  const auto value_node = node[key];
  require_yaml_node(value_node, value_path);

  return value_node.as<float>();
}

std::vector<float> read_action_vector(
  const YAML::Node & action_node,
  const std::string & key,
  const std::vector<std::string> & policy_joints,
  const std::vector<float> & default_values)
{
  const std::string value_path = "actions.joint_pos." + key;
  const size_t action_size = policy_joints.size();

  const auto value_node = action_node[key];
  if (yaml_node_is_missing(value_node)) {
    return default_values;
  }

  if (value_node.IsScalar()) {
    const auto scalar = value_node.as<std::string>();
    if (scalar == "default_position") {
      return default_values;
    }

    const float value = value_node.as<float>();
    return std::vector<float>(action_size, value);
  }

  if (!value_node.IsSequence()) {
    throw std::runtime_error(value_path + " must be a scalar, sequence, or default_position");
  }

  const auto values = value_node.as<std::vector<float>>();
  if (values.size() != action_size) {
    throw std::runtime_error(
      value_path + " has " + std::to_string(values.size()) +
      " entries, expected " + std::to_string(action_size));
  }

  return values;
}

PositionLimit read_position_limit_pair(
  const YAML::Node & limit_node,
  const std::string & limit_path)
{
  if (!limit_node.IsSequence()) {
    throw std::runtime_error(limit_path + " must be [min, max]");
  }
  if (limit_node.size() != 2U) {
    throw std::runtime_error(limit_path + " must contain exactly [min, max]");
  }

  const float min_value = limit_node[0].as<float>();
  const float max_value = limit_node[1].as<float>();
  if (min_value > max_value) {
    throw std::runtime_error(limit_path + " min is greater than max");
  }

  PositionLimit limit;
  limit.emplace(min_value, max_value);
  return limit;
}

std::vector<PositionLimit> read_action_clip(
  const YAML::Node & action_node,
  const std::vector<std::string> & policy_joints)
{
  const std::string clip_path = "actions.joint_pos.clip";
  const size_t action_size = policy_joints.size();

  std::vector<PositionLimit> clip(action_size, std::nullopt);
  const auto clip_node = action_node["clip"];
  if (yaml_node_is_missing(clip_node)) {
    return clip;
  }

  if (!clip_node.IsSequence()) {
    throw std::runtime_error(clip_path + " must be null or a sequence");
  }
  if (clip_node.size() != action_size) {
    throw std::runtime_error(
      clip_path + " has " + std::to_string(clip_node.size()) +
      " entries, expected " + std::to_string(action_size));
  }

  for (size_t i = 0; i < policy_joints.size(); ++i) {
    const auto joint_clip_node = clip_node[i];
    if (yaml_node_is_missing(joint_clip_node)) {
      continue;
    }

    const std::string joint_clip_path = clip_path + " for joint '" + policy_joints[i] + "'";
    clip[i] = read_position_limit_pair(joint_clip_node, joint_clip_path);
  }

  return clip;
}

PositionLimit read_optional_position_limit(
  const YAML::Node & node,
  const std::string & joint_name)
{
  const auto limit_node = node["position_limit"];
  if (yaml_node_is_missing(limit_node)) {
    return std::nullopt;
  }

  const std::string limit_path = "joint_properties." + joint_name + ".position_limit";
  return read_position_limit_pair(limit_node, limit_path);
}

JointProperty read_joint_property(
  const YAML::Node & joint_properties,
  const std::string & joint_name)
{
  const auto joint_node = joint_properties[joint_name];
  if (yaml_node_is_missing(joint_node)) {
    throw std::runtime_error("joint_properties missing policy joint '" + joint_name + "'");
  }

  require_yaml_map(joint_node, "joint_properties." + joint_name);

  return JointProperty{
    read_required_float(joint_node, "default_position", joint_name),
    read_required_float(joint_node, "stiffness", joint_name),
    read_required_float(joint_node, "damping", joint_name),
    read_optional_position_limit(joint_node, joint_name)};
}

JointProperties read_joint_properties(
  const YAML::Node & config,
  const std::vector<std::string> & policy_joints)
{
  const auto joint_properties = config["joint_properties"];
  require_yaml_map(joint_properties, "joint_properties");

  JointProperties properties;
  properties.default_position.reserve(policy_joints.size());
  properties.stiffness.reserve(policy_joints.size());
  properties.damping.reserve(policy_joints.size());
  properties.position_limits.reserve(policy_joints.size());

  for (const auto & joint_name : policy_joints) {
    const auto joint_property = read_joint_property(joint_properties, joint_name);
    properties.default_position.push_back(joint_property.default_position);
    properties.stiffness.push_back(joint_property.stiffness);
    properties.damping.push_back(joint_property.damping);
    properties.position_limits.push_back(joint_property.position_limit);
  }

  return properties;
}

ActionProperties read_action_properties(
  const YAML::Node & config,
  const std::vector<std::string> & policy_joints,
  const std::vector<float> & default_position)
{
  const auto actions_node = config["actions"];
  require_yaml_map(actions_node, "actions");

  const auto action_node = actions_node["joint_pos"];
  require_yaml_map(action_node, "actions.joint_pos");

  ActionProperties properties;
  properties.scale = read_action_vector(
    action_node,
    "scale",
    policy_joints,
    std::vector<float>(policy_joints.size(), 1.0f));
  properties.offset = read_action_vector(action_node, "offset", policy_joints, default_position);
  properties.clip = read_action_clip(action_node, policy_joints);
  return properties;
}

AxisRange read_velocity_command_range(
  const std::filesystem::path & sim2real_config_path,
  const YAML::Node & sim2real_config,
  const std::string & key)
{
  return with_yaml_file_context(
    sim2real_config_path, "velocity policy sim2real_yaml",
    [&] {
      const std::string range_path = "commands.base_velocity.ranges." + key;
      const auto range_node = sim2real_config["commands"]["base_velocity"]["ranges"][key];
      require_yaml_sequence(range_node, range_path);

      const auto values = range_node.as<std::vector<double>>();
      if (values.size() != 2U) {
        throw std::runtime_error(range_path + " must contain exactly [min, max]");
      }

      const bool range_is_ordered = values[0] <= values[1];
      if (!range_is_ordered) {
        throw std::runtime_error(range_path + " min must be <= max");
      }

      const bool range_includes_zero = values[0] <= 0.0 && values[1] >= 0.0;
      if (!range_includes_zero) {
        throw std::runtime_error(range_path + " must include zero");
      }

      return AxisRange{values[0], values[1]};
    });
}

std::optional<AxisRanges> read_velocity_command_ranges(
  const std::filesystem::path & sim2real_config_path,
  const YAML::Node & sim2real_config)
{
  const auto commands = sim2real_config["commands"];
  const auto base_velocity = commands["base_velocity"];
  const bool has_velocity_ranges = base_velocity && base_velocity["ranges"];
  if (!has_velocity_ranges) {
    return std::nullopt;
  }

  return AxisRanges{
    read_velocity_command_range(sim2real_config_path, sim2real_config, "lin_vel_x"),
    read_velocity_command_range(sim2real_config_path, sim2real_config, "lin_vel_y"),
    read_velocity_command_range(sim2real_config_path, sim2real_config, "ang_vel_z")};
}

}  // namespace

std::vector<size_t> make_source_indices_for_target(
  const std::vector<std::string> & source,
  const std::vector<std::string> & target,
  const std::string & source_label,
  const std::string & target_label)
{
  std::unordered_map<std::string, size_t> source_indices;
  for (size_t i = 0; i < source.size(); ++i) {
    source_indices[source[i]] = i;
  }

  std::vector<size_t> indices;
  indices.reserve(target.size());
  for (const auto & target_name : target) {
    auto it = source_indices.find(target_name);
    if (it == source_indices.end()) {
      throw std::runtime_error(
        target_label + " joint '" + target_name + "' does not exist in " + source_label);
    }

    indices.push_back(it->second);
  }

  return indices;
}

Sim2RealConfig::Sim2RealConfig(const std::filesystem::path & path)
: path_(path)
{
  const YAML::Node node = load_yaml_file(path_, "policy sim2real.yaml");
  with_yaml_file_context(
    path_, "policy sim2real.yaml",
    [&] {
      policy_joints_ = read_policy_joint_order(node);
      const auto step_dt_node = node["step_dt"];
      require_yaml_node(step_dt_node, "step_dt");

      step_dt_ = step_dt_node.as<double>();
      joint_properties_ = read_joint_properties(node, policy_joints_);
      action_properties_ = read_action_properties(
        node,
        policy_joints_,
        joint_properties_.default_position);
      observations_ = node["observations"];
    });
  velocity_command_ranges_ = read_velocity_command_ranges(path_, node);
}

}  // namespace ai_sapiens_sim2real
