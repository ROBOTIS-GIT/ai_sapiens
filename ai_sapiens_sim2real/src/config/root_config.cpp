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

#include "ai_sapiens_sim2real/config/root_config.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ai_sapiens_sim2real/config/config_utils.hpp"
#include "ai_sapiens_sim2real/config/root_config_validator.hpp"

namespace ai_sapiens_sim2real
{

namespace
{

std::filesystem::path asset_relative_path(const std::string & path)
{
  auto normalized = path;
  while (!normalized.empty() && normalized.front() == '/') {
    normalized.erase(normalized.begin());
  }
  while (!normalized.empty() && normalized.back() == '/') {
    normalized.pop_back();
  }

  return std::filesystem::path(normalized);
}

void require_root_sections(const YAML::Node & document)
{
  require_yaml_map(document["authority"], "authority");
  require_yaml_map(document["state_behaviors"], "state_behaviors");
  require_yaml_map(document["state_machine"], "state_machine");
  require_yaml_map(document["teleop_input"], "teleop_input");
  require_yaml_map(document["teleop_conditions"], "teleop_conditions");
}

std::vector<std::string> read_controller_joint_names(const YAML::Node & document)
{
  auto joint_names = read_string_sequence(document, "robot_joint_order");
  validate_unique_names(joint_names, "robot_joint_order");

  return joint_names;
}

AuthorityConfig read_authority_config(const YAML::Node & document)
{
  const auto authority = document["authority"];
  const auto api_entry = authority["api_entry"];
  require_yaml_map(api_entry, "authority.api_entry");

  AuthorityConfig config;
  if (api_entry["warmup_duration"]) {
    config.api_entry_warmup_duration = api_entry["warmup_duration"].as<double>();
  }
  if (api_entry["velocity_neutral_threshold"]) {
    config.api_entry_velocity_neutral_threshold =
      api_entry["velocity_neutral_threshold"].as<float>();
  }

  config.api_entry_allowed_from_states = read_required_yaml_string_sequence(
    api_entry["allowed_from_states"],
    "authority.api_entry.allowed_from_states");
  config.default_velocity_state = read_required_yaml_string(
    authority["default_velocity_state"],
    "authority.default_velocity_state");

  return config;
}

bool is_policy_behavior_kind(const std::string & kind)
{
  return kind == "policy" || kind == "mimic";
}

bool is_mimic_behavior(const YAML::Node & behavior_node)
{
  return behavior_node["kind"].as<std::string>() == "mimic";
}

void require_positive_finite_timeout(double timeout, const std::string & path)
{
  if (!std::isfinite(timeout) || timeout <= 0.0) {
    throw std::runtime_error(path + " must be finite and positive");
  }
}

void validate_teleop_timeouts(const OperatorCommandInputOptions & options)
{
  require_positive_finite_timeout(options.teleop_input_timeout, "teleop_input.timeout");
  require_positive_finite_timeout(
    options.teleop_vel_command_timeout, "teleop_input.vel_command_timeout");
  if (options.teleop_vel_command_timeout > options.teleop_input_timeout) {
    throw std::runtime_error(
            "teleop_input.vel_command_timeout must not exceed teleop_input.timeout");
  }
}

}  // namespace

RootConfig::RootConfig(const std::filesystem::path & path)
: path_(std::filesystem::absolute(path))
  , config_dir_(path_.parent_path())
  , document_(load_yaml_file(path_, "root config"))
{
  with_yaml_file_context(
    path_, "root config",
    [&] {
      require_root_sections(document_);

      controller_joints_ = read_controller_joint_names(document_);

      SharedControlData shared_data;
      shared_data.joint_map.controller_joint_names = controller_joints_;

      state_behaviors_config_ = StateBehaviorsConfig::from_yaml(document_, shared_data);
      teleop_conditions_config_ = TeleopConditionsConfig::from_yaml(document_);
      selectors_config_ = SelectorsConfig::from_yaml(document_);
      state_machine_config_ = StateMachineConfig::from_yaml(document_);
      authority_config_ = read_authority_config(document_);

      RootConfigValidator(
        state_machine_config_,
        state_behaviors_config_,
        teleop_conditions_config_,
        selectors_config_,
        authority_config_).validate();
    });
}

std::vector<std::string> RootConfig::controller_joints() const
{
  return controller_joints_;
}

OperatorCommandInputOptions RootConfig::operator_command_input_options() const
{
  OperatorCommandInputOptions options;
  with_yaml_file_context(
    path_, "root config",
    [&] {
      const auto teleop_input = document_["teleop_input"];
      const auto plugin_node = teleop_input["plugin"];
      const auto config_node = teleop_input["config"];

      const auto config_file = read_required_yaml_string(config_node, "teleop_input.config");
      const auto config_path = resolve_config_path(config_dir_, config_file);

      options.teleop_input_plugin = read_required_yaml_string(plugin_node, "teleop_input.plugin");
      options.teleop_input_config_path = config_path.string();
      if (teleop_input["timeout"]) {
        options.teleop_input_timeout = teleop_input["timeout"].as<double>();
      }
      options.teleop_vel_command_timeout = options.teleop_input_timeout;
      if (teleop_input["vel_command_timeout"]) {
        options.teleop_vel_command_timeout = teleop_input["vel_command_timeout"].as<double>();
      }
      validate_teleop_timeouts(options);
    });

  (void)load_yaml_file(options.teleop_input_config_path, "teleop input plugin config");
  return options;
}

const AuthorityConfig & RootConfig::authority_config() const
{
  return authority_config_;
}

const StateMachineConfig & RootConfig::state_machine_config() const
{
  return state_machine_config_;
}

const StateBehaviorsConfig & RootConfig::state_behaviors_config() const
{
  return state_behaviors_config_;
}

const TeleopConditionsConfig & RootConfig::teleop_conditions_config() const
{
  return teleop_conditions_config_;
}

const SelectorsConfig & RootConfig::selectors_config() const
{
  return selectors_config_;
}

std::vector<RootConfig::PolicyBehavior> RootConfig::policy_behaviors() const
{
  return with_yaml_file_context(
    path_, "root config",
    [&] {
      std::vector<PolicyBehavior> behaviors;
      for (const auto & item : document_["state_behaviors"]) {
        const std::string name = item.first.as<std::string>();
        const auto behavior_node = item.second;
        const auto kind_node = behavior_node["kind"];
        const std::string kind = kind_node ? kind_node.as<std::string>() : "";
        if (!is_policy_behavior_kind(kind)) {
          continue;
        }

        behaviors.push_back(read_policy_behavior(name, behavior_node));
      }

      return behaviors;
    });
}

RootConfig::PolicyBehavior RootConfig::read_policy_behavior(
  const std::string & name,
  const YAML::Node & behavior_node) const
{
  PolicyBehavior behavior;
  behavior.name = name;
  behavior.policy_path = behavior_policy_path(behavior_node, name);
  behavior.sim2real_config_path = behavior_sim2real_config_path(behavior_node, name);

  if (is_mimic_behavior(behavior_node)) {
    behavior.mimic = build_mimic_behavior(name, behavior_node);
  }

  return behavior;
}

MimicBehavior RootConfig::build_mimic_behavior(
  const std::string & name, const YAML::Node & behavior_node) const
{
  const auto node = behavior_node_with_mimic_defaults(behavior_node);

  const auto motion_path = behavior_motion_file_path(node, name);
  if (!motion_path) {
    throw std::runtime_error("state_behaviors." + name + " requires motion or motion_file");
  }

  MimicBehavior mimic;
  mimic.motion_file = *motion_path;
  mimic.fps = node["fps"] ? node["fps"].as<float>() : 50.0f;
  mimic.time_start = node["time_start"] ? node["time_start"].as<float>() : 0.0f;
  mimic.time_end = read_mimic_time_end(node);
  mimic.on_complete = read_mimic_on_complete(node);

  if (!mimic.on_complete.empty() && mimic.on_complete != "stay") {
    require_known_mimic_completion_state(mimic.on_complete, name);
  }

  return mimic;
}

std::optional<float> RootConfig::read_mimic_time_end(const YAML::Node & node) const
{
  // A numeric time_end caps the window; absent or "auto" plays to the end.
  const auto time_end = node["time_end"];
  if (!time_end ||
    (time_end.IsScalar() && time_end.as<std::string>() == "auto"))
  {
    return std::nullopt;
  }

  return time_end.as<float>();
}

std::string RootConfig::read_mimic_on_complete(const YAML::Node & node) const
{
  return node["on_complete"] ? node["on_complete"].as<std::string>() : std::string{};
}

void RootConfig::require_known_mimic_completion_state(
  const std::string & state_name,
  const std::string & behavior_name) const
{
  const auto states = document_["state_machine"]["states"];
  if (!states || !states[state_name]) {
    throw std::runtime_error(
      "state_behaviors." + behavior_name +
      ".on_complete references unknown state '" + state_name + "'");
  }
}

YAML::Node RootConfig::behavior_node_with_mimic_defaults(
  const YAML::Node & behavior_node) const
{
  YAML::Node node = YAML::Clone(behavior_node);
  const auto defaults = document_["mimic_defaults"];
  if (yaml_node_is_missing(defaults)) {
    return node;
  }
  require_yaml_map(defaults, "mimic_defaults");

  if (!node["fps"] && defaults["fps"]) {
    node["fps"] = defaults["fps"].as<float>();
  }
  if (!node["time_start"] && defaults["time_start"]) {
    node["time_start"] = defaults["time_start"].as<float>();
  }
  if (!node["time_end"] && defaults["time_end"]) {
    const auto value = defaults["time_end"];
    if (!value.IsScalar() || value.as<std::string>() != "auto") {
      node["time_end"] = value.as<float>();
    }
  }
  if (!node["on_complete"] && defaults["on_complete"]) {
    node["on_complete"] = defaults["on_complete"].as<std::string>();
  }

  return node;
}

std::vector<std::filesystem::path> RootConfig::policy_asset_roots() const
{
  const auto roots = document_["policy_asset_roots"];
  require_yaml_node(roots, "policy_asset_roots");

  std::vector<std::filesystem::path> resolved_roots;
  if (roots.IsScalar()) {
    resolved_roots.push_back(resolve_config_path(config_dir_, roots.as<std::string>()));
  } else if (roots.IsSequence()) {
    for (const auto & root : roots) {
      resolved_roots.push_back(resolve_config_path(config_dir_, root.as<std::string>()));
    }
  } else {
    throw std::runtime_error("policy_asset_roots must be a path or sequence of paths");
  }

  if (resolved_roots.empty()) {
    throw std::runtime_error("policy_asset_roots must not be empty");
  }

  return resolved_roots;
}

std::filesystem::path RootConfig::behavior_asset_path(
  const YAML::Node & behavior_node, const std::string & name) const
{
  if (!behavior_node["asset"]) {
    throw std::runtime_error("state_behaviors." + name + " requires asset");
  }

  const auto asset = asset_relative_path(behavior_node["asset"].as<std::string>());
  const auto roots = policy_asset_roots();
  for (const auto & root : roots) {
    const auto candidate = root / asset;
    if (std::filesystem::is_directory(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error(
    "state_behaviors." + name + ".asset '" + asset.string() +
    "' was not found under policy_asset_roots: " + make_path_list_string(roots));
}

std::filesystem::path RootConfig::behavior_policy_path(
  const YAML::Node & behavior_node, const std::string & name) const
{
  std::filesystem::path path;
  if (behavior_node["policy_path"]) {
    path = resolve_config_path(config_dir_, behavior_node["policy_path"].as<std::string>());
  } else {
    path = behavior_asset_path(behavior_node, name) / "exported" / "policy.onnx";
  }

  require_existing_file(path, "state_behaviors." + name + " policy");
  return path;
}

std::filesystem::path RootConfig::behavior_sim2real_config_path(
  const YAML::Node & behavior_node, const std::string & name) const
{
  std::filesystem::path path;
  if (behavior_node["sim2real_yaml_path"]) {
    const auto sim2real_config_file = behavior_node["sim2real_yaml_path"].as<std::string>();
    path = resolve_config_path(config_dir_, sim2real_config_file);
  } else {
    path = behavior_asset_path(behavior_node, name) / "params" / "sim2real.yaml";
  }

  require_existing_file(path, "state_behaviors." + name + " sim2real_yaml");
  return path;
}

std::optional<std::filesystem::path> RootConfig::behavior_motion_file_path(
  const YAML::Node & behavior_node, const std::string & name) const
{
  std::filesystem::path path;
  if (behavior_node["motion_file"]) {
    path = resolve_config_path(config_dir_, behavior_node["motion_file"].as<std::string>());
  } else if (behavior_node["motion"]) {
    const auto motion_file = behavior_node["motion"].as<std::string>();
    if (std::filesystem::path(motion_file).extension() != ".csv") {
      throw std::runtime_error(
        "state_behaviors." + name + ".motion must include the .csv extension");
    }

    path = behavior_asset_path(behavior_node, name) / "params" / asset_relative_path(motion_file);
  } else {
    return std::nullopt;
  }

  require_existing_file(path, "state_behaviors." + name + " motion");
  return path;
}

}  // namespace ai_sapiens_sim2real
