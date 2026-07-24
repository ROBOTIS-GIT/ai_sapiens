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

#include "ai_sapiens_sim2real/config/root_sections/teleop_conditions_config.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "ai_sapiens_sim2real/config/config_utils.hpp"

namespace ai_sapiens_sim2real
{

TeleopConditionsConfig::TeleopConditionsConfig(
  std::unordered_map<std::string, TeleopCondition> conditions)
: conditions_(std::move(conditions))
{
}

TeleopConditionsConfig TeleopConditionsConfig::from_yaml(const YAML::Node & config)
{
  TeleopConditionsConfig teleop_conditions;
  teleop_conditions.conditions_ =
    teleop_conditions.read_teleop_condition_configs(config["teleop_conditions"]);

  return teleop_conditions;
}

const TeleopCondition * TeleopConditionsConfig::find_condition(
  const std::string & condition_name) const
{
  const auto condition = conditions_.find(condition_name);
  if (condition == conditions_.end()) {
    return nullptr;
  }

  return &condition->second;
}

bool TeleopConditionsConfig::has_condition(const std::string & condition_name) const
{
  return find_condition(condition_name) != nullptr;
}

std::unordered_map<std::string, TeleopCondition>
TeleopConditionsConfig::read_teleop_condition_configs(
  const YAML::Node & conditions_node) const
{
  require_yaml_map(conditions_node, "teleop_conditions");

  std::unordered_map<std::string, TeleopCondition> conditions;
  conditions.reserve(conditions_node.size());

  for (const auto & item : conditions_node) {
    const auto condition_name = item.first.as<std::string>();
    conditions[condition_name] = read_teleop_condition_config(
      item.second,
      condition_name);
  }

  return conditions;
}

TeleopCondition TeleopConditionsConfig::read_teleop_condition_config(
  const YAML::Node & condition_node,
  const std::string & condition_name) const
{
  const std::string condition_path = "teleop_conditions." + condition_name;

  TeleopCondition condition;
  condition.input_code = read_condition_input_code(
    condition_node["input_code"],
    condition_path + ".input_code");

  return condition;
}

uint16_t TeleopConditionsConfig::read_condition_input_code(
  const YAML::Node & node,
  const std::string & path) const
{
  require_yaml_node(node, path);
  return node.as<uint16_t>();
}

}  // namespace ai_sapiens_sim2real
