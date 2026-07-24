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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__TELEOP_CONDITIONS_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__TELEOP_CONDITIONS_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

// Teleop conditions map the device-neutral teleop input code onto condition names.
struct TeleopCondition
{
  uint16_t input_code{0};
};

class TeleopConditionsConfig
{
public:
  TeleopConditionsConfig() = default;
  explicit TeleopConditionsConfig(
    std::unordered_map<std::string, TeleopCondition> conditions);

  static TeleopConditionsConfig from_yaml(const YAML::Node & config);

  const TeleopCondition * find_condition(const std::string & condition_name) const;
  bool has_condition(const std::string & condition_name) const;

private:
  std::unordered_map<std::string, TeleopCondition> read_teleop_condition_configs(
    const YAML::Node & conditions_node) const;
  TeleopCondition read_teleop_condition_config(
    const YAML::Node & condition_node,
    const std::string & condition_name) const;
  uint16_t read_condition_input_code(const YAML::Node & node, const std::string & path) const;

  std::unordered_map<std::string, TeleopCondition> conditions_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__TELEOP_CONDITIONS_CONFIG_HPP_
