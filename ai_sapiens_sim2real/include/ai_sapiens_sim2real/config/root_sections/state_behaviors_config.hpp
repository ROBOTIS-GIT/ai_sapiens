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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__STATE_BEHAVIORS_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__STATE_BEHAVIORS_CONFIG_HPP_

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

// Behaviors describe what to execute after entering a state.
struct StateBehavior
{
  BehaviorKind kind{BehaviorKind::Policy};
  bool mimic{false};
  std::string policy_name;
  double duration{3.0};
  std::vector<float> damping;
  std::vector<float> stiffness;
  std::vector<float> target_position;
};

class StateBehaviorsConfig
{
public:
  StateBehaviorsConfig() = default;
  explicit StateBehaviorsConfig(std::unordered_map<std::string, StateBehavior> behaviors);

  static StateBehaviorsConfig from_yaml(
    const YAML::Node & config,
    const SharedControlData & shared_data);

  const StateBehavior & require_behavior(const std::string & behavior_name) const;
  bool has_behavior(const std::string & behavior_name) const;

private:
  std::unordered_map<std::string, StateBehavior> read_behavior_configs(
    const YAML::Node & behaviors_node,
    const SharedControlData & shared_data) const;
  StateBehavior read_behavior_config(
    const std::string & name,
    const YAML::Node & node,
    const SharedControlData & shared_data) const;
  StateBehavior read_behavior_kind(
    const YAML::Node & node,
    const std::string & behavior_path) const;
  StateBehavior read_behavior_payload(
    StateBehavior behavior,
    const YAML::Node & node,
    const std::string & behavior_path,
    const std::string & name,
    const SharedControlData & shared_data) const;
  StateBehavior read_posture_behavior(
    StateBehavior behavior,
    const YAML::Node & node,
    const std::string & behavior_path,
    const SharedControlData & shared_data) const;
  double read_posture_duration(
    const YAML::Node & node,
    const std::string & behavior_path,
    double default_duration) const;
  void require_policy_location(
    const YAML::Node & node,
    const std::string & behavior_path) const;
  bool has_policy_location(const YAML::Node & node) const;
  std::vector<float> read_joint_map(
    const YAML::Node & node,
    const std::string & path,
    const SharedControlData & shared_data) const;
  std::vector<float> read_joint_values_sequence(
    const YAML::Node & values_node,
    const std::string & path,
    std::size_t controller_joint_count) const;
  std::vector<float> read_joint_values_by_name(
    const YAML::Node & node,
    const std::string & path,
    const SharedControlData & shared_data) const;
  StateBehavior parse_behavior_kind(const std::string & kind) const;
  std::string supported_behavior_kind_names() const;

  const StateBehavior * find_behavior(const std::string & behavior_name) const;

  std::unordered_map<std::string, StateBehavior> behaviors_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__STATE_BEHAVIORS_CONFIG_HPP_
