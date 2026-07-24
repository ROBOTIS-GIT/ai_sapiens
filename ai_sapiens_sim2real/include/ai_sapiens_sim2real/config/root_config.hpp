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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__ROOT_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__ROOT_CONFIG_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/config/authority_config.hpp"
#include "ai_sapiens_sim2real/config/mimic_behavior.hpp"
#include "ai_sapiens_sim2real/config/root_sections/selectors_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_behaviors_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/state_machine_config.hpp"
#include "ai_sapiens_sim2real/config/root_sections/teleop_conditions_config.hpp"
#include "ai_sapiens_sim2real/mode_runtime/operator_command_input_options.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief The robot's root config (e.g. k1_config.yaml): the entry point that
 *        ties together the state machine, teleop input, authority, and the
 *        per-behavior policy assets.
 *
 * Construction loads the file and validates the top-level schema, so a
 * constructed RootConfig is always well-formed. Callers read it through the
 * typed accessors below; the parsed YAML never leaves this class.
 */
class RootConfig
{
public:
  // A policy/mimic state resolved against the root config: assets located and,
  // for a mimic policy, the reference-motion playback resolved. A present
  // `mimic` marks the behavior as a mimic policy.
  struct PolicyBehavior
  {
    std::string name;
    std::filesystem::path policy_path;
    std::filesystem::path sim2real_config_path;
    std::optional<MimicBehavior> mimic;
  };

  explicit RootConfig(const std::filesystem::path & path);

  // robot_joint_order: the joint set every controller and policy maps onto.
  std::vector<std::string> controller_joints() const;

  // Operator command inputs: teleop plugin plus API heartbeat/cmd_vel topics.
  OperatorCommandInputOptions operator_command_input_options() const;

  // authority: API-entry policy and authority-owned state names.
  const AuthorityConfig & authority_config() const;

  // Mode runtime sections parsed from the root config.
  const StateMachineConfig & state_machine_config() const;
  const StateBehaviorsConfig & state_behaviors_config() const;
  const TeleopConditionsConfig & teleop_conditions_config() const;
  const SelectorsConfig & selectors_config() const;

  // Every policy/mimic behavior, resolved and ready to instantiate a runtime.
  std::vector<PolicyBehavior> policy_behaviors() const;

private:
  MimicBehavior build_mimic_behavior(
    const std::string & name, const YAML::Node & behavior_node) const;
  PolicyBehavior read_policy_behavior(
    const std::string & name, const YAML::Node & behavior_node) const;
  YAML::Node behavior_node_with_mimic_defaults(const YAML::Node & behavior_node) const;
  std::optional<float> read_mimic_time_end(const YAML::Node & node) const;
  std::string read_mimic_on_complete(const YAML::Node & node) const;
  void require_known_mimic_completion_state(
    const std::string & state_name, const std::string & behavior_name) const;

  std::vector<std::filesystem::path> policy_asset_roots() const;
  std::filesystem::path behavior_asset_path(
    const YAML::Node & behavior_node, const std::string & name) const;
  std::filesystem::path behavior_policy_path(
    const YAML::Node & behavior_node, const std::string & name) const;
  std::filesystem::path behavior_sim2real_config_path(
    const YAML::Node & behavior_node, const std::string & name) const;
  std::optional<std::filesystem::path> behavior_motion_file_path(
    const YAML::Node & behavior_node, const std::string & name) const;

  std::filesystem::path path_;
  std::filesystem::path config_dir_;
  YAML::Node document_;
  std::vector<std::string> controller_joints_;
  AuthorityConfig authority_config_;
  StateMachineConfig state_machine_config_;
  StateBehaviorsConfig state_behaviors_config_;
  TeleopConditionsConfig teleop_conditions_config_;
  SelectorsConfig selectors_config_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__ROOT_CONFIG_HPP_
