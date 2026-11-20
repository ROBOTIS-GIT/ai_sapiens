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
// Author: Woojin Wie, Kiwoong Park

#include "ai_sapiens_sim2real/policy/supermimic/supermimic_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace ai_sapiens_sim2real
{
namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error("SuperMimic: " + message);}
}

size_t preview_size(const YAML::Node & goal, const char * key)
{
  const auto number = goal[key].as<int>();
  require(number > 0 && number <= 1024,
    std::string("motion_goal.") + key + "=" + std::to_string(number) + "; expected 1..1024");
  return static_cast<size_t>(number);
}

void validate_observation_contract(const Sim2RealConfig & config, const YAML::Node & node)
{
  require(config.observations().size() == 1 &&
    node["schema"].as<std::string>() == "cyclo.supermimic.external_goal.v1",
    "unsupported observation schema");
  require(node["actor_obs"].as<std::string>() == "proprio",
    "only proprio observations are supported; rep_state requires unavailable global/link states");
  const auto representation = node["representation"].as<std::string>();
  require(representation == "gencoord" || representation == "maxcoord",
    "representation=" + representation + "; expected gencoord or maxcoord");
  const auto encoder = node["encoder"].as<std::string>();
  require(encoder == "motion" || encoder == "velocity",
    "encoder=" + encoder + "; expected motion or velocity");
}

void validate_velocity_ranges(const Sim2RealConfig & config)
{
  require(config.velocity_command_ranges().has_value(), "velocity command ranges are missing");
  const auto & active = *config.velocity_command_ranges();
  const std::array<AxisRange, 3> axes{active.linear_x, active.linear_y, active.angular_z};
  for (const auto & axis : axes) {
    require(std::isfinite(axis.min) && std::isfinite(axis.max) &&
      axis.min <= 0 && axis.max >= 0 && axis.min < axis.max, "invalid velocity range");
  }
}

}  // namespace

SuperMimicConfig::SuperMimicConfig(const Sim2RealConfig & config)
: joints(config.policy_joints()), defaults(config.joint_properties().default_position)
{
  const auto node = config.observations()["supermimic"];
  validate_observation_contract(config, node);
  is_motion = node["encoder"].as<std::string>() == "motion";
  load_policy_settings(config, node);
  read_history(node["history"]);
  read_goals(node);
  calculate_input_size();

  if (!is_motion) {
    validate_velocity_ranges(config);
  }
}

void SuperMimicConfig::load_policy_settings(
  const Sim2RealConfig & config, const YAML::Node & node)
{
  require(!joints.empty() && joints.size() <= 1024, "invalid policy joint count");
  require(std::isfinite(config.step_dt()) && std::abs(config.step_dt() - 0.02) < 1e-9,
    "step_dt=" + std::to_string(config.step_dt()) + "; expected 0.02");
  default_velocities = node["default_joint_velocity"] ?
    node["default_joint_velocity"].as<std::vector<float>>() :
    std::vector<float>(joints.size(), 0.f);
  require(default_velocities.size() == joints.size(), "joint velocity defaults size mismatch");
  require(std::none_of(config.action_properties().clip.begin(),
      config.action_properties().clip.end(),
    [](const auto & value) {return value.has_value();}), "external action clipping is unsupported");
  const auto & scales = config.action_properties().scale;
  const auto & offsets = config.action_properties().offset;
  for (size_t j = 0; j < joints.size(); ++j) {
    require(std::isfinite(defaults[j]) && std::isfinite(default_velocities[j]) &&
      std::isfinite(scales[j]) && scales[j] != 0 && std::isfinite(offsets[j]),
      "nonfinite nominal pose/action values or zero action scale");
  }
}

void SuperMimicConfig::read_history(const YAML::Node & node)
{
  history = ConfiguredObservationGroup(node, {
      {"projected_gravity", 3}, {"base_ang_vel", 3}, {"joint_pos_rel", joints.size()},
      {"joint_vel_rel", joints.size()}, {"last_action", joints.size()}});
  history_length = history.history_length();
}

void SuperMimicConfig::read_goals(const YAML::Node & node)
{
  const auto goal = node["motion_goal"];
  if (is_motion) {
    future_frames = preview_size(goal, "frames");
    future_stride = preview_size(goal, "stride");
    start_offset = preview_size(goal, "start_offset");
    require(goal["groups"].IsSequence() && goal["groups"].size() > 0 && goal["groups"].size() <= 3,
      "invalid motion goal groups");
    for (const auto & group : goal["groups"]) {
      goal_groups.emplace_back(group, std::map<std::string, size_t>{
          {"future_joint_pos", joints.size()}, {"future_joint_vel", joints.size()},
          {"future_root_orientation", 6}});
    }
  } else {
    goal_groups.emplace_back(node["velocity_goal"],
      std::map<std::string, size_t>{{"velocity_command", 3}});
  }
}

void SuperMimicConfig::calculate_input_size()
{
  input_size = history_length * history.width();
  for (const auto & group : goal_groups) {
    require(group.history_length() == 1, "goal terms cannot contain history");
    input_size += group.width() * (is_motion ? future_frames : 1);
  }
}

void SuperMimicConfig::validate_motion(const MimicBehavior & mimic) const
{
  require(is_motion && std::isfinite(mimic.fps) && std::abs(mimic.fps - 50) < 1e-6 &&
    mimic.time_start == 0 && !mimic.time_end, "motion encoder requires a full 50 Hz CSV");
}

}  // namespace ai_sapiens_sim2real
