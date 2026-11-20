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

#include "ai_sapiens_sim2real/policy/blinkmimic/blinkmimic_config.hpp"

#include <algorithm>
#include <cmath>

namespace ai_sapiens_sim2real
{
namespace
{
size_t positive_size(const YAML::Node & config, const char * key)
{
  const int value = config[key].as<int>();
  if (value < 1) {
    throw std::runtime_error(std::string("BlinkMimic ") + key + " must be positive");
  }
  return static_cast<size_t>(value);
}

template<typename T>
void require_value(const YAML::Node & node, const char * key, const T & expected)
{
  const auto value = node[key];
  if (!value || value.as<T>() != expected) {
    throw std::runtime_error(std::string("BlinkMimic ") + key + "=" +
      YAML::Dump(value) + "; expected " + YAML::Dump(YAML::Node(expected)));
  }
}

void validate_observation_contract(const Sim2RealConfig & config, const YAML::Node & node)
{
  if (config.observations().size() != 1) {
    throw std::runtime_error("BlinkMimic observations must contain only blinkmimic");
  }

  require_value(node, "schema", std::string("cyclo.blinkmimic.csv.v1"));
  require_value(node, "input_name", std::string("observations"));
  require_value(node, "imu_frame", std::string("pelvis"));
  require_value(node, "reference_velocity", std::string("central_difference"));
  require_value(node, "normalization_in_onnx", true);
}

void validate_timing(const Sim2RealConfig & config, const YAML::Node & node)
{
  const double fps = node["motion_fps"].as<double>();
  if (!std::isfinite(config.step_dt()) || !std::isfinite(fps) ||
    std::abs(config.step_dt() - 0.02) > 1e-9 || std::abs(fps - 50) > 1e-6)
  {
    throw std::runtime_error("BlinkMimic requires step_dt=0.02 and motion_fps=50; got " +
      std::to_string(config.step_dt()) + " and " + std::to_string(fps));
  }
}

void validate_actions(const Sim2RealConfig & config)
{
  const auto & actions = config.action_properties();
  if (actions.offset != config.joint_properties().default_position ||
    std::any_of(actions.clip.begin(), actions.clip.end(), [](const auto & clip) {
      return clip.has_value();
    }))
  {
    throw std::runtime_error(
          "BlinkMimic requires nominal action offsets and no external action clipping");
  }

  for (size_t j = 0; j < config.policy_joints().size(); ++j) {
    if (!std::isfinite(actions.offset[j]) || !std::isfinite(actions.scale[j]) ||
      actions.scale[j] == 0)
    {
      throw std::runtime_error("Invalid BlinkMimic nominal pose/action scale");
    }
  }
}

}  // namespace

BlinkMimicConfig::BlinkMimicConfig(const Sim2RealConfig & config)
: joints(config.policy_joints()), defaults(config.joint_properties().default_position)
{
  const auto node = config.observations()["blinkmimic"];
  validate_observation_contract(config, node);
  validate_timing(config, node);
  validate_actions(config);
  read_history(node["history"]);
  read_motion_goal(node["motion_goal"]);
}

void BlinkMimicConfig::read_history(const YAML::Node & node)
{
  const size_t joint_count = joints.size();
  history = ConfiguredObservationGroup(node, {
      {"joint_pos_rel", joint_count}, {"joint_vel", joint_count}, {"base_ang_vel", 3},
      {"projected_gravity", 3}, {"last_action", joint_count}, {"reference_orientation", 6}});
  history_length = history.history_length();
}

void BlinkMimicConfig::read_motion_goal(const YAML::Node & goal)
{
  const size_t joint_count = joints.size();
  future_frames = positive_size(goal, "frames");
  future_stride = positive_size(goal, "stride");
  start_offset = positive_size(goal, "start_offset");
  if (future_frames > 1024 || future_stride > 1024 || start_offset > 1024 ||
    !goal["groups"].IsSequence() || goal["groups"].size() != 1)
  {
    throw std::runtime_error("Invalid BlinkMimic goal dimensions/groups");
  }
  goal_groups.emplace_back(goal["groups"][0], std::map<std::string, size_t>{
      {"future_root_displacement", 3}, {"future_root_orientation", 6},
      {"future_root_linear_velocity", 3}, {"future_root_angular_velocity", 3},
      {"future_joint_pos", joint_count}, {"future_joint_vel", joint_count}});
  if (goal_groups[0].history_length() != 1) {
    throw std::runtime_error("BlinkMimic goal terms cannot contain history");
  }
}

void BlinkMimicConfig::validate_motion(const MimicBehavior & mimic) const
{
  if (!std::isfinite(mimic.fps) || std::abs(mimic.fps - 50) > 1e-6 ||
    mimic.time_start != 0 || mimic.time_end)
  {
    throw std::runtime_error("BlinkMimic requires a full 50 Hz motion and a 20 ms policy period");
  }
}

}  // namespace ai_sapiens_sim2real
