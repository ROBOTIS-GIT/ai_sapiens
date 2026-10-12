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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__ANY2TRACK_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__ANY2TRACK_CONFIG_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace ai_sapiens_sim2real
{
// Exact ABI exported by humanoid_motion/source/tasks/any2track/export.py.
inline const std::vector<std::string> & any2track_observation_names()
{
  static const std::vector<std::string> names{
    "dif_joint_pos", "dif_joint_vel", "gvec_pelvis", "gyro_pelvis", "joint_pos",
    "joint_vel", "last_motor_targets", "motion_anchor_ori_b", "path_position_error",
    "ref_feet_height", "ref_root_height"};
  return names;
}

inline void validate_any2track_config(const YAML::Node & n)
{
  const auto require = [](bool ok, const std::string & what) {
      if (!ok) {throw std::runtime_error("Any2Track contract: " + what);}
    };
  require(n["runtime_type"].as<std::string>() == "any2track_gt_xy_adapter",
      "unsupported runtime_type");
  require(std::abs(n["step_dt"].as<double>() - 0.02) < 1e-9, "step_dt must be 0.02");
  require(n["history_length"].as<int>() == 79, "history_length must be 79");
  require(std::abs(n["joint_vel_scale"].as<double>() - 0.05) < 1e-9,
      "joint_vel_scale must be 0.05");
  require(n["policy_joints"].size() == 23, "expected 23 joints");
  require(n["commands"].IsMap() && n["commands"].size() == 0, "commands must be empty");
  require(n["torque_limits"].IsSequence() && n["torque_limits"].size() == 23,
    "expected 23 torque limits");
  for (const auto & limit : n["torque_limits"]) {
    require(std::isfinite(limit.as<float>()) && limit.as<float>() > 0, "invalid torque limit");
  }
  for (const auto & joint : n["policy_joints"]) {
    const auto props = n["joint_properties"][joint.as<std::string>()];
    for (const auto * key : {"default_position", "stiffness", "damping"}) {
      require(std::isfinite(props[key].as<float>()), "nonfinite joint property");
    }
    require(props["stiffness"].as<float>() >= 0 && props["damping"].as<float>() >= 0,
      "negative joint gain");
  }
  const auto p = n["path_tracking"];
  require(p["version"].as<int>() == 1 && p["source"].as<std::string>() == "ground_truth" &&
    p["anchor"].as<std::string>() == "pelvis" && p["frame"].as<std::string>() == "robot_heading" &&
    p["reference"].as<std::string>() == "fixed_motion_world" && p["simulation_only"].as<bool>() &&
    p["reference_time_offset_steps"].as<int>() == 1 &&
      p["motion_end"].as<std::string>() == "truncate",
    "unsupported path_tracking");
  const auto o = n["orientation_tracking"];
  require(o["version"].as<int>() == 1 && o["anchor"].as<std::string>() == "pelvis" &&
    o["alignment"].as<std::string>() == "initial_yaw" &&
    o["representation"].as<std::string>() == "relative_rotation_first_two_columns_row_major" &&
    o["quaternion_order"].as<std::string>() == "wxyz" &&
    o["requires_pelvis_orientation_estimate"].as<bool>() &&
    o["reference_time_offset_steps"].as<int>() == 1, "unsupported orientation_tracking");
  const auto v = n["velocity_estimation"];
  require(v["version"].as<int>() == 1 && v["history_length"].as<int>() == 20,
      "unsupported velocity history");
  const auto a = n["actions"]["reference_residual"];
  require(n["actions"].size() == 1 && a.IsMap() && a["clip"].IsNull() &&
    a["offset"].IsSequence() && a["offset"].size() == 0 && a["scale"].size() == 23,
    "expected unclipped reference_residual with empty offset");
  for (const auto & s : a["scale"]) {
    require(std::isfinite(s.as<float>()) && s.as<float>() > 0, "invalid action scale");
  }
  const std::vector<int> dims{23, 23, 3, 3, 23, 23, 23, 6, 2, 4, 1};
  const auto obs = n["observations"];
  require(obs.size() == dims.size(), "unexpected observation terms");
  size_t i = 0;
  for (const auto & term : obs) {
    require(term.first.as<std::string>() == any2track_observation_names()[i],
        "observation order mismatch");
    const auto t = term.second;
    require(t["dimension"].as<int>() == dims[i] && t["history_length"].as<int>() == 1 &&
      t["clip"].IsNull() && t["params"].size() == 0 &&
        t["scale"].size() == static_cast<size_t>(dims[i]),
      "observation shape/transform mismatch");
    for (const auto & s : t["scale"]) {
      require(s.as<float>() == 1.0f, "observation scale must be one");
    }
    ++i;
  }
}
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__CONFIG__ANY2TRACK_CONFIG_HPP_
