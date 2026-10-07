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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__DANCE_REFERENCE_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__DANCE_REFERENCE_CONFIG_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace ai_sapiens_sim2real
{

// Export contract of humanoid_motion FootstepReferenceCommand, versions 4--6.
struct DanceReferenceConfig
{
  float min_stance_duration, support_transfer_duration, support_reach, max_step_lift;
  float weight_shift, moving_crouch, max_step_correction, max_step_yaw;
  float ik_joint_correction_limit, ik_max_correction_rate;
  float ik_position_tolerance, ik_orientation_tolerance;
  int ik_iterations;
  bool stop_new_steps_on_release{false};
  bool ik_monotonic{false};
  float ik_damping{.0004f};
  std::string training_reference_file;
  std::string training_reference_sha256;

  static DanceReferenceConfig read(const YAML::Node & reference, double dt)
  {
    DanceReferenceConfig c;
    const auto s = reference["dance_steps"];
    if (!s.IsMap() || reference["observation_origin"].as<std::string>() != "episode" ||
      reference["motion_command_source"].as<std::string>() !=
      "retargeted_joint_position_velocity")
    {
      throw std::runtime_error(
          "Dance reference requires retargeted joints and episode origin");
    }
    // Exact planner inputs are exported alongside the deployment configuration.
    // Legacy bundles without this file retain their CSV-derived approximation.
    if (s["training_reference_file"]) {
      c.training_reference_file = s["training_reference_file"].as<std::string>();
      if (c.training_reference_file.empty()) {
        throw std::runtime_error("Declared training_reference_file must not be empty");
      }
      c.training_reference_sha256 = s["training_reference_sha256"].as<std::string>("");
      if (c.training_reference_sha256.size() != 64 ||
        !std::all_of(c.training_reference_sha256.begin(), c.training_reference_sha256.end(),
        [](char x) {return (x >= '0' && x <= '9') || (x >= 'a' && x <= 'f');}))
      {
        throw std::runtime_error("training_reference_file requires its exported SHA256");
      }
    } else if (s["training_reference_sha256"]) {
      throw std::runtime_error("training_reference_sha256 requires training_reference_file");
    }
    const auto features = reference["required_runtime_features"];
    const bool v4 = features && features.IsSequence() && features.size() == 1 &&
      features[0].as<std::string>() == "dance_motion_reference_v4";
    c.stop_new_steps_on_release = features && features.IsSequence() && features.size() == 1 &&
      (features[0].as<std::string>() == "dance_motion_reference_v3" || v4);
    c.ik_monotonic = s["ik_monotonic"].as<bool>(false);
    if ((v4 && (!c.ik_monotonic || c.training_reference_file.empty() || !s["ik_damping"])) ||
      (!v4 && c.ik_monotonic))
    {
      throw std::runtime_error("V4 requires exact training reference data and monotonic IK");
    }
    if (c.stop_new_steps_on_release) {
      if (s["release_behavior"].as<std::string>("") != "finish_active_gesture_no_new_steps" ||
        s["step_intent_source"].as<std::string>("") != "deadbanded_request_before_smoothing")
      {
        throw std::runtime_error(
            "Unsupported v3 dance_steps release_behavior or step_intent_source");
      }
    } else if (s["release_behavior"] || s["step_intent_source"]) {
      throw std::runtime_error("Dance release settings require dance_motion_reference_v3");
    }
    const auto number = [&](const char * key) {
        const float value = s[key].as<float>();
        if (!std::isfinite(value)) {
          throw std::runtime_error(std::string("Non-finite dance_steps.") + key);
        }
        return value;
      };
    c.min_stance_duration = number("min_stance_duration");
    c.support_transfer_duration = number("support_transfer_duration");
    c.support_reach = number("support_reach");
    c.max_step_lift = number("max_step_lift");
    c.weight_shift = number("weight_shift");
    c.moving_crouch = number("moving_crouch");
    c.max_step_correction = number("max_step_correction");
    c.max_step_yaw = number("max_step_yaw");
    c.ik_iterations = s["ik_iterations"].as<int>();
    c.ik_damping = s["ik_damping"].as<float>(.0004f);
    if (!std::isfinite(c.ik_damping) || c.ik_damping <= 0) {
      throw std::runtime_error("Invalid dance_steps.ik_damping");
    }
    c.ik_joint_correction_limit = number("ik_joint_correction_limit");
    c.ik_max_correction_rate = number("ik_max_correction_rate");
    c.ik_position_tolerance = number("ik_position_tolerance");
    c.ik_orientation_tolerance = number("ik_orientation_tolerance");
    if (!(dt > 0 && c.min_stance_duration >= dt &&
      c.support_transfer_duration >= dt && c.support_transfer_duration <= c.min_stance_duration &&
      c.support_reach >= .04f && c.support_reach <= .2f && c.max_step_lift > 0 &&
      c.weight_shift >= 0 && c.weight_shift <= .05f && c.moving_crouch >= 0 &&
      c.moving_crouch <= .05f && c.max_step_correction > 0 && c.max_step_correction <= .3f &&
      c.max_step_yaw > 0 && c.max_step_yaw <= .5f &&
      c.ik_iterations >= 1 && c.ik_iterations <= 10 && c.ik_joint_correction_limit > 0 &&
      c.ik_max_correction_rate > 0 && c.ik_position_tolerance > 0 &&
      c.ik_orientation_tolerance > 0))
    {
      throw std::runtime_error("Invalid dance_steps planner/IK limits");
    }
    if (s["gesture_source"].as<std::string>() != "reference_trajectory" ||
      s["scheduling"].as<std::string>() != "reach_and_source_motion")
    {
      throw std::runtime_error("Unsupported dance_steps source or scheduling");
    }
    return c;
  }
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__DANCE_REFERENCE_CONFIG_HPP_
