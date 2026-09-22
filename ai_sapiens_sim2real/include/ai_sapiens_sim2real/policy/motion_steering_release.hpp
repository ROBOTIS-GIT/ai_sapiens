// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AI_SAPIENS_SIM2REAL__POLICY__MOTION_STEERING_RELEASE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MOTION_STEERING_RELEASE_HPP_

#include "ai_sapiens_sim2real/policy/planar_motion_steering.hpp"

namespace ai_sapiens_sim2real
{

// Deployment-only override: release translation and turning independently,
// discarding missed motion while the applied command ramps down.
class MotionSteeringRelease
{
public:
  void reset()
  {
    translation_commanded_ = rotation_commanded_ = false;
    translation_settling_ = rotation_settling_ = false;
  }

  // Use this SAME command for integration and release detection. The inclusive
  // deadband is in physical command units: m/s for XY, rad/s for yaw.
  static Eigen::Vector3f filter_command(const Eigen::Vector3f & requested)
  {
    Eigen::Vector3f result = requested;
    for (int i = 0; i < 3; ++i) {
      if (std::abs(result[i]) <= 0.1f) {
        result[i] = 0.0f;
      }
    }
    return result;
  }

  // Call after the ordinary steering step, including the zero-velocity entry
  // frame, with the filtered request. All poses use the fixed motion frame.
  void apply(
    PlanarMotionSteering & steering, const Eigen::Vector3f & requested,
    const Eigen::Vector2f & robot_xy, const Eigen::Quaternionf & robot_orientation,
    const Eigen::Vector2f & reference_xy, const Eigen::Quaternionf & reference_orientation)
  {
    update_release(!requested.head<2>().isZero(0.0f),
      translation_commanded_, translation_settling_);
    update_release(requested.z() != 0.0f, rotation_commanded_, rotation_settling_);

    if (translation_settling_) {
      // The requested behavior deliberately discards target error on release.
      // Slewing this correction would leave a catch-up target after release.
      steering.offset = robot_xy - reference_xy;
      if (steering.velocity.head<2>().norm() <= 0.01f) {
        steering.velocity.head<2>().setZero();
        translation_settling_ = false;
      }
    }

    if (rotation_settling_) {
      // Translation-only release must never
      // overwrite the clip heading or a previously completed steering turn.
      steering.yaw = wrap(PlanarMotionSteering::heading(robot_orientation) -
          PlanarMotionSteering::heading(reference_orientation));
      if (std::abs(steering.velocity.z()) <= 0.01f) {
        steering.velocity.z() = 0.0f;
        rotation_settling_ = false;
      }
    }
  }

private:
  static float wrap(float angle) {return std::atan2(std::sin(angle), std::cos(angle));}

  static void update_release(bool active, bool & commanded, bool & settling)
  {
    if (active) {
      commanded = true;
      settling = false;
    } else {
      settling = settling || commanded;
      commanded = false;
    }
  }

  bool translation_commanded_{false}, rotation_commanded_{false};
  bool translation_settling_{false}, rotation_settling_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__MOTION_STEERING_RELEASE_HPP_
