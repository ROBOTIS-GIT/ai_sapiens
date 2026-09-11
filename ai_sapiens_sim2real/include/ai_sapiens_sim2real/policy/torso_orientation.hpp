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
// Author: Woojin Wie

#ifndef AI_SAPIENS_SIM2REAL__POLICY__TORSO_ORIENTATION_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__TORSO_ORIENTATION_HPP_

#include <array>
#include <cmath>

#include <Eigen/Dense>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

// The robot joint that yaws the torso relative to the base/pelvis.
inline constexpr char kWaistYawJointName[] = "waist_yaw_joint";

/**
 * @brief Torso orientation in the world frame.
 *
 * The deploy stack does not run full forward kinematics for the motion anchor.
 * Instead, it composes the base/root IMU orientation with the measured
 * waist_yaw joint. This keeps the convention shared between the motion
 * alignment captured on entry and the motion_anchor_ori_b observation.
 *
 * If the active policy does not include the waist joint, this returns the base
 * orientation unchanged.
 */
inline Eigen::Quaternionf torso_orientation_in_world(
  const SensorData & sensors, const PolicyJointContext & joints)
{
  float waist_yaw = 0.0f;
  for (size_t i = 0; i < joints.policy_joint_names.size(); ++i) {
    if (joints.policy_joint_names[i] == kWaistYawJointName) {
      waist_yaw = sensors.joint_pos(static_cast<Eigen::Index>(joints.policy_to_controller[i]));
      break;
    }
  }

  return sensors.orientation * Eigen::AngleAxisf(waist_yaw, Eigen::Vector3f::UnitZ());
}

// OpenTrack orientation contract v1: fix the initial yaw offset, then express
// the aligned reference in the measured pelvis frame (never the torso frame).
inline Eigen::Quaternionf initial_pelvis_alignment(
  const Eigen::Quaternionf & robot, const Eigen::Quaternionf & reference)
{
  auto yaw = [](const Eigen::Quaternionf & input) {
      const auto q = input.normalized();
      return std::atan2(2 * (q.w()*q.z() + q.x()*q.y()),
               1 - 2 * (q.y()*q.y() + q.z()*q.z()));
    };
  return Eigen::Quaternionf(Eigen::AngleAxisf(yaw(robot)-yaw(reference), Eigen::Vector3f::UnitZ()));
}
inline std::array<float, 6> pelvis_orientation_observation(
  const Eigen::Quaternionf & robot, const Eigen::Quaternionf & reference,
  const Eigen::Quaternionf & initial_alignment)
{
  const auto rotation = (robot.normalized().conjugate() * initial_alignment *
    reference.normalized()).normalized().toRotationMatrix();
  return {rotation(0,0), rotation(0,1), rotation(1,0), rotation(1,1), rotation(2,0), rotation(2,1)};
}

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__TORSO_ORIENTATION_HPP_
