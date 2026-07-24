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

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__TORSO_ORIENTATION_HPP_
