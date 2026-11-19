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

// Author: Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_VALIDATION_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_VALIDATION_HPP_

#include <Eigen/Geometry>

#include <cmath>

namespace ai_sapiens_sim2real
{
// Allow modest unit-length drift, not arbitrary vectors disguised as rotations.
inline bool valid_imu_sample(
  const Eigen::Quaternionf & orientation, const Eigen::Vector3f & angular_velocity,
  bool orientation_available, bool angular_velocity_available)
{
  if (!orientation_available || !angular_velocity_available || !orientation.coeffs().allFinite() ||
    !angular_velocity.allFinite())
  {
    return false;
  }
  const float norm = orientation.norm();
  return std::isfinite(norm) && norm >= 0.9f && norm <= 1.1f;
}
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_VALIDATION_HPP_
