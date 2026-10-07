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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__LOCALIZATION_POSE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__LOCALIZATION_POSE_HPP_

#include <cmath>
#include <Eigen/Geometry>

#include "ai_sapiens_sim2real/config/motion_observation_origin.hpp"

namespace ai_sapiens_sim2real
{

// Localization is optional for the node, mandatory for policies declaring global XY.
struct LocalizationData
{
  Eigen::Vector2f position{Eigen::Vector2f::Zero()};
  Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
  bool valid{false};
  bool align_on_entry{true};
};

// Align odometry to the motion's world axes once per entry. Episode coordinates
// start at zero; motion coordinates retain the reference's original XY values.
class MotionFrameAlignment
{
public:
  void reset()
  {
    rotation_ = Eigen::Quaternionf::Identity();
    imu_rotation_ = Eigen::Quaternionf::Identity();
    use_imu_orientation_ = false;
    odom_origin_.setZero();
    reference_origin_.setZero();
    robot_offset_.setZero();
  }

  void align(
    const Eigen::Vector2f & robot_xy, const Eigen::Quaternionf & robot_root,
    const Eigen::Vector2f & reference_xy, const Eigen::Quaternionf & reference_root,
    MotionObservationOrigin origin = MotionObservationOrigin::Episode)
  {
    rotation_ = Eigen::AngleAxisf(yaw(reference_root) - yaw(robot_root),
      Eigen::Vector3f::UnitZ());
    odom_origin_ = robot_xy;
    reference_origin_ = reference_xy;
    robot_offset_.setZero();
    if (origin == MotionObservationOrigin::Motion) {
      reference_origin_.setZero();
      robot_offset_ = reference_xy;
    }
  }

  Eigen::Vector2f position(const Eigen::Vector2f & odom_xy) const
  {
    return rotate(odom_xy - odom_origin_) + robot_offset_;
  }

  Eigen::Vector2f reference_position(const Eigen::Vector2f & reference_xy) const
  {
    return reference_xy - reference_origin_;
  }

  Eigen::Quaternionf orientation(const Eigen::Quaternionf & odom_root) const
  {
    return rotation_ * odom_root;
  }

  // IMU and odometry may have different world headings. Align each source
  // independently; using the odom rotation on IMU attitude mixes two frames.
  // Only yaw is aligned, preserving the robot's measured roll and pitch.
  void use_imu_orientation(
    const Eigen::Quaternionf & imu_root, const Eigen::Quaternionf & reference_root,
    bool align_on_entry)
  {
    use_imu_orientation_ = true;
    imu_rotation_ = Eigen::Quaternionf::Identity();
    if (align_on_entry) {
      imu_rotation_ = Eigen::AngleAxisf(
        yaw(reference_root) - yaw(imu_root.normalized()), Eigen::Vector3f::UnitZ());
    }
  }

  Eigen::Quaternionf orientation(
    const Eigen::Quaternionf & odom_root, const Eigen::Quaternionf & imu_root) const
  {
    return use_imu_orientation_ ? imu_rotation_ * imu_root.normalized() : orientation(odom_root);
  }

private:
  static float yaw(const Eigen::Quaternionf & q)
  {
    return std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
      1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  }

  Eigen::Vector2f rotate(const Eigen::Vector2f & xy) const
  {
    return (rotation_ * Eigen::Vector3f(xy.x(), xy.y(), 0.0f)).head<2>();
  }

  Eigen::Quaternionf rotation_{Eigen::Quaternionf::Identity()};
  Eigen::Quaternionf imu_rotation_{Eigen::Quaternionf::Identity()};
  bool use_imu_orientation_{false};
  Eigen::Vector2f odom_origin_{Eigen::Vector2f::Zero()};
  Eigen::Vector2f reference_origin_{Eigen::Vector2f::Zero()};
  Eigen::Vector2f robot_offset_{Eigen::Vector2f::Zero()};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__LOCALIZATION_POSE_HPP_
