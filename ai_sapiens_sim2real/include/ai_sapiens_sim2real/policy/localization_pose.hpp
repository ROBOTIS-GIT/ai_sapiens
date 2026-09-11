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

// Robot and reference displacements from their respective episode start positions,
// expressed in the motion's world axes. Capture both origins and yaw once per entry.
class MotionFrameAlignment
{
public:
  void reset()
  {
    rotation_ = Eigen::Quaternionf::Identity();
    odom_origin_.setZero();
    reference_origin_.setZero();
  }

  void align(
    const Eigen::Vector2f & robot_xy, const Eigen::Quaternionf & robot_root,
    const Eigen::Vector2f & reference_xy, const Eigen::Quaternionf & reference_root)
  {
    rotation_ = Eigen::AngleAxisf(yaw(reference_root) - yaw(robot_root),
      Eigen::Vector3f::UnitZ());
    odom_origin_ = robot_xy;
    reference_origin_ = reference_xy;
  }

  Eigen::Vector2f position(const Eigen::Vector2f & odom_xy) const
  {
    return rotate(odom_xy - odom_origin_);
  }

  Eigen::Vector2f reference_position(const Eigen::Vector2f & reference_xy) const
  {
    return reference_xy - reference_origin_;
  }

  Eigen::Quaternionf orientation(const Eigen::Quaternionf & odom_root) const
  {
    return rotation_ * odom_root;
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
  Eigen::Vector2f odom_origin_{Eigen::Vector2f::Zero()};
  Eigen::Vector2f reference_origin_{Eigen::Vector2f::Zero()};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__LOCALIZATION_POSE_HPP_
