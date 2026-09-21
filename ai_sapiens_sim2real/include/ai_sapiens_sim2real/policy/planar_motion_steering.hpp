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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__PLANAR_MOTION_STEERING_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__PLANAR_MOTION_STEERING_HPP_

#include <cmath>

#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/config/planar_steering_config.hpp"

namespace ai_sapiens_sim2real
{

// Actor-visible part of cyclo_mjlab's PlanarMotionSteering. All vectors use the
// fixed motion frame, except velocity (additional robot-heading vx/vy/yaw rate).
struct PlanarMotionSteering
{
  Eigen::Vector2f offset{Eigen::Vector2f::Zero()};
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};
  float yaw{0.0f};
  Eigen::Vector2f measured_velocity{Eigen::Vector2f::Zero()};
  Eigen::Vector2f previous_robot_position{Eigen::Vector2f::Zero()};

  void reset()
  {
    offset.setZero();
    velocity.setZero();
    yaw = 0.0f;
    measured_velocity.setZero();
    previous_robot_position.setZero();
  }

  Eigen::Quaternionf orientation() const
  {
    return Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  }

  static float heading(const Eigen::Quaternionf & q)
  {
    return std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
      1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  }

  void update_command(
    const Eigen::Vector3f & requested, float dt, const PlanarSteeringConfig & config)
  {
    const Eigen::Vector3f target(
      clamp_to_axis(requested.x(), config.ranges.linear_x),
      clamp_to_axis(requested.y(), config.ranges.linear_y),
      clamp_to_axis(requested.z(), config.ranges.angular_z));
    const float tau = config.smoothing_time_constant;
    const float alpha = tau == 0.0f ? 1.0f : -std::expm1(-dt / tau);
    velocity += alpha * (target - velocity);
  }

  void reanchor(
    const Eigen::Vector2f & root, const Eigen::Quaternionf & reference_orientation,
    const Eigen::Vector2f & robot_position, const Eigen::Quaternionf & robot_orientation)
  {
    offset = robot_position - root;
    const float angle = heading(robot_orientation) - heading(reference_orientation);
    yaw = std::atan2(std::sin(angle), std::cos(angle));
  }

  void reset_velocity_estimator(const Eigen::Vector2f & robot_position)
  {
    previous_robot_position = robot_position;
    measured_velocity.setZero();
  }

  void step_velocity(
    const Eigen::Vector2f & root, const Eigen::Quaternionf & reference_orientation,
    const Eigen::Vector2f & robot_position, const Eigen::Quaternionf & robot_orientation,
    const Eigen::Vector3f & requested, float dt, const PlanarSteeringConfig & config)
  {
    update_command(requested, dt, config);
    const float tau = config.velocity_estimator_time_constant;
    const float alpha = tau == 0.0f ? 1.0f : -std::expm1(-dt / tau);
    measured_velocity += alpha * ((robot_position - previous_robot_position) / dt - measured_velocity);
    previous_robot_position = robot_position;
    reanchor(root, reference_orientation, robot_position, robot_orientation);
  }

  void step(
    const Eigen::Vector2f & previous_root, const Eigen::Vector2f & current_root,
    const Eigen::Quaternionf & robot_orientation, const Eigen::Vector3f & requested,
    float dt, const PlanarSteeringConfig & config)
  {
    if (config.tracking_mode != "trajectory") {
      throw std::runtime_error("velocity steering requires measured position via step_velocity");
    }
    update_command(requested, dt, config);
    const float delta_yaw = dt * velocity.z();
    const Eigen::Vector2f root_delta = current_root - previous_root;
    // Redirect the clip's own displacement using midpoint yaw. Rotating the
    // complete clip about its starting point would introduce a different path.
    offset += Eigen::Rotation2Df(yaw + 0.5f * delta_yaw) * root_delta - root_delta +
      dt * (Eigen::Rotation2Df(heading(robot_orientation)) * velocity.head<2>());
    yaw += delta_yaw;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  }
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__PLANAR_MOTION_STEERING_HPP_
