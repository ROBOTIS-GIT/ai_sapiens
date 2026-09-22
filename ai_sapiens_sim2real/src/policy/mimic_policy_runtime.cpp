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

#include "ai_sapiens_sim2real/policy/mimic_policy_runtime.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "ai_sapiens_sim2real/config/mimic_behavior.hpp"
#include "ai_sapiens_sim2real/policy/torso_orientation.hpp"

namespace ai_sapiens_sim2real
{

std::unique_ptr<MimicPolicyRuntime> MimicPolicyRuntime::create(
  rclcpp::Node::SharedPtr node,
  std::string state_name,
  std::string model_path,
  const Sim2RealConfig & sim2real_config,
  const MimicBehavior & mimic,
  const std::vector<std::string> & controller_joint_names,
  SharedControlData * shared_data)
{
  return std::make_unique<MimicPolicyRuntime>(
    std::move(node),
    std::move(state_name),
    std::move(model_path),
    sim2real_config,
    controller_joint_names,
    shared_data,
    load_playback(mimic, sim2real_config, controller_joint_names),
    mimic.on_complete,
    mimic.reanchor_on_release);
}

MotionPlayback MimicPolicyRuntime::load_playback(
  const MimicBehavior & mimic,
  const Sim2RealConfig & sim2real_config,
  const std::vector<std::string> & controller_joint_names)
{
  MotionPlayback playback;
  const bool mjlab_format = mimic.mjlab_format.value_or(
    static_cast<bool>(sim2real_config.observations()["robot_root_position_xy_w"]));
  if (sim2real_config.steering() && (!mjlab_format || !std::isfinite(mimic.fps) ||
    !std::isfinite(sim2real_config.step_dt()) ||
    std::abs(mimic.fps * sim2real_config.step_dt() - 1.0) > 1e-5))
  {
    throw std::runtime_error("mimic steering requires MJLab motion with fps * step_dt == 1");
  }
  playback.reference = std::make_shared<MotionReference>(
    mimic.motion_file.string(), mimic.fps, controller_joint_names, mjlab_format);
  const float duration = playback.reference->duration();
  // Reject an obviously bad window on the raw config (before clamping hides it).
  const float requested_end = mimic.time_end.value_or(duration);
  if (mimic.time_start < 0.0f || mimic.time_start > requested_end) {
    throw std::runtime_error("mimic time_start must be non-negative and not exceed time_end");
  }

  playback.time_start = std::clamp(mimic.time_start, 0.0f, duration);
  playback.time_end = std::clamp(requested_end, 0.0f, duration);
  return playback;
}

MimicPolicyRuntime::MimicPolicyRuntime(
  rclcpp::Node::SharedPtr node,
  std::string state_name,
  std::string model_path,
  const Sim2RealConfig & sim2real_config,
  const std::vector<std::string> & controller_joint_names,
  SharedControlData * shared_data,
  MotionPlayback playback,
  std::string completion_state,
  bool reanchor_on_release)
: PolicyRuntime(
    std::move(node),
    std::move(state_name),
    std::move(model_path),
    sim2real_config,
    controller_joint_names,
    shared_data,
    playback.reference.get())
  , playback_(std::move(playback))
  , completion_state_(std::move(completion_state))
  , steering_(sim2real_config.steering())
  , steering_dt_(static_cast<float>(sim2real_config.step_dt()))
  , reanchor_on_release_(reanchor_on_release)
{
  if (reanchor_on_release_ && !steering_) {
    throw std::runtime_error("reanchor_on_release requires a localized Mimic steering policy");
  }
  const bool has_velocity = static_cast<bool>(sim2real_config.observations()["velocity_commands"]);
  if (steering_) {
    steering_->validate();
    if (!requires_localization() || !has_velocity) {
      throw std::runtime_error(
          "mimic steering requires localized root feedback and velocity_commands observations");
    }
    set_velocity_command_ranges(steering_->ranges);
  } else if (requires_localization() && has_velocity) {
    throw std::runtime_error(
        "global-position mimic requires commands.reference_trajectory.steering in sim2real.yaml");
  }
}

void MimicPolicyRuntime::on_enter()
{
  steering_release_.reset();
  playback_.reference->seek(playback_.time_start);
  policy_->uses_motion_steering = steering_.has_value();
  previous_root_ = playback_.reference->root_position().head<2>();
  if (requires_localization()) {
    const auto & localization = shared_data_->localization;
    if (localization.align_on_entry) {
      // Snapshot again on every entry, including Velocity -> Mimic after walking.
      // The estimator keeps its continuous odom; only policy coordinates restart.
      policy_->motion_frame.align(localization.position, localization.orientation,
        playback_.reference->root_position().head<2>(), playback_.reference->root_quaternion());
    }
    // Global-position observations and anchor orientation share the same motion frame.
    policy_->motion_init_quat = Eigen::Quaternionf::Identity();
    return;
  }
  const auto ref_yaw = yaw_quaternion(playback_.reference->root_quaternion()).toRotationMatrix();
  const auto robot_yaw =
    yaw_quaternion(torso_orientation_in_world(*sensors_, joint_context())).toRotationMatrix();
  policy_->motion_init_quat = Eigen::Quaternionf(robot_yaw * ref_yaw.transpose());
}

bool MimicPolicyRuntime::prepare_observation()
{
  // Window ended: hand off to the completion state; false stops this tick's step.
  const auto motion_time = playback_.seek_time(policy_->episode_time);
  if (!motion_time) {
    if (completion_state_ != "stay") {
      requests_->state_name = completion_state_;
    }

    return false;
  }

  playback_.reference->seek(*motion_time);
  return true;
}

void MimicPolicyRuntime::prepare_command_observation()
{
  if (!steering_) {
    return;
  }
  const Eigen::Vector2f root = playback_.reference->root_position().head<2>();
  const auto orientation =
    policy_->motion_frame.orientation(shared_data_->localization.orientation);
  const Eigen::Vector3f requested = reanchor_on_release_ ?
    steering_release_.filter_command(shared_data_->mode.velocity_commands) :
    shared_data_->mode.velocity_commands;
  // Training reset observes frame zero with zero applied velocity, even for a held command.
  if (policy_->episode_time > 0.0) {
    policy_->motion_steering.step(previous_root_, root, orientation,
      requested, steering_dt_, *steering_);
  }
  if (reanchor_on_release_) {
    // Use episode-relative coordinates on both sides. Do not reset odometry,
    // the motion frame, playback time, or the original joint reference.
    steering_release_.apply(policy_->motion_steering, requested,
      policy_->motion_frame.position(shared_data_->localization.position), orientation,
      policy_->motion_frame.reference_position(root), playback_.reference->root_quaternion());
  }
  previous_root_ = root;
}

Eigen::Quaternionf MimicPolicyRuntime::yaw_quaternion(const Eigen::Quaternionf & q)
{
  const float yaw = std::atan2(
          2.0f * (q.w() * q.z() + q.x() * q.y()),
          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  const float half_yaw = yaw * 0.5f;
  return Eigen::Quaternionf(std::cos(half_yaw), 0.0f, 0.0f, std::sin(half_yaw)).normalized();
}

}  // namespace ai_sapiens_sim2real
