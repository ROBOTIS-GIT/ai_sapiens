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
    load_playback(mimic, controller_joint_names),
    mimic.on_complete);
}

MotionPlayback MimicPolicyRuntime::load_playback(
  const MimicBehavior & mimic,
  const std::vector<std::string> & controller_joint_names)
{
  MotionPlayback playback;
  playback.reference = std::make_shared<MotionReference>(
    mimic.motion_file.string(), mimic.fps, controller_joint_names);
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
  std::string completion_state)
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
{
}

void MimicPolicyRuntime::on_enter()
{
  playback_.reference->seek(playback_.time_start);
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

Eigen::Quaternionf MimicPolicyRuntime::yaw_quaternion(const Eigen::Quaternionf & q)
{
  const float yaw = std::atan2(
          2.0f * (q.w() * q.z() + q.x() * q.y()),
          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  const float half_yaw = yaw * 0.5f;
  return Eigen::Quaternionf(std::cos(half_yaw), 0.0f, 0.0f, std::sin(half_yaw)).normalized();
}

}  // namespace ai_sapiens_sim2real
