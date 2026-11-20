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
// Author: Woojin Wie, Kiwoong Park

// Copyright 2026 The SuperMimic Authors.
// Observation/goal conventions adapted from ROBOTIS-GIT/ai_sapiens feature-supermimic
// (3250b6e), originating in HybridRobotics/SuperMimic (Apache-2.0).
// Changes: common CSV reference, PolicyObservation interface, retained-history switching.

#include "ai_sapiens_sim2real/policy/supermimic/supermimic_observation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "ai_sapiens_sim2real/policy/mimic/motion_source.hpp"

namespace ai_sapiens_sim2real
{
namespace
{
void append_vector(std::vector<float> & target, const Eigen::VectorXf & value)
{
  target.insert(target.end(), value.data(), value.data() + value.size());
}

float yaw(const Eigen::Matrix3f & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}
}  // namespace

SuperMimicObservation::SuperMimicObservation(
  const Sim2RealConfig & config,
  const std::optional<MimicBehavior> & mimic, const std::vector<std::string> & controller_joints,
  const SharedControlData * data)
: config_(config), data_(data), joint_indices_(make_source_indices_for_target(
      controller_joints, config.policy_joints(), "controller joints", "policy joints")),
  history_(config_.history.width(), config_.history_length)
{
  if (!data_) {throw std::runtime_error("SuperMimic requires shared sensor data");}
  if (config_.is_motion != mimic.has_value()) {
    throw std::runtime_error(
      "SuperMimic motion encoder requires kind: mimic and CSV; "
      "velocity encoder requires kind: policy");
  }
  if (mimic) {motion_ = prepare_motion(*mimic);}
}

std::shared_ptr<const SuperMimicMotion> SuperMimicObservation::prepare_motion(
  const MimicBehavior & mimic) const
{
  config_.validate_motion(mimic);
  return std::make_shared<const SuperMimicMotion>(
    mimic.require_motion_source().make_cursor(), mimic.fps, config_.joints);
}

void SuperMimicObservation::select_motion(std::shared_ptr<const SuperMimicMotion> motion)
{
  if (!motion || !config_.is_motion) {throw std::invalid_argument("Invalid SuperMimic motion");}
  motion_ = std::move(motion);
  cursor_ = 0;
  has_alignment_ = false;
  history_.discard_snapshot();
}

void SuperMimicObservation::reset()
{
  cursor_ = 0;
  has_alignment_ = false;
  history_.reset();
}

Eigen::Matrix3f SuperMimicObservation::robot_rotation() const
{
  const auto & sensors = data_->sensors;
  const auto & q = sensors.orientation;
  if (!sensors.is_imu_usable || !q.coeffs().allFinite() ||
    std::abs(q.squaredNorm() - 1.0f) > 0.02f || !sensors.angular_velocity.allFinite())
  {
    throw std::runtime_error("Invalid SuperMimic pelvis IMU sample");
  }
  return q.normalized().toRotationMatrix();
}

std::vector<float> SuperMimicObservation::sensor_frame(const Eigen::Matrix3f & rotation) const
{
  const auto & sensors = data_->sensors;
  const auto joints = config_.joints.size();
  if (data_->policy.last_action.size() < joints) {
    throw std::runtime_error("SuperMimic action feedback size mismatch");
  }
  for (const auto index : joint_indices_) {
    if (index >= static_cast<size_t>(sensors.joint_pos.size()) ||
      index >= static_cast<size_t>(sensors.joint_vel.size()))
    {
      throw std::runtime_error("SuperMimic joint sensor size mismatch");
    }
  }
  std::vector<float> frame;
  frame.reserve(config_.history.width());
  for (const auto & term : config_.history.terms()) {
    const size_t term_begin = frame.size();
    if (term.name == "projected_gravity") {
      append_vector(frame, -rotation.row(2).transpose());
    } else if (term.name == "base_ang_vel") {
      append_vector(frame, sensors.angular_velocity);
    } else if (term.name == "joint_pos_rel") {
      for (size_t j = 0; j < joints; ++j) {
        frame.push_back(sensors.joint_pos[joint_indices_[j]] - config_.defaults[j]);
      }
    } else if (term.name == "joint_vel_rel") {
      for (size_t j = 0; j < joints; ++j) {
        frame.push_back(sensors.joint_vel[joint_indices_[j]] - config_.default_velocities[j]);
      }
    } else if (term.name == "last_action") {
      frame.insert(frame.end(), data_->policy.last_action.begin(),
        data_->policy.last_action.begin() + joints);
    }
    term.apply(frame, term_begin, frame.size());
  }
  if (!std::all_of(frame.begin(), frame.end(), [](float value) {return std::isfinite(value);})) {
    throw std::runtime_error("Nonfinite SuperMimic sensor/action history sample");
  }
  return frame;
}

std::vector<float> SuperMimicObservation::compute()
{
  history_.discard_snapshot();

  if (!has_frame()) {throw std::runtime_error("SuperMimic motion has ended");}
  history_.begin_sample();
  had_alignment_before_sample_ = has_alignment_;
  const auto rotation = robot_rotation();
  const auto frame = sensor_frame(rotation);
  history_.append(frame);

  std::vector<float> input;
  input.reserve(get_observation_size());
  config_.history.append_frames(input, history_.frames());
  append_goal(input, rotation);
  return input;
}

void SuperMimicObservation::append_goal(
  std::vector<float> & input,
  const Eigen::Matrix3f & rotation)
{
  if (!motion_) {
    if (!data_->mode.velocity_commands.allFinite()) {
      throw std::runtime_error("Nonfinite SuperMimic velocity command");
    }
    for (const auto & term : config_.goal_groups[0].terms()) {
      const size_t term_begin = input.size();
      append_vector(input, data_->mode.velocity_commands);
      term.apply(input, term_begin, input.size());
    }
    return;
  }
  if (!has_alignment_) {
    alignment_ = Eigen::AngleAxisf(yaw(motion_->frame(0).rotation) - yaw(rotation),
      Eigen::Vector3f::UnitZ()).toRotationMatrix();
    has_alignment_ = true;
  }
  const Eigen::Matrix3f robot_inverse = (alignment_ * rotation).transpose();
  for (const auto & group : config_.goal_groups) {
    std::vector<float> goals;
    goals.reserve(config_.future_frames * group.width());
    for (size_t k = 0; k < config_.future_frames; ++k) {
      const auto & frame = motion_->frame(std::min(
        cursor_ + config_.start_offset + k * config_.future_stride, motion_->size() - 1));
      for (const auto & term : group.terms()) {
        const size_t term_begin = goals.size();
        if (term.name == "future_joint_pos") {
          append_vector(goals, frame.joint_position);
        } else if (term.name == "future_joint_vel") {
          append_vector(goals, frame.joint_velocity);
        } else if (term.name == "future_root_orientation") {
          const Eigen::Matrix3f relative = robot_inverse * frame.rotation;
          for (int row = 0; row < 3; ++row) {
            goals.push_back(relative(row, 0));
            goals.push_back(relative(row, 1));
          }
        }
        term.apply(goals, term_begin, goals.size());
      }
    }
    group.append_frames(input, goals);
  }
}

void SuperMimicObservation::reject_latest()
{
  if (history_.reject_latest()) {
    // Recompute an initial alignment if its sensor sample was rejected.
    has_alignment_ = had_alignment_before_sample_;
  }
}

std::vector<ObservationSlice> SuperMimicObservation::layout() const
{
  auto slices = config_.history.layout(0, config_.history_length);
  size_t offset = config_.history.width() * config_.history_length;
  const size_t frames = config_.is_motion ? config_.future_frames : 1;
  for (const auto & group : config_.goal_groups) {
    const auto goals = group.layout(offset, frames);
    slices.insert(slices.end(), goals.begin(), goals.end());
    offset += group.width() * frames;
  }
  return slices;
}

}  // namespace ai_sapiens_sim2real
