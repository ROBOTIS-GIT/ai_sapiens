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

#include "ai_sapiens_sim2real/policy/blinkmimic/blinkmimic_observation.hpp"

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

void append_rotation(std::vector<float> & target, const Eigen::Matrix3f & rotation)
{
  // Training flattens the first two columns in row-major order.
  for (int row = 0; row < 3; ++row) {
    target.push_back(rotation(row, 0));
    target.push_back(rotation(row, 1));
  }
}

float yaw(const Eigen::Matrix3f & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

}  // namespace

BlinkMimicObservation::BlinkMimicObservation(
  const Sim2RealConfig & config, const MimicBehavior & mimic,
  const std::vector<std::string> & controller_joints, const SharedControlData * data)
: config_(config), data_(data), motion_(prepare_motion(mimic)),
  joint_indices_(make_source_indices_for_target(
      controller_joints, config_.joints, "controller joints", "policy joints")),
  history_(config_.history.width(), config_.history_length)
{
  if (!data_) {throw std::runtime_error("BlinkMimic requires shared sensor data");}
}

void BlinkMimicObservation::reset()
{
  cursor_ = 0;
  has_alignment_ = false;
  history_.reset();
}

std::shared_ptr<const BlinkMimicMotion> BlinkMimicObservation::prepare_motion(
  const MimicBehavior & mimic) const
{
  config_.validate_motion(mimic);
  return std::make_shared<const BlinkMimicMotion>(
    mimic.require_motion_source().make_cursor(), mimic.fps, config_.joints);
}

void BlinkMimicObservation::select_motion(std::shared_ptr<const BlinkMimicMotion> motion)
{
  if (!motion) {throw std::invalid_argument("BlinkMimic motion must not be null");}

  motion_ = std::move(motion);
  cursor_ = 0;
  has_alignment_ = false;
  history_.discard_snapshot();
}

size_t BlinkMimicObservation::get_observation_size() const
{
  return config_.history_length * config_.history.width() +
         config_.future_frames * config_.goal_groups[0].width();
}

std::vector<float> BlinkMimicObservation::sensor_frame()
{
  const auto & sensors = data_->sensors;
  const auto & raw = sensors.orientation;
  if (!sensors.is_imu_usable || !raw.coeffs().allFinite() ||
    std::abs(raw.squaredNorm() - 1.0f) > 0.02f || !sensors.angular_velocity.allFinite())
  {
    throw std::runtime_error("Invalid BlinkMimic pelvis IMU sample");
  }
  const Eigen::Matrix3f robot = raw.normalized().toRotationMatrix();
  if (!has_alignment_) {
    alignment_ = Eigen::AngleAxisf(yaw(motion_->frame(0).rotation) - yaw(robot),
        Eigen::Vector3f::UnitZ()).toRotationMatrix();
    has_alignment_ = true;
  }

  std::vector<float> frame;
  frame.reserve(config_.history.width());
  for (const auto & term : config_.history.terms()) {
    const size_t term_begin = frame.size();
    if (term.name == "joint_pos_rel") {
      for (size_t j = 0; j < config_.defaults.size(); ++j) {
        frame.push_back(sensors.joint_pos[joint_indices_[j]] - config_.defaults[j]);
      }
    } else if (term.name == "joint_vel") {
      for (const auto index : joint_indices_) {
        frame.push_back(sensors.joint_vel[index]);
      }
    } else if (term.name == "base_ang_vel") {
      append_vector(frame, sensors.angular_velocity);
    } else if (term.name == "projected_gravity") {
      append_vector(frame, -robot.row(2).transpose());
    } else if (term.name == "last_action") {
      for (size_t j = 0; j < config_.defaults.size(); ++j) {
        frame.push_back(data_->policy.last_action.at(j));
      }
    } else if (term.name == "reference_orientation") {
      append_rotation(frame, (alignment_ * robot).transpose() * motion_->frame(cursor_).rotation);
    }
    term.apply(frame, term_begin, frame.size());
  }

  if (!std::all_of(frame.begin(), frame.end(), [](float value) {return std::isfinite(value);})) {
    throw std::runtime_error("Nonfinite BlinkMimic sensor/action history sample");
  }
  return frame;
}

std::vector<float> BlinkMimicObservation::compute()
{
  history_.discard_snapshot();

  if (!has_frame()) {throw std::runtime_error("BlinkMimic motion has ended");}
  history_.begin_sample();
  had_alignment_before_sample_ = has_alignment_;
  const auto frame = sensor_frame();

  history_.append(frame);

  std::vector<float> input;
  input.reserve(get_observation_size());
  config_.history.append_frames(input, history_.frames());
  append_goals(input);
  return input;
}

void BlinkMimicObservation::append_goals(std::vector<float> & input) const
{
  const auto & current = motion_->frame(cursor_);
  const float norm = std::sqrt(current.rotation(0, 0) * current.rotation(0, 0) +
      current.rotation(1, 0) * current.rotation(1, 0) + 1e-9f);
  const float c = current.rotation(0, 0) / norm;
  const float s = current.rotation(1, 0) / norm;
  Eigen::Matrix3f heading_transpose;
  heading_transpose << c, s, 0, -s, c, 0, 0, 0, 1;

  const auto & group = config_.goal_groups[0];
  std::vector<float> goals;
  goals.reserve(group.width() * config_.future_frames);
  for (size_t k = 0; k < config_.future_frames; ++k) {
    const auto & future = motion_->frame(std::min(
      cursor_ + config_.start_offset + k * config_.future_stride, motion_->size() - 1));
    for (const auto & term : group.terms()) {
      const size_t term_begin = goals.size();
      if (term.name == "future_root_displacement") {
        append_vector(goals, heading_transpose * (future.position - current.position));
      } else if (term.name == "future_root_orientation") {
        append_rotation(goals, heading_transpose * future.rotation);
      } else if (term.name == "future_root_linear_velocity") {
        append_vector(goals, future.rotation.transpose() * future.linear_velocity);
      } else if (term.name == "future_root_angular_velocity") {
        append_vector(goals, future.rotation.transpose() * future.angular_velocity);
      } else if (term.name == "future_joint_pos") {
        append_vector(goals, future.joint_position);
      } else if (term.name == "future_joint_vel") {
        append_vector(goals, future.joint_velocity);
      }
      term.apply(goals, term_begin, goals.size());
    }
  }
  group.append_frames(input, goals);
}

void BlinkMimicObservation::reject_latest()
{
  if (history_.reject_latest()) {
    // Recompute an initial alignment if its sensor sample was rejected.
    has_alignment_ = had_alignment_before_sample_;
  }
}

std::vector<ObservationSlice> BlinkMimicObservation::layout() const
{
  auto slices = config_.history.layout(0, config_.history_length);
  const auto goals = config_.goal_groups[0].layout(
    config_.history.width() * config_.history_length, config_.future_frames);
  slices.insert(slices.end(), goals.begin(), goals.end());
  return slices;
}

}  // namespace ai_sapiens_sim2real
