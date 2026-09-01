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

#include "ai_sapiens_sim2real/sensor_handles/teleop_input_handle.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ai_sapiens_sim2real
{

namespace
{

// Zero-preserving declared-range normalization.
float normalize_axis(double output, const AxisRange & declared, bool & out_of_range)
{
  constexpr double kTolerance = 1e-3;
  const bool is_above_declared_max = output > declared.max + kTolerance;
  const bool is_below_declared_min = output < declared.min - kTolerance;
  if (is_above_declared_max || is_below_declared_min) {
    out_of_range = true;
    return 0.0f;
  }

  return normalize_axis_to_unit(output, declared);
}

}  // namespace

TeleopInputHandle::TeleopInputHandle(
  rclcpp::Node::SharedPtr node,
  TeleopInput * teleop,
  ModeRequests * requests,
  const AxisRanges * active_velocity_command_ranges,
  std::shared_ptr<TeleopInputPluginBase> plugin,
  double timeout_seconds,
  double vel_command_timeout_seconds)
: node_(std::move(node)),
  teleop_(teleop),
  requests_(requests),
  active_velocity_command_ranges_(active_velocity_command_ranges),
  plugin_(std::move(plugin)),
  plugin_output_ranges_(plugin_->output_axis_ranges()),
  timeout_(timeout_seconds),
  vel_command_timeout_(vel_command_timeout_seconds)
{
  if (timeout_seconds <= 0.0) {
    throw std::runtime_error("Teleop input watchdog timeout must be positive");
  }
  if (vel_command_timeout_seconds <= 0.0) {
    throw std::runtime_error("Teleop velocity command timeout must be positive");
  }
  if (vel_command_timeout_seconds > timeout_seconds) {
    throw std::runtime_error(
            "Teleop velocity command timeout must not exceed the input watchdog timeout");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[TeleopInputHandle] plugin(%s) vel_command_timeout(%.3fs) timeout(%.3fs)",
    plugin_->name().c_str(),
    vel_command_timeout_seconds,
    timeout_seconds);
}

Eigen::Vector3f TeleopInputHandle::normalize_plugin_output(
  const Eigen::Vector3f & plugin_output, bool & out_of_range) const
{
  Eigen::Vector3f normalized;
  normalized.x() = normalize_axis(plugin_output.x(), plugin_output_ranges_.linear_x, out_of_range);
  normalized.y() = normalize_axis(plugin_output.y(), plugin_output_ranges_.linear_y, out_of_range);
  normalized.z() =
    normalize_axis(plugin_output.z(), plugin_output_ranges_.angular_z, out_of_range);
  return normalized;
}

void TeleopInputHandle::update(const rclcpp::Time & /*time*/)
{
  TeleopInputCommand command;
  const bool has_accepted_command = plugin_->read_latest_accepted_command(command);
  const auto command_age = std::chrono::steady_clock::now() - command.received_at;
  const bool is_command_timed_out = has_accepted_command && command_age > timeout_;
  const bool is_velocity_command_timed_out =
    has_accepted_command && command_age > vel_command_timeout_;
  const bool is_input_unavailable = !has_accepted_command || is_command_timed_out;

  copy_command_state(command, has_accepted_command, is_input_unavailable);
  if (is_input_unavailable) {
    log_unavailable_input_once(command, has_accepted_command);
    apply_unavailable_teleop_input();
    return;
  }

  unavailable_logged_ = false;

  if (is_velocity_command_timed_out) {
    teleop_->velocity_commands.setZero();
    teleop_->velocity_command_normalized.setZero();
    return;
  }

  bool out_of_range = false;
  const Eigen::Vector3f normalized = normalize_plugin_output(command.velocity, out_of_range);
  if (out_of_range) {
    teleop_->velocity_commands.setZero();
    teleop_->velocity_command_normalized.setZero();
    log_out_of_range_command(command.velocity);
    return;
  }

  teleop_->velocity_command_normalized = normalized;
  const auto & active = *active_velocity_command_ranges_;
  teleop_->velocity_commands.x() = scale_unit_to_axis(normalized.x(), active.linear_x);
  teleop_->velocity_commands.y() = scale_unit_to_axis(normalized.y(), active.linear_y);
  teleop_->velocity_commands.z() = scale_unit_to_axis(normalized.z(), active.angular_z);
}

void TeleopInputHandle::copy_command_state(
  const TeleopInputCommand & command,
  bool has_accepted_command,
  bool input_unavailable)
{
  teleop_->received = has_accepted_command;
  teleop_->unavailable = input_unavailable;
  teleop_->api_mode_requested = command.api_mode;
  teleop_->input_code = command.input_code;
  teleop_->selector_code = command.selector_code;
  teleop_->update_time = command.received_at;
}

void TeleopInputHandle::apply_unavailable_teleop_input()
{
  teleop_->velocity_commands.setZero();
  teleop_->velocity_command_normalized.setZero();
  requests_->damping = true;
}

void TeleopInputHandle::log_unavailable_input_once(
  const TeleopInputCommand & command,
  bool has_accepted_command)
{
  if (unavailable_logged_) {
    return;
  }

  if (has_accepted_command) {
    const auto elapsed = std::chrono::steady_clock::now() - command.received_at;
    RCLCPP_WARN(
      node_->get_logger(),
      "Teleop input unavailable: no accepted input for %.3fs; exceeded %.3fs timeout "
      "(plugin=%s)",
      std::chrono::duration<double>(elapsed).count(),
      timeout_.count(),
      plugin_->name().c_str());
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Teleop input unavailable: no accepted input sample yet (plugin=%s, timeout=%.3fs)",
      plugin_->name().c_str(),
      timeout_.count());
  }

  unavailable_logged_ = true;
}

void TeleopInputHandle::log_out_of_range_command(const Eigen::Vector3f & command) const
{
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "Teleop input out of declared range: command=(%.3f, %.3f, %.3f), "
    "declared_ranges=(linear_x=[%.3f, %.3f], linear_y=[%.3f, %.3f], "
    "angular_z=[%.3f, %.3f]); commanding zero (plugin=%s)",
    command.x(),
    command.y(),
    command.z(),
    plugin_output_ranges_.linear_x.min,
    plugin_output_ranges_.linear_x.max,
    plugin_output_ranges_.linear_y.min,
    plugin_output_ranges_.linear_y.max,
    plugin_output_ranges_.angular_z.min,
    plugin_output_ranges_.angular_z.max,
    plugin_->name().c_str());
}

std::string TeleopInputHandle::get_name() const
{
  return "teleop_input";
}

bool TeleopInputHandle::is_ready() const
{
  return plugin_->is_ready();
}

}  // namespace ai_sapiens_sim2real
