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

#include "ai_sapiens_sim2real/command_publishers/joint_command_publisher.hpp"

#include <algorithm>
#include <stdexcept>

#include <rclcpp/exceptions/exceptions.hpp>

namespace ai_sapiens_sim2real
{

JointCommandPublisher::JointCommandPublisher(
  rclcpp::Node::SharedPtr node,
  BehaviorOutput * output,
  const std::string & topic,
  const std::vector<std::string> & controller_joint_names,
  bool enforce_position_limits,
  bool enabled)
: node_(node)
  , output_(output)
  , topic_(topic)
  , controller_joint_names_(controller_joint_names)
  , enforce_position_limits_(enforce_position_limits)
  , enabled_(enabled)
{
  if (controller_joint_names_.empty()) {
    throw std::runtime_error("JointCommandPublisher requires controller_joint_names");
  }

  // publish() indexes these by controller joint, so sizes must match exactly.
  const size_t joint_count = controller_joint_names_.size();
  const bool has_matching_action_output_sizes =
    output_->command.position.size() == joint_count &&
    output_->feedforward.size() == joint_count &&
    output_->command.stiffness.size() == joint_count &&
    output_->command.damping.size() == joint_count;
  if (!has_matching_action_output_sizes) {
    throw std::runtime_error(
      "processed_action, feedforward, stiffness, and damping sizes must match "
      "controller_joint_names");
  }

  msg_.positions.resize(joint_count);
  msg_.feedforward.resize(joint_count);
  msg_.kp.resize(joint_count);
  msg_.kd.resize(joint_count);

  if (enabled_) {
    // Create publisher
    publisher_ =
      node_->create_publisher<ai_sapiens_interfaces::msg::JointImpedanceCommand>(
        topic_, rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(
      node_->get_logger(),
      "[JointCommandPublisher] publishing to %s (%zu joints)",
      topic_.c_str(),
      controller_joint_names_.size());

    RCLCPP_INFO(
      node_->get_logger(),
      "[JointCommandPublisher] position limits %s",
      enforce_position_limits_ ? "enabled" : "disabled");
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "[JointCommandPublisher] disabled (would publish to %s)",
      topic_.c_str());
  }
}

void JointCommandPublisher::publish(const rclcpp::Time & time, CommandPublishMode mode)
{
  if (!enabled_ || !publisher_) {
    return;
  }

  msg_.header.stamp = time;

  for (size_t controller_index = 0; controller_index < controller_joint_names_.size();
    ++controller_index)
  {
    float val = output_->command.position[controller_index];

    const bool has_position_limit =
      controller_index < output_->position_limits.size() &&
      output_->position_limits[controller_index];
    if (enforce_position_limits_ && has_position_limit) {
      const auto & limit = *output_->position_limits[controller_index];
      val = std::clamp(val, limit.first, limit.second);
    }

    msg_.positions[controller_index] = static_cast<double>(val);
    msg_.feedforward[controller_index] =
      static_cast<double>(output_->feedforward[controller_index]);
    msg_.kp[controller_index] = static_cast<double>(output_->command.stiffness[controller_index]);
    msg_.kd[controller_index] = static_cast<double>(output_->command.damping[controller_index]);
  }

  try {
    publisher_->publish(msg_);
  } catch (const rclcpp::exceptions::RCLErrorBase & error) {
    if (mode == CommandPublishMode::EmergencyDamping) {
      RCLCPP_ERROR(
        node_->get_logger(), "Emergency Damping publish failed (topic=%s, rcl=%d): %s",
        topic_.c_str(), error.ret, error.formatted_message.c_str());
    } else {
      record_publish_failure(error.message.c_str(), error.ret);
    }
    return;
  }

  consecutive_failures_ = 0;
  auto & published = output_->last_published;
  published.position.assign(msg_.positions.begin(), msg_.positions.end());
  published.stiffness.assign(msg_.kp.begin(), msg_.kp.end());
  published.damping.assign(msg_.kd.begin(), msg_.kd.end());
  output_->has_published_command = true;
}

void JointCommandPublisher::record_publish_failure(const char * message, int error_code)
{
  constexpr unsigned int kMaxToleratedFailures = 10;
  if (!output_->is_command_publish_fault_latched) {
    ++consecutive_failures_;
    if (consecutive_failures_ > kMaxToleratedFailures) {
      output_->is_command_publish_fault_latched = true;
      RCLCPP_ERROR(
        node_->get_logger(),
        "Damping latched after %u consecutive joint command publish failures (topic=%s, rcl=%d): %s",
        consecutive_failures_, topic_.c_str(), error_code, message);
      return;
    }
  }

  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Joint command publish failed (topic=%s, rcl=%d): %s", topic_.c_str(), error_code, message);
}

std::string JointCommandPublisher::get_name() const
{
  return "joint_command_publisher";
}

void JointCommandPublisher::set_enabled(bool enabled)
{
  const bool should_create_publisher = enabled && !publisher_;
  if (should_create_publisher) {
    publisher_ =
      node_->create_publisher<ai_sapiens_interfaces::msg::JointImpedanceCommand>(
        topic_, rclcpp::SystemDefaultsQoS());
  }

  enabled_ = enabled;
}


}  // namespace ai_sapiens_sim2real
