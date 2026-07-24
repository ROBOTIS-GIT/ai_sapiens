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
// Author: Woojin Wie

#include "ai_sapiens_sim2real/sensor_handles/twist_command_handle.hpp"

#include <algorithm>

namespace ai_sapiens_sim2real
{

TwistCommandHandle::TwistCommandHandle(
  rclcpp::Node::SharedPtr node,
  ApiInput * api,
  const AxisRanges * active_velocity_command_ranges,
  const std::string & topic)
: node_(node)
  , api_(api)
  , active_velocity_command_ranges_(active_velocity_command_ranges)
{
  // Initialize buffer with zero velocities
  Eigen::Vector3f initial_cmd = Eigen::Vector3f::Zero();
  buffer_.initRT(initial_cmd);

  // Create subscription
  subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    topic,
    10,
    std::bind(&TwistCommandHandle::callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "[TwistCommandHandle] subscribed to %s", topic.c_str());
}

void TwistCommandHandle::update(const rclcpp::Time & /*time*/)
{
  // Clamp on the RT thread, where the active range is written; the callback only buffers raw.
  const Eigen::Vector3f & raw = *buffer_.readFromRT();
  const auto & active = *active_velocity_command_ranges_;
  api_->velocity_command_raw = raw;
  api_->velocity_commands.x() = clamp_axis(raw.x(), active.linear_x);
  api_->velocity_commands.y() = clamp_axis(raw.y(), active.linear_y);
  api_->velocity_commands.z() = clamp_axis(raw.z(), active.angular_z);
}

std::string TwistCommandHandle::get_name() const
{
  return "twist_command";
}

bool TwistCommandHandle::is_ready() const
{
  // Twist command is optional, always ready (defaults to zero)
  return true;
}

float TwistCommandHandle::clamp_axis(double value, const AxisRange & range)
{
  return static_cast<float>(std::clamp(value, range.min, range.max));
}

void TwistCommandHandle::callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Buffer the raw command; update() clamps it to the active range.
  Eigen::Vector3f cmd;
  cmd.x() = static_cast<float>(msg->linear.x);
  cmd.y() = static_cast<float>(msg->linear.y);
  cmd.z() = static_cast<float>(msg->angular.z);

  // Write to buffer (thread-safe)
  buffer_.writeFromNonRT(cmd);
}


}  // namespace ai_sapiens_sim2real
