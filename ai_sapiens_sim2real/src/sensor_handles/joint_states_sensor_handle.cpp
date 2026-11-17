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

#include "ai_sapiens_sim2real/sensor_handles/joint_states_sensor_handle.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace ai_sapiens_sim2real
{

JointStatesSensorHandle::JointStatesSensorHandle(
  rclcpp::Node::SharedPtr node,
  SensorData * sensors,
  ModeRequests * requests,
  double timeout_seconds,
  const std::vector<std::string> & joint_names,
  const std::string & topic,
  const rclcpp::SubscriptionOptions & subscription_options,
  double expected_rate_hz)
: node_(node)
  , sensors_(sensors)
  , requests_(requests)
  , topic_(topic)
  , joint_names_(joint_names)
  , num_joints_(joint_names.size())
  , timeout_(std::chrono::duration<double>(timeout_seconds))
{
  if (joint_names_.empty()) {
    throw std::runtime_error("JointStatesSensorHandle requires joint names");
  }
  if (timeout_seconds <= 0.0) {
    throw std::runtime_error("Joint states sensor watchdog timeout must be positive");
  }

  callback_data_.resize(num_joints_);
  message_indices_.assign(num_joints_, std::numeric_limits<size_t>::max());

  // Start the watchdog after startup.
  JointStatesData initial_data;
  initial_data.resize(num_joints_);
  initial_data.received_at = std::chrono::steady_clock::now();
  buffer_.initRT(initial_data);

  input_gap_monitor_ = std::make_unique<InputGapMonitor>(
    node_, topic_, expected_rate_hz,
    subscription_options.callback_group);

  diagnostics_ = std::make_unique<JointStatesDiagnostics>(node_, topic_, joint_names_);

  // Create subscription
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    topic,
    10,
    std::bind(&JointStatesSensorHandle::callback, this, std::placeholders::_1),
      subscription_options);

  RCLCPP_INFO(
    node_->get_logger(),
    "[JointStatesSensorHandle] subscribed to %s (%zu joints)",
    topic.c_str(),
    num_joints_);

  // Log expected joint names
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    RCLCPP_INFO(node_->get_logger(), "  [%zu] %s", i, joint_names_[i].c_str());
  }
}

void JointStatesSensorHandle::update(const rclcpp::Time & /*time*/)
{
  // Read from RealtimeBuffer (non-blocking)
  JointStatesData * data = buffer_.readFromRT();

  // Copy to the shared sensor block.
  sensors_->joint_pos = data->position;
  sensors_->joint_vel = data->velocity;

  const auto elapsed = std::chrono::steady_clock::now() - data->received_at;
  if (elapsed > timeout_) {
    log_stale_once(elapsed);
    stale_latched_ = true;
  }
  if (stale_latched_) {
    requests_->damping = true;
  }
}

std::string JointStatesSensorHandle::get_name() const
{
  return "joint_states";
}

void JointStatesSensorHandle::callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  input_gap_monitor_->received_raw();
  update_joint_mapping(msg->name);
  const size_t complete_joints_found = parse_joint_state_message(*msg);

  if (complete_joints_found != num_joints_) {
    diagnostics_->record_incomplete(*msg, message_indices_, complete_joints_found);
    return;
  }

  callback_data_.received_at = std::chrono::steady_clock::now();
  buffer_.writeFromNonRT(callback_data_);
  input_gap_monitor_->received_accepted();
  received_once_.store(true);
  diagnostics_->record_ready();
}

void JointStatesSensorHandle::update_joint_mapping(const std::vector<std::string> & names)
{
  if (names == cached_message_names_) {
    return;
  }

  cached_message_names_ = names;
  std::unordered_map<std::string, size_t> name_to_index;
  name_to_index.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    name_to_index[names[i]] = i;  // Preserve the existing last-occurrence rule for duplicates.
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    const auto found = name_to_index.find(joint_names_[i]);
    message_indices_[i] = found == name_to_index.end() ?
      std::numeric_limits<size_t>::max() : found->second;
  }
}

size_t JointStatesSensorHandle::parse_joint_state_message(const sensor_msgs::msg::JointState & msg)
{
  size_t complete_joints_found = 0;
  for (size_t i = 0; i < num_joints_; ++i) {
    const auto index = message_indices_[i];
    if (index >= msg.position.size() || index >= msg.velocity.size()) {
      continue;
    }

    callback_data_.position[i] = static_cast<float>(msg.position[index]);
    callback_data_.velocity[i] = static_cast<float>(msg.velocity[index]);
    ++complete_joints_found;
  }

  return complete_joints_found;
}

void JointStatesSensorHandle::log_stale_once(
  std::chrono::steady_clock::duration elapsed) const
{
  if (stale_latched_) {
    return;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "Damping failsafe latched: no joint states message for %.3fs; exceeded %.3fs timeout "
    "(topic=%s)",
    std::chrono::duration<double>(elapsed).count(),
    timeout_.count(),
    topic_.c_str());
}

}  // namespace ai_sapiens_sim2real
