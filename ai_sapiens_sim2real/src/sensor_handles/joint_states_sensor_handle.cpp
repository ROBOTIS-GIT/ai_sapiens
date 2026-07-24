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

#include <stdexcept>

namespace ai_sapiens_sim2real
{

JointStatesSensorHandle::JointStatesSensorHandle(
  rclcpp::Node::SharedPtr node,
  SensorData * sensors,
  ModeRequests * requests,
  double timeout_seconds,
  const std::vector<std::string> & joint_names,
  const std::string & topic)
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

  // Start the watchdog after startup.
  JointStatesData initial_data;
  initial_data.resize(num_joints_);
  initial_data.received_at = std::chrono::steady_clock::now();
  buffer_.initRT(initial_data);

  // Create subscription
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    topic,
    10,
    std::bind(&JointStatesSensorHandle::callback, this, std::placeholders::_1));

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
  JointStatesData data;
  std::vector<std::string> missing_joints;
  const size_t complete_joints_found =
    parse_joint_state_message(*msg, &data, &missing_joints);

  if (complete_joints_found != num_joints_) {
    log_incomplete_joint_state(complete_joints_found, missing_joints);
    return;
  }

  data.received_at = std::chrono::steady_clock::now();

  buffer_.writeFromNonRT(data);
  mark_ready_once();
}

size_t JointStatesSensorHandle::parse_joint_state_message(
  const sensor_msgs::msg::JointState & msg,
  JointStatesData * data,
  std::vector<std::string> * missing_joints) const
{
  data->resize(num_joints_);

  std::unordered_map<std::string, size_t> msg_name_to_idx;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    msg_name_to_idx[msg.name[i]] = i;
  }

  size_t complete_joints_found = 0;
  for (size_t model_idx = 0; model_idx < joint_names_.size(); ++model_idx) {
    const auto & joint_name = joint_names_[model_idx];

    const auto it = msg_name_to_idx.find(joint_name);
    if (it == msg_name_to_idx.end()) {
      missing_joints->push_back(joint_name);
      if (!received_once_.load()) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Joint '%s' not found in JointState message",
          joint_name.c_str());
      }
      continue;
    }

    const size_t msg_idx = it->second;
    const bool has_position_and_velocity =
      msg_idx < msg.position.size() && msg_idx < msg.velocity.size();
    if (!has_position_and_velocity) {
      missing_joints->push_back(joint_name + "(missing position/velocity)");
      if (!received_once_.load()) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Joint '%s' found but position or velocity is missing",
          joint_name.c_str());
      }
      continue;
    }

    data->position[model_idx] = static_cast<float>(msg.position[msg_idx]);
    data->velocity[model_idx] = static_cast<float>(msg.velocity[msg_idx]);
    complete_joints_found++;
  }

  return complete_joints_found;
}

void JointStatesSensorHandle::log_incomplete_joint_state(
  size_t complete_joints_found,
  const std::vector<std::string> & missing_joints) const
{
  const std::string missing_summary = build_missing_joint_summary(missing_joints);
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "Ignoring incomplete JointState message on %s: %zu/%zu joints complete. "
    "Missing/incomplete: %s",
    topic_.c_str(),
    complete_joints_found,
    num_joints_,
    missing_summary.c_str());
}

std::string JointStatesSensorHandle::build_missing_joint_summary(
  const std::vector<std::string> & missing_joints) const
{
  std::string missing_summary;
  constexpr size_t kMaxMissingJointsToLog = 8;
  const size_t count = std::min(kMaxMissingJointsToLog, missing_joints.size());
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) {
      missing_summary += ", ";
    }

    missing_summary += missing_joints[i];
  }
  if (missing_joints.size() > count) {
    missing_summary += ", ...";
  }

  return missing_summary;
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

void JointStatesSensorHandle::mark_ready_once()
{
  if (!received_once_.load()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[JointStatesSensorHandle] all %zu joints received",
      num_joints_);
  }

  received_once_.store(true);
}

}  // namespace ai_sapiens_sim2real
