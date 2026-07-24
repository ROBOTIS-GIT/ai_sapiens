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

#include "ai_sapiens_sim2real/sensor_handles/api_heartbeat_handle.hpp"

#include <stdexcept>

namespace ai_sapiens_sim2real
{

ApiHeartbeatHandle::ApiHeartbeatHandle(
  rclcpp::Node::SharedPtr node,
  ApiInput * api,
  const std::string & topic,
  double timeout_seconds)
: node_(node),
  api_(api),
  topic_(topic),
  timeout_(std::chrono::duration<double>(timeout_seconds))
{
  if (timeout_seconds <= 0.0) {
    throw std::runtime_error("API heartbeat watchdog timeout must be positive");
  }

  ApiHeartbeatSnapshot initial;
  initial.received_at = std::chrono::steady_clock::now();
  buffer_.initRT(initial);
  subscription_ = node_->create_subscription<ai_sapiens_interfaces::msg::ApiHeartbeat>(
    topic_,
    10,
    [this](const ai_sapiens_interfaces::msg::ApiHeartbeat::SharedPtr msg) {
      ApiHeartbeatSnapshot snapshot;
      snapshot.sequence = msg->sequence;
      snapshot.received_at = std::chrono::steady_clock::now();
      buffer_.writeFromNonRT(snapshot);
      received_once_.store(true, std::memory_order_release);
    });
  RCLCPP_INFO(
    node_->get_logger(),
    "[ApiHeartbeatHandle] topic(%s) timeout(%.3fs)",
    topic_.c_str(),
    timeout_seconds);
}

void ApiHeartbeatHandle::update(const rclcpp::Time & /*time*/)
{
  ApiHeartbeatSnapshot * snapshot = buffer_.readFromRT();
  const auto now = std::chrono::steady_clock::now();
  const bool has_received_heartbeat = received_once_.load(std::memory_order_acquire);

  const bool is_new_sequence = !last_sequence_seen_ || *last_sequence_seen_ != snapshot->sequence;
  if (has_received_heartbeat && is_new_sequence) {
    last_sequence_seen_ = snapshot->sequence;
    last_sequence_advance_time_ = now;
  }

  const bool is_heartbeat_timed_out = now - last_sequence_advance_time_ > timeout_;
  api_->heartbeat_received = has_received_heartbeat;
  api_->heartbeat_stale = !has_received_heartbeat || is_heartbeat_timed_out;
  if (api_->heartbeat_stale) {
    log_stale_heartbeat_once(*snapshot, now);
  } else {
    heartbeat_stale_logged_ = false;
  }

  api_->heartbeat_sequence = snapshot->sequence;
  api_->heartbeat_update_time = snapshot->received_at;
}

void ApiHeartbeatHandle::log_stale_heartbeat_once(
  const ApiHeartbeatSnapshot & snapshot,
  std::chrono::steady_clock::time_point now)
{
  const bool has_received_heartbeat = received_once_.load(std::memory_order_acquire);
  if (!has_received_heartbeat || heartbeat_stale_logged_) {
    return;
  }

  const auto elapsed = now - last_sequence_advance_time_;
  RCLCPP_WARN(
    node_->get_logger(),
    "API heartbeat stale: no advancing heartbeat for %.3fs; exceeded %.3fs timeout "
    "(topic=%s, last_sequence=%u)",
    std::chrono::duration<double>(elapsed).count(),
    timeout_.count(),
    topic_.c_str(),
    snapshot.sequence);
  heartbeat_stale_logged_ = true;
}

std::string ApiHeartbeatHandle::get_name() const
{
  return "api_heartbeat";
}


}  // namespace ai_sapiens_sim2real
