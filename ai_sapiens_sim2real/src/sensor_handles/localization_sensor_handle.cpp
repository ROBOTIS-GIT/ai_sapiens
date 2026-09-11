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

#include "ai_sapiens_sim2real/sensor_handles/localization_sensor_handle.hpp"

#include <cmath>
#include <stdexcept>

namespace ai_sapiens_sim2real
{

LocalizationSensorHandle::LocalizationSensorHandle(
  rclcpp::Node::SharedPtr node, LocalizationData * localization,
  double timeout_seconds, const std::string & topic,
  const std::string & world_frame, const std::string & base_frame)
: node_(node), localization_(localization), timeout_seconds_(timeout_seconds),
  world_frame_(world_frame), base_frame_(base_frame)
{
  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0 ||
    topic.empty() || world_frame.empty() || base_frame.empty())
  {
    throw std::runtime_error(
        "Localization requires a positive finite timeout and topic/frame names");
  }
  buffer_.initRT(LocalizationSample{});
  subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {callback(msg);});
  RCLCPP_INFO(node_->get_logger(), "[LocalizationSensorHandle] %s (%s -> %s)",
    topic.c_str(), world_frame.c_str(), base_frame.c_str());
}

void LocalizationSensorHandle::callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  LocalizationSample sample;
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  sample.pose.position = Eigen::Vector2f(p.x, p.y);
  sample.pose.orientation = Eigen::Quaternionf(q.w, q.x, q.y, q.z);
  sample.stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    msg->header.stamp.nanosec;
  sample.received_at = std::chrono::steady_clock::now();
  const double source_age = (node_->now().nanoseconds() - sample.stamp_ns) * 1.0e-9;
  const float norm = sample.pose.orientation.norm();
  const char * reason = nullptr;
  if (msg->header.frame_id != world_frame_ || msg->child_frame_id != base_frame_) {
    reason = "frame mismatch";
  } else if (!sample.pose.position.allFinite() || !std::isfinite(p.z) ||
    !sample.pose.orientation.coeffs().allFinite() || !std::isfinite(norm) || norm <= 1.0e-6f)
  {
    reason = "invalid pose";
  } else if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U ||
    sample.stamp_ns <= 0)
  {
    reason = "invalid timestamp";
  } else if (sample.stamp_ns == last_stamp_ns_) {
    // The estimator can republish its current state after a contact event.
    // Keep the last valid sample, including its original reception time, so
    // duplicates neither invalidate localization nor extend the watchdog.
    return;
  } else if (sample.stamp_ns < last_stamp_ns_) {
    reason = "timestamp moved backwards";
  } else if (source_age < -0.05 || source_age > timeout_seconds_) {
    reason = "timestamp outside freshness window";
  }
  sample.pose.valid = reason == nullptr;
  if (sample.pose.valid) {
    sample.pose.orientation.normalize();
    last_stamp_ns_ = sample.stamp_ns;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Invalid localization: %s; frame=%s -> %s expected=%s -> %s "
      "age=%.6fs timeout=%.3fs stamp=%lld last=%lld quaternion_norm=%.6f",
      reason, msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
      world_frame_.c_str(), base_frame_.c_str(), source_age, timeout_seconds_,
      static_cast<long long>(sample.stamp_ns), static_cast<long long>(last_stamp_ns_), norm);
  }
  // Invalid messages invalidate the input instead of refreshing the last valid pose.
  buffer_.writeFromNonRT(sample);
}

void LocalizationSensorHandle::update(const rclcpp::Time & time)
{
  const auto & sample = *buffer_.readFromRT();
  const double received_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - sample.received_at).count();
  const double source_age = (time.nanoseconds() - sample.stamp_ns) * 1.0e-9;
  localization_->valid = sample.pose.valid && received_age <= timeout_seconds_ &&
    source_age >= -0.05 && source_age <= timeout_seconds_;
  if (localization_->valid) {
    localization_->position = sample.pose.position;
    localization_->orientation = sample.pose.orientation;
  }
}

}  // namespace ai_sapiens_sim2real
