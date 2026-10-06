// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "ai_sapiens_sim2real/sensor_handles/depth_sensor_handle.hpp"
#include <algorithm>
#include <cmath>
namespace ai_sapiens_sim2real
{
DepthSensorHandle::DepthSensorHandle(
  rclcpp::Node::SharedPtr node, SensorData * sensors,
  const ai_sapiens_depth::DepthConfig & config)
: sensors_(sensors), config_(config)
{
  Sample initial;
  buffer_.initRT(initial);
  subscription_ = node->create_subscription<ai_sapiens_interfaces::msg::DepthHistory>(
    config.topic, rclcpp::SensorDataQoS(),
    [this](ai_sapiens_interfaces::msg::DepthHistory::ConstSharedPtr m) {receive(m);});
}
void DepthSensorHandle::receive(ai_sapiens_interfaces::msg::DepthHistory::ConstSharedPtr m)
{
  const double stamp = rclcpp::Time(m->header.stamp).seconds();
  if (stamp == last_stamp_) {return;}  // A replay never refreshes receive age.
  Sample sample;
  sample.valid = stamp > last_stamp_ && m->width == 32 && m->height == 18 && m->frames == 8 &&
    m->header.frame_id == config_.frame;
  last_stamp_ = stamp;
  for (size_t i = 0; i < m->frame_stamps.size(); ++i) {
    const double t = m->frame_stamps[i];
    sample.valid &= std::isfinite(t) && t >= 0 && t <= stamp + 1e-6 &&
      stamp - t <= 0.72 + config_.max_age && (i == 0 || t >= m->frame_stamps[i - 1]);
  }
  sample.valid &= stamp - m->frame_stamps.back() <= config_.period + config_.max_age;
  for (float v : m->data) {
    sample.valid &= std::isfinite(v) && v >= 0 && v <= 1;
  }
  sample.message = *m;
  sample.received = std::chrono::steady_clock::now();
  buffer_.writeFromNonRT(sample);
}
void DepthSensorHandle::start_episode()
{
  const auto * sample = buffer_.readFromRT();
  entry_stamp_ = sample->message.frame_stamps.back();
  std::copy(sample->message.data.end() - 576, sample->message.data.end(), entry_frame_.begin());
  for (int i = 0; i < 8; ++i) {
    std::copy(entry_frame_.begin(), entry_frame_.end(), sensors_->depth_history.begin() + i * 576);
  }
}
void DepthSensorHandle::update(const rclcpp::Time & time)
{
  const auto * sample = buffer_.readFromRT();
  const double age = time.seconds() - rclcpp::Time(sample->message.header.stamp).seconds();
  sensors_->depth_valid = sample->valid && age >= -0.02 && age <= config_.max_age &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() - sample->received).count() <=
    config_.max_age;
  if (sensors_->depth_valid) {
    sensors_->depth_history = sample->message.data;
    for (int i = 0; i < 8; ++i) {
      if (sample->message.frame_stamps[i] < entry_stamp_) {
        std::copy(entry_frame_.begin(), entry_frame_.end(),
            sensors_->depth_history.begin() + i * 576);
      }
    }
  }
}
}  // namespace ai_sapiens_sim2real
