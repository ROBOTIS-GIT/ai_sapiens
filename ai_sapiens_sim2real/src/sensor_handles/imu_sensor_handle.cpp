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

#include "ai_sapiens_sim2real/sensor_handles/imu_sensor_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/imu_validation.hpp"

#include <stdexcept>

namespace ai_sapiens_sim2real
{

ImuSensorHandle::ImuSensorHandle(
  rclcpp::Node::SharedPtr node,
  SensorData * sensors,
  ModeRequests * requests,
  double timeout_seconds,
  const std::string & topic)
: node_(node)
  , sensors_(sensors)
  , requests_(requests)
  , topic_(topic)
  , timeout_(std::chrono::duration<double>(timeout_seconds))
{
  if (timeout_seconds <= 0.0) {
    throw std::runtime_error("IMU sensor watchdog timeout must be positive");
  }

  // Start the watchdog after startup.
  ImuData initial_data;
  initial_data.received_at = std::chrono::steady_clock::now();
  buffer_.initRT(initial_data);

  // Create subscription
  subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    topic,
    10,
    std::bind(&ImuSensorHandle::callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "[ImuSensorHandle] subscribed to %s", topic.c_str());
}

void ImuSensorHandle::update(const rclcpp::Time & /*time*/)
{
  // Read from RealtimeBuffer (non-blocking)
  ImuData * data = buffer_.readFromRT();

  // Never publish a fabricated identity or non-finite data as a measurement.
  // Last good numeric values remain available, but their validity is explicit.
  if (data->valid) {
    sensors_->angular_velocity = data->angular_velocity;
    sensors_->orientation = data->orientation;
    sensors_->compute_projected_gravity();
  }

  const auto elapsed = std::chrono::steady_clock::now() - data->received_at;
  if (elapsed > timeout_) {
    log_stale_once(elapsed);
    stale_latched_ = true;
  }
  const bool invalid = invalid_latched_.load();
  sensors_->imu_usable = data->valid && !stale_latched_ && !invalid;
  if (invalid && !invalid_logged_) {
    RCLCPP_ERROR(node_->get_logger(),
      "Damping failsafe latched: invalid IMU orientation/angular velocity (topic=%s); "
      "restart required after sensor recovery", topic_.c_str());
    invalid_logged_ = true;
  }
  if (stale_latched_ || invalid) {
    requests_->damping = true;
  }
}

std::string ImuSensorHandle::get_name() const
{
  return "imu_sensor";
}

void ImuSensorHandle::callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  ImuData data;

  // Extract angular velocity
  data.angular_velocity.x() = static_cast<float>(msg->angular_velocity.x);
  data.angular_velocity.y() = static_cast<float>(msg->angular_velocity.y);
  data.angular_velocity.z() = static_cast<float>(msg->angular_velocity.z);

  // Extract orientation (ROS: x,y,z,w -> Eigen: w,x,y,z)
  data.orientation = Eigen::Quaternionf(
          static_cast<float>(msg->orientation.w),
          static_cast<float>(msg->orientation.x),
          static_cast<float>(msg->orientation.y),
          static_cast<float>(msg->orientation.z)
  );

  data.valid = valid_imu_sample(
    data.orientation, data.angular_velocity, msg->orientation_covariance[0] != -1.0,
    msg->angular_velocity_covariance[0] != -1.0);
  if (data.valid) {
    data.orientation.normalize();
  } else if (received_once_.load()) {
    // Before the first valid sample, remain in startup wait. Afterwards, use
    // the same restart-required behavior as the existing timeout watchdog.
    invalid_latched_.store(true);
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Waiting for valid IMU orientation/angular velocity (topic=%s)", topic_.c_str());
  }

  data.received_at = std::chrono::steady_clock::now();

  // Write to buffer (thread-safe)
  buffer_.writeFromNonRT(data);
  if (data.valid) {
    received_once_.store(true);
  }
}

void ImuSensorHandle::log_stale_once(std::chrono::steady_clock::duration elapsed) const
{
  if (stale_latched_) {
    return;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "Damping failsafe latched: no IMU message for %.3fs; exceeded %.3fs timeout (topic=%s)",
    std::chrono::duration<double>(elapsed).count(),
    timeout_.count(),
    topic_.c_str());
}


}  // namespace ai_sapiens_sim2real
