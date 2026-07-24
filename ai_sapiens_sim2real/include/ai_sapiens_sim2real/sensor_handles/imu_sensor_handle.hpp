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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_SENSOR_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_SENSOR_HANDLE_HPP_

#include <atomic>
#include <chrono>
#include <string>

#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief IMU data structure for RealtimeBuffer.
 */
struct ImuData
{
  Eigen::Vector3f angular_velocity{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
  std::chrono::steady_clock::time_point received_at{};
};

/**
 * @brief Sensor handle for IMU data.
 *
 * Subscribes to sensor_msgs::msg::Imu topic and provides realtime-safe
 * access to angular velocity and orientation (for projected gravity).
 */
class ImuSensorHandle : public SensorHandleBase
{
public:
  /**
   * @brief Construct IMU sensor handle.
   * @param node ROS node for subscription
   * @param sensors Sensor data block to update
   * @param topic IMU topic name
   */
  ImuSensorHandle(
    rclcpp::Node::SharedPtr node,
    SensorData * sensors,
    ModeRequests * requests,
    double timeout_seconds,
    const std::string & topic = "/imu_sensor_broadcaster/imu");

  void update(const rclcpp::Time & /*time*/) override;

  std::string get_name() const override;

  // Readiness is startup-only; later stalls latch damping in update().
  bool is_ready() const override
  {
    return received_once_.load();
  }

private:
  void callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void log_stale_once(std::chrono::steady_clock::duration elapsed) const;

  rclcpp::Node::SharedPtr node_;
  SensorData * sensors_;
  ModeRequests * requests_;
  std::string topic_;

  // Subscription callback writes buffer_; update() copies it into SharedControlData.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<ImuData> buffer_;
  std::atomic<bool> received_once_{false};
  // Stale feedback latches damping until restart.
  std::chrono::duration<double> timeout_;
  bool stale_latched_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_SENSOR_HANDLE_HPP_
