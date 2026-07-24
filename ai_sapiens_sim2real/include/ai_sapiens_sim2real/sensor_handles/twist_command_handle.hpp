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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__TWIST_COMMAND_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__TWIST_COMMAND_HANDLE_HPP_

#include <string>

#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"
#include "ai_sapiens_sim2real/axis_range.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief Sensor handle for Twist velocity commands.
 *
 * Subscribes to geometry_msgs::msg::Twist topic and provides realtime-safe
 * access to velocity commands (lin_x, lin_y, ang_z).
 */
class TwistCommandHandle : public SensorHandleBase
{
public:
  /**
   * @brief Construct Twist command handle.
   * @param node ROS node for subscription
   * @param api API input block to update
   * @param topic Twist topic name
   */
  TwistCommandHandle(
    rclcpp::Node::SharedPtr node,
    ApiInput * api,
    const AxisRanges * active_velocity_command_ranges,
    const std::string & topic = "/cmd_vel");

  void update(const rclcpp::Time & /*time*/) override;

  std::string get_name() const override;

  bool is_ready() const override;

private:
  static float clamp_axis(double value, const AxisRange & range);

  void callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  ApiInput * api_;
  const AxisRanges * active_velocity_command_ranges_;

  // Subscription callback writes buffer_; update() selects it when API mode owns velocity.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<Eigen::Vector3f> buffer_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__TWIST_COMMAND_HANDLE_HPP_
