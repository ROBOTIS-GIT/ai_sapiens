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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__JOINT_STATES_SENSOR_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__JOINT_STATES_SENSOR_HANDLE_HPP_

#include <atomic>
#include <chrono>
#include <string>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief Joint states data structure for RealtimeBuffer.
 */
struct JointStatesData
{
  Eigen::VectorXf position;
  Eigen::VectorXf velocity;
  std::chrono::steady_clock::time_point received_at{};

  void resize(size_t num_joints)
  {
    position.resize(num_joints);
    velocity.resize(num_joints);
    position.setZero();
    velocity.setZero();
  }
};

/**
 * @brief Sensor handle for joint states.
 *
 * Subscribes to sensor_msgs::msg::JointState topic and provides realtime-safe
 * access to joint positions and velocities. Uses joint name mapping to handle
 * arbitrary joint ordering in the message.
 */
class JointStatesSensorHandle : public SensorHandleBase
{
public:
  /**
   * @brief Construct joint states sensor handle.
   * @param node ROS node for subscription
   * @param sensors Sensor data block to update
   * @param joint_names Ordered list of joint names (defines model order)
   * @param topic JointState topic name
   */
  JointStatesSensorHandle(
    rclcpp::Node::SharedPtr node,
    SensorData * sensors,
    ModeRequests * requests,
    double timeout_seconds,
    const std::vector<std::string> & joint_names,
    const std::string & topic = "/joint_states");

  void update(const rclcpp::Time & /*time*/) override;

  std::string get_name() const override;

  // Readiness is startup-only; later stalls latch damping in update().
  bool is_ready() const override
  {
    return received_once_.load();
  }

  /**
   * @brief Get number of joints being tracked.
   * @return Number of joints
   */
  size_t get_num_joints() const
  {
    return num_joints_;
  }

private:
  // ROS callback parses JointState into model joint order and writes the RT buffer.
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  size_t parse_joint_state_message(
    const sensor_msgs::msg::JointState & msg,
    JointStatesData * data,
    std::vector<std::string> * missing_joints) const;
  void log_incomplete_joint_state(
    size_t complete_joints_found,
    const std::vector<std::string> & missing_joints) const;
  std::string build_missing_joint_summary(const std::vector<std::string> & missing_joints) const;
  void log_stale_once(std::chrono::steady_clock::duration elapsed) const;
  void mark_ready_once();

  rclcpp::Node::SharedPtr node_;
  SensorData * sensors_;
  ModeRequests * requests_;
  std::string topic_;

  std::vector<std::string> joint_names_;
  size_t num_joints_;
  // Subscription callback writes buffer_; update() copies it into SharedControlData.
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<JointStatesData> buffer_;
  std::atomic<bool> received_once_{false};
  // Stale feedback latches damping until restart.
  std::chrono::duration<double> timeout_;
  bool stale_latched_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__JOINT_STATES_SENSOR_HANDLE_HPP_
