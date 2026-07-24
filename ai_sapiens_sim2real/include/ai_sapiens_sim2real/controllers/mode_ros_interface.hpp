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

#ifndef AI_SAPIENS_SIM2REAL__CONTROLLERS__MODE_ROS_INTERFACE_HPP_
#define AI_SAPIENS_SIM2REAL__CONTROLLERS__MODE_ROS_INTERFACE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_interfaces/msg/mode_status.hpp"
#include "ai_sapiens_interfaces/srv/list_modes.hpp"
#include "ai_sapiens_interfaces/srv/set_mode_by_name.hpp"
#include "ai_sapiens_sim2real/controllers/mode_controller.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief The mode subsystem's ROS surface: request/list services in, status out.
 *
 * Keeps ROS I/O out of the RT ModeController. Service callbacks route a mode
 * request through ModeController (which validates and enqueues it on its
 * lock-free gate); the timer publishes the controller's status snapshot.
 */
class ModeRosInterface
{
public:
  ModeRosInterface(
    rclcpp::Node::SharedPtr node,
    ModeController & mode_controller,
    const std::string & set_mode_service_name,
    const std::string & list_service_name,
    const std::string & mode_status_topic,
    double mode_status_publish_period);

private:
  void handle_set_mode(
    const ai_sapiens_interfaces::srv::SetModeByName::Request & request,
    ai_sapiens_interfaces::srv::SetModeByName::Response & response);
  void fill_mode_list(ai_sapiens_interfaces::srv::ListModes::Response & response) const;
  void publish_status();

  rclcpp::Node::SharedPtr node_;
  ModeController * mode_controller_;

  rclcpp::Service<ai_sapiens_interfaces::srv::SetModeByName>::SharedPtr
    set_mode_service_;
  rclcpp::Service<ai_sapiens_interfaces::srv::ListModes>::SharedPtr list_service_;
  rclcpp::Publisher<ai_sapiens_interfaces::msg::ModeStatus>::SharedPtr mode_status_pub_;
  rclcpp::TimerBase::SharedPtr mode_status_timer_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONTROLLERS__MODE_ROS_INTERFACE_HPP_
