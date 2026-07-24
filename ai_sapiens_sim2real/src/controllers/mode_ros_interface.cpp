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

#include "ai_sapiens_sim2real/controllers/mode_ros_interface.hpp"

#include <chrono>
#include <memory>

namespace ai_sapiens_sim2real
{

ModeRosInterface::ModeRosInterface(
  rclcpp::Node::SharedPtr node,
  ModeController & mode_controller,
  const std::string & set_mode_service_name,
  const std::string & list_service_name,
  const std::string & mode_status_topic,
  double mode_status_publish_period)
: node_(std::move(node)),
  mode_controller_(&mode_controller)
{
  set_mode_service_ =
    node_->create_service<ai_sapiens_interfaces::srv::SetModeByName>(
    set_mode_service_name,
    [this](
      const std::shared_ptr<ai_sapiens_interfaces::srv::SetModeByName::Request> request,
      std::shared_ptr<ai_sapiens_interfaces::srv::SetModeByName::Response> response)
    {
      handle_set_mode(*request, *response);
    });

  list_service_ = node_->create_service<ai_sapiens_interfaces::srv::ListModes>(
    list_service_name,
    [this](
      const std::shared_ptr<ai_sapiens_interfaces::srv::ListModes::Request>/*request*/,
      std::shared_ptr<ai_sapiens_interfaces::srv::ListModes::Response> response)
    {
      fill_mode_list(*response);
    });

  mode_status_pub_ =
    node_->create_publisher<ai_sapiens_interfaces::msg::ModeStatus>(mode_status_topic, 10);
  if (mode_status_publish_period > 0.0) {
    mode_status_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(mode_status_publish_period)),
      [this]() {publish_status();});
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[ModeRosInterface] set(%s) list(%s) status(%s) period(%.3fs)",
    set_mode_service_name.c_str(),
    list_service_name.c_str(),
    mode_status_topic.c_str(),
    mode_status_publish_period);
}

void ModeRosInterface::handle_set_mode(
  const ai_sapiens_interfaces::srv::SetModeByName::Request & request,
  ai_sapiens_interfaces::srv::SetModeByName::Response & response)
{
  const auto result = mode_controller_->set_mode_by_name(request.mode_name);
  response.success = result.success;
  response.message = result.message;
  response.active_mode = result.active_mode;
}

void ModeRosInterface::fill_mode_list(
  ai_sapiens_interfaces::srv::ListModes::Response & response) const
{
  response.success = true;
  response.modes = mode_controller_->concrete_state_names();
  response.available_modes = mode_controller_->available_service_state_names();
  response.message = "ok";
}

void ModeRosInterface::publish_status()
{
  mode_controller_->flush_transition_log();

  const auto snapshot = mode_controller_->status_snapshot();
  ai_sapiens_interfaces::msg::ModeStatus msg;
  msg.header.stamp = node_->now();
  msg.active_mode = snapshot.active_state;
  msg.authority = snapshot.authority;
  msg.teleop_input_valid = snapshot.teleop_input_valid;
  msg.api_heartbeat_valid = snapshot.api_heartbeat_valid;
  msg.api_request_available = snapshot.api_request_available;
  msg.last_transition_reason = snapshot.last_transition_reason;

  mode_status_pub_->publish(msg);
}

}  // namespace ai_sapiens_sim2real
