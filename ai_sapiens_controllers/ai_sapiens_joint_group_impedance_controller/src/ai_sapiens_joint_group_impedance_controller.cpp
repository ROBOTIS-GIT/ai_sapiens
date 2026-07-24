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

#include \
  "ai_sapiens_joint_group_impedance_controller/ai_sapiens_joint_group_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ai_sapiens_joint_group_impedance_controller
{
namespace
{

constexpr char kFeedforwardInterface[] = "feedforward";

bool all_finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
             return std::isfinite(value);
  });
}

}  // namespace

controller_interface::CallbackReturn AiSapiensJointGroupImpedanceController::read_parameters()
{
  try {
    params_ = param_listener_->get_params();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "joints parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = params_.joints;

  const std::string pos_suffix = "/" + std::string(hardware_interface::HW_IF_POSITION);
  const std::string feedforward_suffix = "/" + std::string(kFeedforwardInterface);
  const std::string proportional_suffix = "/" +
    std::string(hardware_interface::HW_IF_PROPORTIONAL_GAIN);
  const std::string derivative_suffix = "/" +
    std::string(hardware_interface::HW_IF_DERIVATIVE_GAIN);

  command_interface_names_.clear();
  command_interface_names_.reserve(joint_names_.size() * 4u);
  for (const std::string & joint : joint_names_) {
    command_interface_names_.push_back(joint + pos_suffix);
  }

  for (const std::string & joint : joint_names_) {
    command_interface_names_.push_back(joint + feedforward_suffix);
  }

  for (const std::string & joint : joint_names_) {
    command_interface_names_.push_back(joint + proportional_suffix);
    command_interface_names_.push_back(joint + derivative_suffix);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AiSapiensJointGroupImpedanceController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception & ex) {
    fprintf(stderr, "Exception during controller initialization: %s\n", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void AiSapiensJointGroupImpedanceController::initialize_command_storage(size_t joint_count)
{
  invalid_command_.valid = false;
  invalid_command_.positions.resize(joint_count);
  invalid_command_.feedforward.resize(joint_count);
  invalid_command_.kp.resize(joint_count);
  invalid_command_.kd.resize(joint_count);
  rt_command_.initRT(invalid_command_);
}

void AiSapiensJointGroupImpedanceController::reset_command()
{
  rt_command_.writeFromNonRT(invalid_command_);
}

controller_interface::CallbackReturn AiSapiensJointGroupImpedanceController::on_configure(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  const size_t num_dof = joint_names_.size();
  initialize_command_storage(num_dof);

  joint_command_sub_ = get_node()->create_subscription<JointCommandMsg>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this, num_dof](const JointCommandMsg::SharedPtr msg) {
      if (msg->positions.size() != num_dof ||
      msg->feedforward.size() != num_dof ||
      msg->kp.size() != num_dof ||
      msg->kd.size() != num_dof)
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "~/commands sizes positions=%zu feedforward=%zu kp=%zu kd=%zu != %zu "
          "(joint order). Dropping.",
          msg->positions.size(), msg->feedforward.size(), msg->kp.size(), msg->kd.size(),
          num_dof);
        return;
      }
      if (!all_finite(msg->positions) || !all_finite(msg->feedforward) ||
      !all_finite(msg->kp) || !all_finite(msg->kd))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Non-finite value in ~/commands. Dropping.");
        return;
      }

      Command command;
      command.valid = true;
      command.positions = msg->positions;
      command.feedforward = msg->feedforward;
      command.kp = msg->kp;
      command.kd = msg->kd;
      rt_command_.writeFromNonRT(command);
    });

  RCLCPP_INFO(get_node()->get_logger(), "Configured for %zu joints.", num_dof);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AiSapiensJointGroupImpedanceController::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  ordered_command_interfaces_.clear();
  if (
    command_interfaces_.size() != command_interface_names_.size() ||
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_names_, "", ordered_command_interfaces_) ||
    ordered_command_interfaces_.size() != command_interface_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, matched %zu of %zu assigned interfaces",
      command_interface_names_.size(), ordered_command_interfaces_.size(),
      command_interfaces_.size());
    ordered_command_interfaces_.clear();
    return controller_interface::CallbackReturn::ERROR;
  }

  reset_command();
  RCLCPP_INFO(get_node()->get_logger(), "Activated successfully.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AiSapiensJointGroupImpedanceController::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  reset_command();
  ordered_command_interfaces_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AiSapiensJointGroupImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;
  return config;
}

controller_interface::InterfaceConfiguration
AiSapiensJointGroupImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type AiSapiensJointGroupImpedanceController::update(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  const Command * command = rt_command_.readFromRT();
  if (!command->valid) {
    return controller_interface::return_type::OK;
  }

  apply_command(*command);
  return controller_interface::return_type::OK;
}

bool AiSapiensJointGroupImpedanceController::apply_command(const Command & command)
{
  const size_t joint_count = joint_names_.size();
  const size_t feedforward_start = joint_count;
  const size_t gain_start = joint_count * 2u;

  const auto set_value = [this](size_t index, double value) {
      auto & command_interface = ordered_command_interfaces_[index].get();
      if (!command_interface.set_value(value)) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Unable to set %s to %.9g", command_interface.get_name().c_str(), value);
        return false;
      }
      return true;
    };

  for (size_t i = 0; i < joint_count; ++i) {
    if (!set_value(i, command.positions[i])) {
      return false;
    }
  }
  for (size_t i = 0; i < joint_count; ++i) {
    if (!set_value(feedforward_start + i, command.feedforward[i])) {
      return false;
    }
  }
  for (size_t i = 0; i < joint_count; ++i) {
    if (!set_value(gain_start + 2u * i, command.kp[i]) ||
      !set_value(gain_start + 2u * i + 1u, command.kd[i]))
    {
      return false;
    }
  }
  return true;
}

}  // namespace ai_sapiens_joint_group_impedance_controller

PLUGINLIB_EXPORT_CLASS(
  ai_sapiens_joint_group_impedance_controller::AiSapiensJointGroupImpedanceController,
  controller_interface::ControllerInterface)
