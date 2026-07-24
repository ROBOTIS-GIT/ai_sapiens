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
// Author: Woojin Wie, Kiwoong Park

#ifndef \
  AI_SAPIENS_JOINT_GROUP_IMPEDANCE_CONTROLLER__AI_SAPIENS_JOINT_GROUP_IMPEDANCE_CONTROLLER_HPP_
#define \
  AI_SAPIENS_JOINT_GROUP_IMPEDANCE_CONTROLLER__AI_SAPIENS_JOINT_GROUP_IMPEDANCE_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ai_sapiens_interfaces/msg/joint_impedance_command.hpp"
#include \
  "ai_sapiens_joint_group_impedance_controller/ai_sapiens_joint_group_impedance_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace ai_sapiens_joint_group_impedance_controller
{

class AiSapiensJointGroupImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
  using JointCommandMsg = ai_sapiens_interfaces::msg::JointImpedanceCommand;

  struct Command
  {
    bool valid{false};
    std::vector<double> positions;
    std::vector<double> feedforward;
    std::vector<double> kp;
    std::vector<double> kd;
  };

  controller_interface::CallbackReturn read_parameters();
  void initialize_command_storage(size_t joint_count);
  void reset_command();
  bool apply_command(const Command & command);

  realtime_tools::RealtimeBuffer<Command> rt_command_;
  Command invalid_command_;
  rclcpp::Subscription<JointCommandMsg>::SharedPtr joint_command_sub_;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  ordered_command_interfaces_;
};

}  // namespace ai_sapiens_joint_group_impedance_controller

#endif \
  // AI_SAPIENS_JOINT_GROUP_IMPEDANCE_CONTROLLER__AI_SAPIENS_JOINT_GROUP_IMPEDANCE_CONTROLLER_HPP_
