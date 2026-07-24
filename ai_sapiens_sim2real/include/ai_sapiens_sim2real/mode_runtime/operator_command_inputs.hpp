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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__OPERATOR_COMMAND_INPUTS_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__OPERATOR_COMMAND_INPUTS_HPP_

#include <memory>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/control_loop.hpp"
#include "ai_sapiens_sim2real/mode_runtime/operator_command_input_options.hpp"
#include "ai_sapiens_sim2real/sensor_handles/api_heartbeat_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/teleop_input_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/twist_command_handle.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"

namespace ai_sapiens_sim2real
{

struct OperatorCommandInputs
{
  std::shared_ptr<pluginlib::ClassLoader<TeleopInputPluginBase>> teleop_input_loader;
  std::shared_ptr<TeleopInputPluginBase> teleop_input_plugin;
};

// Register teleop/API command input handles with the control loop.
OperatorCommandInputs add_operator_command_input_handles(
  const std::shared_ptr<ControlLoop> & control_loop,
  const rclcpp::Node::SharedPtr & node,
  SharedControlData * state,
  const OperatorCommandInputOptions & options);

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__OPERATOR_COMMAND_INPUTS_HPP_
