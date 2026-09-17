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

#include "ai_sapiens_sim2real/mode_runtime/operator_command_inputs.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/config/config_utils.hpp"
#include "ai_sapiens_sim2real/control_loop.hpp"
#include "ai_sapiens_sim2real/sensor_handles/api_heartbeat_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/teleop_input_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/twist_command_handle.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"
#include "ai_sapiens_sim2real/sensor_handles/group_teleop_handle.hpp"

namespace ai_sapiens_sim2real
{

namespace
{

std::shared_ptr<GroupTeleopHandle> make_group_input_handle(
  OperatorCommandInputs & inputs, const rclcpp::Node::SharedPtr & node,
  SharedControlData * state, const OperatorCommandInputOptions & options)
{
  inputs.group_input_plugin =
    inputs.teleop_input_loader->createSharedInstance(options.group->plugin);
  inputs.group_input_plugin->configure(
    node, load_yaml_file(options.group->config_path, "group plugin config"));
  if (inputs.group_input_plugin->topic_name() == inputs.teleop_input_plugin->topic_name()) {
    throw std::runtime_error("individual and group inputs must use different topics");
  }
  return std::make_shared<GroupTeleopHandle>(
    node, state, options, inputs.teleop_input_plugin, inputs.group_input_plugin);
}

}  // namespace

OperatorCommandInputs add_operator_command_input_handles(
  const std::shared_ptr<ControlLoop> & control_loop,
  const rclcpp::Node::SharedPtr & node,
  SharedControlData * state,
  const OperatorCommandInputOptions & options)
{
  OperatorCommandInputs inputs;
  inputs.teleop_input_loader =
    std::make_shared<pluginlib::ClassLoader<TeleopInputPluginBase>>(
    "ai_sapiens_sim2real", "ai_sapiens_sim2real::TeleopInputPluginBase");
  inputs.teleop_input_plugin =
    inputs.teleop_input_loader->createSharedInstance(options.teleop_input_plugin);
  inputs.teleop_input_plugin->configure(
    node, individual_input_config(
      load_yaml_file(options.teleop_input_config_path, "teleop input plugin config"),
      options.group.has_value()));

  if (options.group.has_value()) {
    control_loop->add_sensor_handle(make_group_input_handle(inputs, node, state, options), true);
  } else {
    control_loop->add_sensor_handle(
      std::make_shared<TeleopInputHandle>(
          node,
          &state->teleop,
          &state->requests,
          &state->active_velocity_command_ranges,
          inputs.teleop_input_plugin,
          options.teleop_input_timeout,
          options.teleop_vel_command_timeout),
      true);
  }
  control_loop->add_sensor_handle(
    std::make_shared<ApiHeartbeatHandle>(
        node,
        &state->api,
        options.api_heartbeat_topic,
        options.api_heartbeat_timeout),
    false);
  control_loop->add_sensor_handle(
    std::make_shared<TwistCommandHandle>(
        node,
        &state->api,
        &state->active_velocity_command_ranges,
        options.cmd_vel_topic),
    false);
  return inputs;
}

}  // namespace ai_sapiens_sim2real
