// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Kiwoong Park

#include "ai_sapiens_sim2real/teleop_devtools/keyboard/keyboard_teleop_input_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace ai_sapiens_sim2real
{

void KeyboardTeleopInputPlugin::configure(
  const rclcpp::Node::SharedPtr & node,
  const YAML::Node & config)
{
  if (!node) {
    throw std::runtime_error("keyboard teleop input requires a ROS node");
  }

  node_ = node;
  auto resolved_config = YAML::Clone(config);
  const auto navigation = resolved_config["selector_navigation"];
  if (navigation && navigation["selector"] && !navigation["options"]) {
    if (!node_->has_parameter("config_path")) {
      throw std::runtime_error(
              "keyboard selector_navigation requires the node config_path parameter");
    }
    const auto root_config_path = node_->get_parameter("config_path").as_string();
    if (root_config_path.empty()) {
      throw std::runtime_error(
              "keyboard selector_navigation requires a non-empty node config_path");
    }
    resolved_config = resolve_keyboard_selector_config(
      config, YAML::LoadFile(root_config_path));
  }
  config_ = KeyboardTeleopConfig::from_yaml(resolved_config);
  subscription_ = node_->create_subscription<KeyboardInput>(
    config_.topic, rclcpp::SensorDataQoS(),
    [this](const KeyboardInput::SharedPtr message) {
      handle_raw_message(*message);
    });

  RCLCPP_INFO(
    node_->get_logger(), "%s: topic=%s", name().c_str(), config_.topic.c_str());
}

std::string KeyboardTeleopInputPlugin::name() const
{
  return "KeyboardTeleopInputPlugin";
}

std::string KeyboardTeleopInputPlugin::topic_name() const
{
  return config_.topic;
}

bool KeyboardTeleopInputPlugin::is_message_valid(const KeyboardInput & message) const
{
  const bool finite =
    std::isfinite(message.linear_x) &&
    std::isfinite(message.linear_y) &&
    std::isfinite(message.angular_z);
  const bool normalized =
    std::abs(message.linear_x) <= 1.0F &&
    std::abs(message.linear_y) <= 1.0F &&
    std::abs(message.angular_z) <= 1.0F;
  if (!finite || !normalized) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "%s: rejecting non-finite or non-normalized velocity input", name().c_str());
    return false;
  }
  if (!is_known_input_code(message.input_code)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "%s: rejecting unknown input code %u", name().c_str(),
      static_cast<unsigned int>(message.input_code));
    return false;
  }
  if (!is_known_selector_code(message.selector_code)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "%s: rejecting unknown selector code %u", name().c_str(),
      static_cast<unsigned int>(message.selector_code));
    return false;
  }
  return true;
}

bool KeyboardTeleopInputPlugin::is_message_fresh(const KeyboardInput & message) const
{
  if (!has_last_sequence_) {
    return true;
  }
  const uint32_t forward_distance = message.sequence - last_sequence_;
  return forward_distance != 0U && forward_distance < 0x80000000U;
}

TeleopInputCommand KeyboardTeleopInputPlugin::make_command_from_message(
  const KeyboardInput & message) const
{
  TeleopInputCommand command;
  command.api_mode = message.api_mode;
  command.input_code = message.input_code;
  command.selector_code = message.selector_code;
  command.velocity.x() = message.linear_x;
  command.velocity.y() = message.linear_y;
  command.velocity.z() = message.angular_z;
  return command;
}

void KeyboardTeleopInputPlugin::on_stale_message(const KeyboardInput & message) const
{
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "%s: ignoring non-advancing sequence (current=%u, last=%u)",
    name().c_str(), static_cast<unsigned int>(message.sequence),
    static_cast<unsigned int>(last_sequence_));
}

void KeyboardTeleopInputPlugin::on_message_accepted(const KeyboardInput & message)
{
  has_last_sequence_ = true;
  last_sequence_ = message.sequence;
}

bool KeyboardTeleopInputPlugin::is_known_input_code(uint16_t code) const
{
  return
    code == 0 ||
    code == config_.damping_code ||
    code == config_.ready_pose_code ||
    code == config_.velocity_code ||
    code == config_.mimic_code;
}

bool KeyboardTeleopInputPlugin::is_known_selector_code(uint16_t code) const
{
  return std::any_of(
    config_.selector_options.begin(), config_.selector_options.end(),
    [code](const KeyboardSelectorOption & option) {
      return option.code == code;
    });
}

}  // namespace ai_sapiens_sim2real

PLUGINLIB_EXPORT_CLASS(
  ai_sapiens_sim2real::KeyboardTeleopInputPlugin,
  ai_sapiens_sim2real::TeleopInputPluginBase)
