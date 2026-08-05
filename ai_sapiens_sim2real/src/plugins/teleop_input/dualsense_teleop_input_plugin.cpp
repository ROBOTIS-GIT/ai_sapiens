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

#include "ai_sapiens_sim2real/plugins/teleop_input/dualsense_teleop_input_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace ai_sapiens_sim2real
{
namespace
{

bool has_node(const YAML::Node & node, const std::string & key)
{
  return node && node[key];
}

double read_double(const YAML::Node & node, const std::string & key, double fallback)
{
  return has_node(node, key) ? node[key].as<double>() : fallback;
}

bool read_bool(const YAML::Node & node, const std::string & key, bool fallback)
{
  return has_node(node, key) ? node[key].as<bool>() : fallback;
}

}  // namespace

void DualSenseTeleopInputPlugin::configure(
  const rclcpp::Node::SharedPtr & node,
  const YAML::Node & config)
{
  if (!node) {
    throw std::runtime_error("DualSense teleop input requires a ROS node");
  }

  node_ = node;
  read_config(config);

  subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
    topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
      handle_raw_message(*msg);
    });

  RCLCPP_INFO(node_->get_logger(), "%s: topic=%s", name().c_str(), topic_.c_str());
  log_selected_selector();
}

std::string DualSenseTeleopInputPlugin::name() const
{
  return "DualSenseTeleopInputPlugin";
}

std::string DualSenseTeleopInputPlugin::topic_name() const
{
  return topic_;
}

void DualSenseTeleopInputPlugin::read_config(const YAML::Node & config)
{
  if (!config) {
    throw std::runtime_error("DualSense teleop input config is empty");
  }

  topic_ = has_node(config, "topic") ? config["topic"].as<std::string>() : "/joy";
  if (topic_.empty()) {
    throw std::runtime_error("DualSense teleop input topic must not be empty");
  }

  deadzone_ = read_double(config, "deadzone", deadzone_);
  if (deadzone_ < 0.0 || deadzone_ >= 1.0) {
    throw std::runtime_error("DualSense teleop input deadzone must be in [0, 1)");
  }

  const auto velocity_command = config["velocity_command"];
  if (!velocity_command || !velocity_command["axes"]) {
    throw std::runtime_error("DualSense teleop input config requires velocity_command.axes");
  }
  const auto axes = velocity_command["axes"];
  linear_x_ = read_axis_config(axes, "linear_x");
  linear_y_ = read_axis_config(axes, "linear_y");
  angular_z_ = read_axis_config(axes, "angular_z");

  api_mode_conditions_ = read_button_conditions(config["api_mode"]);
  input_codes_ = read_button_codes(config["input_code"], "input_code", true);
  selector_codes_ = read_button_codes(config["selector_code"], "selector_code", false);
  selector_axis_codes_ = read_axis_codes(config["selector_code"], "selector_code");
  read_selector_navigation(config["selector_navigation"]);
  log_selected_selector_enabled_ =
    read_bool(config, "log_selected_selector_enabled", true);

  if (selector_navigation_enabled_ &&
    (!selector_codes_.empty() || !selector_axis_codes_.empty()))
  {
    throw std::runtime_error(
      "selector_navigation cannot be combined with momentary selector_code mappings");
  }
}

void DualSenseTeleopInputPlugin::read_selector_navigation(const YAML::Node & node)
{
  selector_navigation_enabled_ = false;
  selector_options_.clear();
  selected_selector_index_ = 0;
  previous_buttons_.clear();
  if (!node) {
    return;
  }
  if (!node["previous_button"] || !node["next_button"] || !node["options"]) {
    throw std::runtime_error(
      "selector_navigation requires previous_button, next_button, and options");
  }

  selector_previous_button_ = node["previous_button"].as<std::size_t>();
  selector_next_button_ = node["next_button"].as<std::size_t>();
  if (selector_previous_button_ == selector_next_button_) {
    throw std::runtime_error("selector_navigation buttons must be different");
  }

  const auto options = node["options"];
  if (!options.IsSequence() || options.size() == 0) {
    throw std::runtime_error("selector_navigation.options must be a non-empty sequence");
  }
  for (const auto & option : options) {
    if (!option["code"] || !option["label"]) {
      throw std::runtime_error("selector_navigation options require code and label");
    }
    const auto code = option["code"].as<uint16_t>();
    const auto label = option["label"].as<std::string>();
    if (code == 0 || label.empty()) {
      throw std::runtime_error("selector_navigation option code and label must be non-empty");
    }
    const bool duplicate = std::any_of(
      selector_options_.begin(), selector_options_.end(),
      [code](const SelectorOption & existing) {return existing.code == code;});
    if (duplicate) {
      throw std::runtime_error("selector_navigation option codes must be unique");
    }
    selector_options_.push_back({code, label});
  }

  if (node["initial_code"]) {
    const auto initial_code = node["initial_code"].as<uint16_t>();
    const auto initial = std::find_if(
      selector_options_.begin(), selector_options_.end(),
      [initial_code](const SelectorOption & option) {return option.code == initial_code;});
    if (initial == selector_options_.end()) {
      throw std::runtime_error("selector_navigation.initial_code is not in options");
    }
    selected_selector_index_ =
      static_cast<std::size_t>(std::distance(selector_options_.begin(), initial));
  }
  selector_navigation_enabled_ = true;
}

DualSenseTeleopInputPlugin::AxisConfig DualSenseTeleopInputPlugin::read_axis_config(
  const YAML::Node & axes,
  const std::string & name)
{
  if (!axes || !axes[name] || !axes[name]["index"]) {
    throw std::runtime_error("velocity_command.axes." + name + ".index is required");
  }
  const auto axis = axes[name];
  return AxisConfig{
    axis["index"].as<std::size_t>(),
    read_bool(axis, "invert", false)};
}

std::vector<DualSenseTeleopInputPlugin::ButtonCondition>
DualSenseTeleopInputPlugin::read_button_conditions(const YAML::Node & node)
{
  std::vector<ButtonCondition> conditions;
  if (!node) {
    return conditions;
  }

  const auto when = node["when"] ? node["when"] : node;
  if (when["button"]) {
    conditions.push_back({
        when["button"].as<std::size_t>(),
        read_bool(when, "pressed", true)});
    return conditions;
  }

  const auto buttons = when["buttons"] ? when["buttons"] : when;
  if (!buttons || !buttons.IsMap()) {
    throw std::runtime_error("api_mode requires button or buttons map");
  }
  for (const auto & item : buttons) {
    conditions.push_back({
        item.first.as<std::size_t>(),
        item.second.as<bool>()});
  }
  return conditions;
}

std::vector<DualSenseTeleopInputPlugin::ButtonCode>
DualSenseTeleopInputPlugin::read_button_codes(
  const YAML::Node & node,
  const char * name,
  bool required)
{
  if (!node || !node["buttons"]) {
    if (!required) {
      return {};
    }
    throw std::runtime_error(std::string(name) + ".buttons is required");
  }

  std::vector<ButtonCode> codes;
  const auto buttons = node["buttons"];
  if (buttons.IsMap()) {
    for (const auto & item : buttons) {
      codes.push_back({
          item.first.as<std::size_t>(),
          item.second.as<uint16_t>()});
    }
    return codes;
  }
  if (buttons.IsSequence()) {
    for (const auto & item : buttons) {
      if (!item["button"] || !item["code"]) {
        throw std::runtime_error(std::string(name) + ".buttons entries require button and code");
      }
      codes.push_back({
          item["button"].as<std::size_t>(),
          item["code"].as<uint16_t>()});
    }
    return codes;
  }
  throw std::runtime_error(std::string(name) + ".buttons must be a map or sequence");
}

std::vector<DualSenseTeleopInputPlugin::AxisCode>
DualSenseTeleopInputPlugin::read_axis_codes(
  const YAML::Node & node,
  const char * name)
{
  if (!node || !node["axes"]) {
    return {};
  }

  const auto axes = node["axes"];
  if (!axes.IsSequence()) {
    throw std::runtime_error(std::string(name) + ".axes must be a sequence");
  }

  std::vector<AxisCode> codes;
  for (const auto & item : axes) {
    if (!item["axis"] || !item["code"]) {
      throw std::runtime_error(std::string(name) + ".axes entries require axis and code");
    }
    const float minimum = item["minimum"] ? item["minimum"].as<float>() : 0.5f;
    if (!std::isfinite(minimum) || minimum < -1.0f || minimum > 1.0f) {
      throw std::runtime_error(std::string(name) + ".axes minimum must be in [-1, 1]");
    }
    codes.push_back({
        item["axis"].as<std::size_t>(),
        minimum,
        item["code"].as<uint16_t>()});
  }
  return codes;
}

bool DualSenseTeleopInputPlugin::is_message_valid(const sensor_msgs::msg::Joy & msg) const
{
  if (!has_configured_axes(msg)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "%s: waiting for Joy axes required by config", name().c_str());
    return false;
  }
  if (!has_finite_axes(msg)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "%s: rejecting non-finite Joy axis value", name().c_str());
    return false;
  }
  if (!has_configured_buttons(msg)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "%s: waiting for Joy buttons required by config", name().c_str());
    return false;
  }
  return true;
}

bool DualSenseTeleopInputPlugin::is_message_fresh(
  const sensor_msgs::msg::Joy & /*msg*/) const
{
  // sensor_msgs/Joy does not expose a packet sequence. The shared teleop
  // watchdog uses the steady-clock acceptance time to detect a stopped stream.
  return true;
}

TeleopInputCommand DualSenseTeleopInputPlugin::make_command_from_message(
  const sensor_msgs::msg::Joy & msg) const
{
  TeleopInputCommand command;
  command.api_mode = are_button_conditions_satisfied(msg, api_mode_conditions_);
  command.input_code = select_input_code(msg);
  command.selector_code = select_selector_code(msg);
  command.velocity.x() = axis_value(msg, linear_x_);
  command.velocity.y() = axis_value(msg, linear_y_);
  command.velocity.z() = axis_value(msg, angular_z_);
  return command;
}

void DualSenseTeleopInputPlugin::on_message_accepted(const sensor_msgs::msg::Joy & msg)
{
  const bool button_pressed = has_any_button_rising_edge(msg);
  apply_selector_navigation(msg);
  if (button_pressed) {
    log_selected_selector();
  }
  remember_button_state(msg);
}

bool DualSenseTeleopInputPlugin::has_configured_axes(
  const sensor_msgs::msg::Joy & msg) const
{
  const auto axis_count = msg.axes.size();
  if (linear_x_.index >= axis_count ||
    linear_y_.index >= axis_count ||
    angular_z_.index >= axis_count)
  {
    return false;
  }
  for (const auto & selector : selector_axis_codes_) {
    if (selector.index >= axis_count) {
      return false;
    }
  }
  return true;
}

bool DualSenseTeleopInputPlugin::has_finite_axes(const sensor_msgs::msg::Joy & msg) const
{
  if (!std::isfinite(msg.axes[linear_x_.index]) ||
    !std::isfinite(msg.axes[linear_y_.index]) ||
    !std::isfinite(msg.axes[angular_z_.index]))
  {
    return false;
  }
  for (const auto & selector : selector_axis_codes_) {
    if (!std::isfinite(msg.axes[selector.index])) {
      return false;
    }
  }
  return true;
}

bool DualSenseTeleopInputPlugin::has_configured_buttons(const sensor_msgs::msg::Joy & msg) const
{
  const auto button_count = msg.buttons.size();
  for (const auto & condition : api_mode_conditions_) {
    if (condition.index >= button_count) {
      return false;
    }
  }
  for (const auto & input_code : input_codes_) {
    if (input_code.index >= button_count) {
      return false;
    }
  }
  for (const auto & selector_code : selector_codes_) {
    if (selector_code.index >= button_count) {
      return false;
    }
  }
  if (selector_navigation_enabled_ &&
    (selector_previous_button_ >= button_count || selector_next_button_ >= button_count))
  {
    return false;
  }
  return true;
}

bool DualSenseTeleopInputPlugin::are_button_conditions_satisfied(
  const sensor_msgs::msg::Joy & msg,
  const std::vector<ButtonCondition> & conditions) const
{
  if (conditions.empty()) {
    return false;
  }
  for (const auto & condition : conditions) {
    if (is_button_pressed(msg, condition.index) != condition.pressed) {
      return false;
    }
  }
  return true;
}

bool DualSenseTeleopInputPlugin::is_button_pressed(
  const sensor_msgs::msg::Joy & msg,
  std::size_t index)
{
  return index < msg.buttons.size() && msg.buttons[index] != 0;
}

float DualSenseTeleopInputPlugin::axis_value(
  const sensor_msgs::msg::Joy & msg,
  const AxisConfig & axis) const
{
  float value = std::clamp(msg.axes[axis.index], -1.0f, 1.0f);
  const float magnitude = std::abs(value);
  const float deadzone = static_cast<float>(deadzone_);
  if (magnitude <= deadzone) {
    return 0.0f;
  }

  // Remove the deadzone continuously, then use the full remaining stick travel.
  value = std::copysign((magnitude - deadzone) / (1.0f - deadzone), value);
  if (axis.invert) {
    value *= -1.0f;
  }
  return value;
}

uint16_t DualSenseTeleopInputPlugin::select_input_code(
  const sensor_msgs::msg::Joy & msg) const
{
  for (const auto & input_code : input_codes_) {
    if (is_button_pressed(msg, input_code.index)) {
      return input_code.code;
    }
  }
  return 0;
}

uint16_t DualSenseTeleopInputPlugin::select_selector_code(
  const sensor_msgs::msg::Joy & msg) const
{
  if (selector_navigation_enabled_) {
    return selector_options_[selected_index_after_input(msg)].code;
  }
  for (const auto & selector_code : selector_codes_) {
    if (is_button_pressed(msg, selector_code.index)) {
      return selector_code.code;
    }
  }
  for (const auto & selector_code : selector_axis_codes_) {
    if (msg.axes[selector_code.index] >= selector_code.minimum) {
      return selector_code.code;
    }
  }
  return 0;
}

bool DualSenseTeleopInputPlugin::is_button_rising_edge(
  const sensor_msgs::msg::Joy & msg,
  std::size_t button) const
{
  const bool was_pressed =
    button < previous_buttons_.size() && previous_buttons_[button] != 0;
  return is_button_pressed(msg, button) && !was_pressed;
}

bool DualSenseTeleopInputPlugin::has_any_button_rising_edge(
  const sensor_msgs::msg::Joy & msg) const
{
  for (std::size_t button = 0; button < msg.buttons.size(); ++button) {
    if (is_button_rising_edge(msg, button)) {
      return true;
    }
  }
  return false;
}

int DualSenseTeleopInputPlugin::selector_navigation_step(
  const sensor_msgs::msg::Joy & msg) const
{
  if (!selector_navigation_enabled_) {
    return 0;
  }
  const bool previous = is_button_rising_edge(msg, selector_previous_button_);
  const bool next = is_button_rising_edge(msg, selector_next_button_);
  if (previous == next) {
    return 0;
  }
  return previous ? -1 : 1;
}

std::size_t DualSenseTeleopInputPlugin::selected_index_after_input(
  const sensor_msgs::msg::Joy & msg) const
{
  const int step = selector_navigation_step(msg);
  if (step < 0) {
    return selected_selector_index_ == 0 ?
           selector_options_.size() - 1 : selected_selector_index_ - 1;
  }
  if (step > 0) {
    return (selected_selector_index_ + 1) % selector_options_.size();
  }
  return selected_selector_index_;
}

void DualSenseTeleopInputPlugin::apply_selector_navigation(
  const sensor_msgs::msg::Joy & msg)
{
  if (selector_navigation_enabled_) {
    selected_selector_index_ = selected_index_after_input(msg);
  }
}

void DualSenseTeleopInputPlugin::remember_button_state(
  const sensor_msgs::msg::Joy & msg)
{
  previous_buttons_ = msg.buttons;
}

void DualSenseTeleopInputPlugin::log_selected_selector() const
{
  if (!node_ || !log_selected_selector_enabled_) {
    return;
  }

  if (selector_navigation_enabled_) {
    const auto & selected = selector_options_[selected_selector_index_];
    RCLCPP_INFO(
      node_->get_logger(), "%s: selected [%zu/%zu] %s (selector=%u)",
      name().c_str(), selected_selector_index_ + 1, selector_options_.size(),
      selected.label.c_str(), static_cast<unsigned int>(selected.code));
  }
}

}  // namespace ai_sapiens_sim2real

PLUGINLIB_EXPORT_CLASS(
  ai_sapiens_sim2real::DualSenseTeleopInputPlugin,
  ai_sapiens_sim2real::TeleopInputPluginBase)
