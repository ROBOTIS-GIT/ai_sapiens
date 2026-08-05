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

#ifndef AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__DUALSENSE_TELEOP_INPUT_PLUGIN_HPP_
#define AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__DUALSENSE_TELEOP_INPUT_PLUGIN_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/joy.hpp>

#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"

namespace ai_sapiens_sim2real
{

class DualSenseTeleopInputPlugin
  : public TeleopInputPluginTemplate<sensor_msgs::msg::Joy>
{
public:
  void configure(
    const rclcpp::Node::SharedPtr & node,
    const YAML::Node & config) override;
  std::string name() const override;
  std::string topic_name() const override;

  // Joy axes are normalized to [-1, 1] by sensor_msgs/msg/Joy producers.
  AxisRanges output_axis_ranges() const override
  {
    return AxisRanges{{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};
  }

private:
  struct AxisConfig
  {
    std::size_t index{0};
    bool invert{false};
  };

  struct ButtonCondition
  {
    std::size_t index{0};
    bool pressed{true};
  };

  struct ButtonCode
  {
    std::size_t index{0};
    uint16_t code{0};
    bool latch{true};
  };

  struct AxisCode
  {
    std::size_t index{0};
    float minimum{0.5f};
    uint16_t code{0};
  };

  struct SelectorOption
  {
    uint16_t code{0};
    std::string label;
  };

  void read_config(const YAML::Node & config);
  void read_selector_navigation(const YAML::Node & node);
  static AxisConfig read_axis_config(const YAML::Node & axes, const std::string & name);
  static std::vector<ButtonCondition> read_button_conditions(const YAML::Node & node);
  static std::vector<ButtonCode> read_button_codes(
    const YAML::Node & node,
    const char * name,
    bool required);
  static std::vector<AxisCode> read_axis_codes(
    const YAML::Node & node,
    const char * name);

  bool is_message_valid(const sensor_msgs::msg::Joy & msg) const override;
  bool is_message_fresh(const sensor_msgs::msg::Joy & msg) const override;
  TeleopInputCommand make_command_from_message(const sensor_msgs::msg::Joy & msg) const override;
  void on_message_accepted(const sensor_msgs::msg::Joy & msg) override;

  bool has_configured_axes(const sensor_msgs::msg::Joy & msg) const;
  bool has_finite_axes(const sensor_msgs::msg::Joy & msg) const;
  bool has_configured_buttons(const sensor_msgs::msg::Joy & msg) const;
  bool are_button_conditions_satisfied(
    const sensor_msgs::msg::Joy & msg,
    const std::vector<ButtonCondition> & conditions) const;
  static bool is_button_pressed(const sensor_msgs::msg::Joy & msg, std::size_t index);
  float axis_value(const sensor_msgs::msg::Joy & msg, const AxisConfig & axis) const;
  const ButtonCode * pressed_input_code(const sensor_msgs::msg::Joy & msg) const;
  uint16_t select_input_code(const sensor_msgs::msg::Joy & msg) const;
  uint16_t select_selector_code(const sensor_msgs::msg::Joy & msg) const;
  void apply_input_code_latch(const sensor_msgs::msg::Joy & msg);

  // Persistent selector navigation is kept separate from command conversion.
  bool is_button_rising_edge(
    const sensor_msgs::msg::Joy & msg,
    std::size_t button) const;
  bool has_any_button_rising_edge(const sensor_msgs::msg::Joy & msg) const;
  int selector_navigation_step(const sensor_msgs::msg::Joy & msg) const;
  std::size_t selected_index_after_input(const sensor_msgs::msg::Joy & msg) const;
  void apply_selector_navigation(const sensor_msgs::msg::Joy & msg);
  void remember_button_state(const sensor_msgs::msg::Joy & msg);
  void log_selected_selector() const;

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  double deadzone_{0.08};

  AxisConfig linear_x_;
  AxisConfig linear_y_;
  AxisConfig angular_z_;

  std::vector<ButtonCondition> api_mode_conditions_;
  std::vector<ButtonCode> input_codes_;
  uint16_t latched_input_code_{0};
  std::vector<ButtonCode> selector_codes_;
  std::vector<AxisCode> selector_axis_codes_;

  bool selector_navigation_enabled_{false};
  std::size_t selector_previous_button_{0};
  std::size_t selector_next_button_{0};
  std::size_t selected_selector_index_{0};
  std::vector<SelectorOption> selector_options_;
  std::vector<int32_t> previous_buttons_;
  bool log_selected_selector_enabled_{true};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__DUALSENSE_TELEOP_INPUT_PLUGIN_HPP_
