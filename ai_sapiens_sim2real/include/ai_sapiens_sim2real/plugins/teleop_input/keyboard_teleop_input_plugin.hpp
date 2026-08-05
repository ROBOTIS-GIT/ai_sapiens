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

#ifndef AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__KEYBOARD_TELEOP_INPUT_PLUGIN_HPP_
#define AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__KEYBOARD_TELEOP_INPUT_PLUGIN_HPP_

#include <cstdint>
#include <string>

#include <ai_sapiens_interfaces/msg/keyboard_input.hpp>

#include "ai_sapiens_sim2real/teleop_devtools/keyboard/keyboard_teleop.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"

namespace ai_sapiens_sim2real
{

class KeyboardTeleopInputPlugin
  : public TeleopInputPluginTemplate<ai_sapiens_interfaces::msg::KeyboardInput>
{
public:
  void configure(
    const rclcpp::Node::SharedPtr & node,
    const YAML::Node & config) override;
  std::string name() const override;
  std::string topic_name() const override;

  AxisRanges output_axis_ranges() const override
  {
    return AxisRanges{{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};
  }

private:
  using KeyboardInput = ai_sapiens_interfaces::msg::KeyboardInput;

  bool is_message_valid(const KeyboardInput & message) const override;
  bool is_message_fresh(const KeyboardInput & message) const override;
  TeleopInputCommand make_command_from_message(const KeyboardInput & message) const override;
  void on_stale_message(const KeyboardInput & message) const override;
  void on_message_accepted(const KeyboardInput & message) override;

  bool is_known_input_code(uint16_t code) const;
  bool is_known_selector_code(uint16_t code) const;

  rclcpp::Node::SharedPtr node_;
  KeyboardTeleopConfig config_;
  rclcpp::Subscription<KeyboardInput>::SharedPtr subscription_;
  bool has_last_sequence_{false};
  uint32_t last_sequence_{0};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__KEYBOARD_TELEOP_INPUT_PLUGIN_HPP_
