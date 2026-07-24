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

#ifndef AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__RADIOMASTER_POCKET_TELEOP_INPUT_PLUGIN_HPP_
#define AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__RADIOMASTER_POCKET_TELEOP_INPUT_PLUGIN_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <ai_sapiens_interfaces/msg/rc_status.hpp>

#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"

namespace ai_sapiens_sim2real
{

class RadiomasterPocketTeleopInputPlugin
  : public TeleopInputPluginTemplate<ai_sapiens_interfaces::msg::RcStatus>
{
public:
  // Plugin interface
  void configure(const rclcpp::Node::SharedPtr & node, const YAML::Node & config) override;
  std::string name() const override;
  std::string topic_name() const override;

  AxisRanges output_axis_ranges() const override;

private:
  using RcStatus = ai_sapiens_interfaces::msg::RcStatus;
  using RcChannel = ai_sapiens_interfaces::msg::RcChannel;

  // Core plugin hooks called by TeleopInputPluginTemplate:
  // valid -> fresh -> command, then stale/accepted side effects
  bool is_message_valid(const RcStatus & msg) const override;
  bool is_message_fresh(const RcStatus & msg) const override;
  TeleopInputCommand make_command_from_message(const RcStatus & msg) const override;
  void on_stale_message(const RcStatus & msg) const override;
  void on_message_accepted(const RcStatus & msg) override;

  struct AxisConfig
  {
    uint8_t channel{0};
    bool invert{false};
  };

  struct RcCondition
  {
    uint8_t channel{0};
    double value{1500.0};
  };

  struct InputCodeConfig
  {
    std::vector<RcCondition> conditions;
    uint16_t code{0};
  };

  struct SelectorCodeConfig
  {
    uint8_t channel{0};
    double tolerance{20.0};
    std::vector<uint16_t> required_for_input_codes;
    std::vector<std::pair<double, uint16_t>> table;
  };

  using ChannelLookup = std::array<const RcChannel *, 17>;

  // Realtime tick freshness
  bool is_fresh_tick(uint32_t realtime_tick) const;
  void log_tick_above_max(uint32_t realtime_tick) const;
  void log_stale_tick(uint32_t realtime_tick) const;

  // Config parsing
  void read_config(const YAML::Node & config);
  void read_velocity_command_config(const YAML::Node & config);
  void update_always_required_channels();
  void add_always_required_channel(uint8_t channel);
  static AxisConfig read_axis_config(const YAML::Node & channels, const std::string & name);
  static std::vector<RcCondition> read_optional_rc_conditions(const YAML::Node & node);
  static std::vector<InputCodeConfig> read_input_code_configs(const YAML::Node & input_code);
  static void read_input_code_channel_map(
    const YAML::Node & channels,
    const std::vector<RcCondition> & parent_conditions,
    std::vector<InputCodeConfig> & input_codes);
  static SelectorCodeConfig read_selector_code_config(const YAML::Node & selector_code);
  static uint8_t read_channel_id(const YAML::Node & node, const std::string & path);
  static uint8_t parse_channel_id(const std::string & value, const std::string & path);

  // RC status validation and command decoding
  ChannelLookup make_channel_lookup(const RcStatus & msg) const;
  bool is_status_health_ok(const RcStatus & msg) const;
  bool are_channel_values_valid(const RcStatus & msg) const;
  bool are_required_channels_readable(const ChannelLookup & channels) const;
  bool has_readable_channel(const ChannelLookup & channels, uint8_t channel) const;
  const RcChannel * read_channel(const ChannelLookup & channels, uint8_t channel) const;
  float axis_value(const ChannelLookup & channels, const AxisConfig & axis) const;
  bool are_rc_conditions_satisfied(
    const ChannelLookup & channels,
    const std::vector<RcCondition> & conditions) const;
  bool is_rc_condition_satisfied(
    const ChannelLookup & channels,
    const RcCondition & condition) const;
  bool is_input_code_decodable(const ChannelLookup & channels) const;
  bool is_selector_code_decodable_if_needed(const ChannelLookup & channels) const;
  bool does_input_code_require_selector_code(uint16_t input_code) const;
  bool is_api_mode_requested(const ChannelLookup & channels) const;
  uint16_t select_input_code(const ChannelLookup & channels) const;
  uint16_t select_selector_code(const ChannelLookup & channels) const;

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  double axis_deadzone_{0.0};
  double switch_match_tolerance_{50.0};

  AxisConfig linear_x_;
  AxisConfig linear_y_;
  AxisConfig angular_z_;

  std::vector<RcCondition> api_mode_conditions_;
  std::vector<InputCodeConfig> input_codes_;
  SelectorCodeConfig selector_code_;
  std::vector<uint8_t> always_required_channels_;

  rclcpp::Subscription<RcStatus>::SharedPtr subscription_;

  bool has_last_realtime_tick_{false};
  uint32_t last_realtime_tick_{0};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__PLUGINS__TELEOP_INPUT__RADIOMASTER_POCKET_TELEOP_INPUT_PLUGIN_HPP_
