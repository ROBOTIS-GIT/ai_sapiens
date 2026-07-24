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

#include "ai_sapiens_sim2real/plugins/teleop_input/radiomaster_pocket_teleop_input_plugin.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace ai_sapiens_sim2real
{
namespace
{

namespace realtime_tick
{

constexpr uint32_t kMax = 32767U;
constexpr uint64_t kCounterSize = static_cast<uint64_t>(kMax) + 1;
constexpr uint64_t kHalfCycle = kCounterSize / 2;

bool is_valid(uint32_t tick)
{
  return tick <= kMax;
}

bool is_newer_than_last(uint32_t current, uint32_t last)
{
  if (current == last) {
    return false;
  }

  // Compare the circular 0..32767 counter by the forward distance from last to current.
  // A distance of half a cycle or more is ambiguous, so it is not accepted as newer.
  const uint64_t current_value = current;
  const uint64_t last_value = last;
  const uint64_t forward_steps =
    (current_value + kCounterSize - last_value) % kCounterSize;
  return forward_steps < kHalfCycle;
}

}  // namespace realtime_tick

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

template<typename T>
bool contains_value(const std::vector<T> & values, const T & value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

bool has_configured_channel(uint8_t channel)
{
  return channel != 0U;
}

bool is_normalized_axis(float axis)
{
  return std::isfinite(axis) && axis >= -1.0F && axis <= 1.0F;
}

bool is_rc_us_within_tolerance(double actual_rc_us, double target_rc_us, double tolerance_us)
{
  return std::abs(actual_rc_us - target_rc_us) <= tolerance_us;
}

}  // namespace

// Plugin interface
void RadiomasterPocketTeleopInputPlugin::configure(
  const rclcpp::Node::SharedPtr & node,
  const YAML::Node & config)
{
  node_ = node;
  read_config(config);

  subscription_ = node_->create_subscription<RcStatus>(
    topic_,
    10,
    [this](const RcStatus::SharedPtr msg) {
      handle_raw_message(*msg);
    });

  RCLCPP_INFO(node_->get_logger(), "[%s] topic(%s)", name().c_str(), topic_.c_str());
}

std::string RadiomasterPocketTeleopInputPlugin::name() const
{
  return "RadiomasterPocketTeleopInputPlugin";
}

std::string RadiomasterPocketTeleopInputPlugin::topic_name() const
{
  return topic_;
}

AxisRanges RadiomasterPocketTeleopInputPlugin::output_axis_ranges() const
{
  return AxisRanges{{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};
}

// TeleopInputPluginTemplate hooks
bool RadiomasterPocketTeleopInputPlugin::is_message_valid(const RcStatus & msg) const
{
  return is_status_health_ok(msg) && are_channel_values_valid(msg);
}

bool RadiomasterPocketTeleopInputPlugin::is_message_fresh(
  const RcStatus & msg) const
{
  const auto current_realtime_tick = msg.realtime_tick;
  const bool is_tick_in_expected_range = realtime_tick::is_valid(current_realtime_tick);

  return is_tick_in_expected_range && is_fresh_tick(current_realtime_tick);
}

TeleopInputCommand RadiomasterPocketTeleopInputPlugin::make_command_from_message(
  const RcStatus & msg) const
{
  const auto channels = make_channel_lookup(msg);

  TeleopInputCommand command;

  command.api_mode = is_api_mode_requested(channels);
  command.input_code = select_input_code(channels);
  command.selector_code = select_selector_code(channels);

  command.velocity.x() = axis_value(channels, linear_x_);
  command.velocity.y() = axis_value(channels, linear_y_);
  command.velocity.z() = axis_value(channels, angular_z_);

  return command;
}

// Realtime tick freshness
void RadiomasterPocketTeleopInputPlugin::on_stale_message(
  const RcStatus & msg) const
{
  const auto realtime_tick = msg.realtime_tick;

  if (!realtime_tick::is_valid(realtime_tick)) {
    log_tick_above_max(realtime_tick);
    return;
  }

  log_stale_tick(realtime_tick);
}

void RadiomasterPocketTeleopInputPlugin::on_message_accepted(
  const RcStatus & msg)
{
  has_last_realtime_tick_ = true;
  last_realtime_tick_ = msg.realtime_tick;
}

bool RadiomasterPocketTeleopInputPlugin::is_fresh_tick(
  uint32_t realtime_tick) const
{
  return !has_last_realtime_tick_ ||
         realtime_tick::is_newer_than_last(realtime_tick, last_realtime_tick_);
}

void RadiomasterPocketTeleopInputPlugin::log_tick_above_max(
  uint32_t realtime_tick) const
{
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "[%s] ignored RC status with realtime tick above expected rollover max "
    "(topic=%s, current=%u, max=%u)",
    name().c_str(),
    topic_.c_str(),
    static_cast<unsigned>(realtime_tick),
    static_cast<unsigned>(realtime_tick::kMax));
}

void RadiomasterPocketTeleopInputPlugin::log_stale_tick(
  uint32_t realtime_tick) const
{
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "[%s] ignored RC status with non-advancing realtime tick "
    "(topic=%s, current=%u, last=%u, max=%u)",
    name().c_str(),
    topic_.c_str(),
    static_cast<unsigned>(realtime_tick),
    static_cast<unsigned>(last_realtime_tick_),
    static_cast<unsigned>(realtime_tick::kMax));
}

// Configuration
void RadiomasterPocketTeleopInputPlugin::read_config(const YAML::Node & config)
{
  if (!config || config.IsNull()) {
    throw std::runtime_error("Radiomaster Pocket teleop input config is required");
  }
  const auto topic = config["topic"];
  if (!topic || topic.IsNull()) {
    throw std::runtime_error("Radiomaster Pocket teleop input config requires topic");
  }

  topic_ = topic.as<std::string>();
  axis_deadzone_ = read_double(config, "axis_deadzone", axis_deadzone_);
  switch_match_tolerance_ = read_double(config, "switch_match_tolerance", switch_match_tolerance_);

  if (axis_deadzone_ < 0.0 || axis_deadzone_ >= 1.0) {
    throw std::runtime_error("axis_deadzone must be in [0, 1)");
  }
  if (switch_match_tolerance_ <= 0.0) {
    throw std::runtime_error("switch_match_tolerance must be positive");
  }

  read_velocity_command_config(config);

  api_mode_conditions_ = read_optional_rc_conditions(config["api_mode"]);
  input_codes_ = read_input_code_configs(config["input_code"]);
  selector_code_ = read_selector_code_config(config["selector_code"]);

  update_always_required_channels();
}

void RadiomasterPocketTeleopInputPlugin::read_velocity_command_config(
  const YAML::Node & config)
{
  const auto velocity_command = config["velocity_command"];

  if (!velocity_command || velocity_command.IsNull()) {
    throw std::runtime_error(
      "Radiomaster Pocket teleop input config requires velocity_command");
  }

  const auto channels = velocity_command["channels"];
  if (!channels || channels.IsNull()) {
    throw std::runtime_error(
      "Radiomaster Pocket teleop input config requires velocity_command.channels");
  }

  linear_x_ = read_axis_config(channels, "linear_x");
  linear_y_ = read_axis_config(channels, "linear_y");
  angular_z_ = read_axis_config(channels, "angular_z");
}

void RadiomasterPocketTeleopInputPlugin::update_always_required_channels()
{
  always_required_channels_.clear();

  add_always_required_channel(linear_x_.channel);
  add_always_required_channel(linear_y_.channel);
  add_always_required_channel(angular_z_.channel);

  for (const auto & condition : api_mode_conditions_) {
    add_always_required_channel(condition.channel);
  }

  for (const auto & input_code : input_codes_) {
    for (const auto & condition : input_code.conditions) {
      add_always_required_channel(condition.channel);
    }
  }

  add_always_required_channel(selector_code_.channel);
}

void RadiomasterPocketTeleopInputPlugin::add_always_required_channel(uint8_t channel)
{
  if (channel == 0U) {
    return;
  }

  if (contains_value(always_required_channels_, channel)) {
    return;
  }

  always_required_channels_.push_back(channel);
}

RadiomasterPocketTeleopInputPlugin::AxisConfig
RadiomasterPocketTeleopInputPlugin::read_axis_config(
  const YAML::Node & channels,
  const std::string & name)
{
  if (!channels || channels.IsNull()) {
    throw std::runtime_error("velocity_command.channels is required");
  }

  const auto axis = channels[name];
  if (!axis || axis.IsNull()) {
    throw std::runtime_error("velocity_command.channels." + name + " is required");
  }
  const auto channel = axis["channel"];
  if (!channel || channel.IsNull()) {
    throw std::runtime_error("velocity_command.channels." + name + ".channel is required");
  }

  return AxisConfig{
    read_channel_id(channel, "velocity_command.channels." + name + ".channel"),
    read_bool(axis, "invert", false)};
}

std::vector<RadiomasterPocketTeleopInputPlugin::RcCondition>
RadiomasterPocketTeleopInputPlugin::read_optional_rc_conditions(const YAML::Node & node)
{
  std::vector<RcCondition> conditions;
  if (!node || node.IsNull()) {
    return conditions;
  }

  const auto condition_map = node["when"] ? node["when"] : node;
  for (const auto & rc_item : condition_map) {
    conditions.push_back({parse_channel_id(rc_item.first.as<std::string>(), "api_mode.when"),
        rc_item.second.as<double>()});
  }

  return conditions;
}

std::vector<RadiomasterPocketTeleopInputPlugin::InputCodeConfig>
RadiomasterPocketTeleopInputPlugin::read_input_code_configs(const YAML::Node & input_code)
{
  if (!input_code || input_code.IsNull()) {
    throw std::runtime_error("input_code is required");
  }

  const auto channels = input_code["channels"];
  if (!channels || channels.IsNull()) {
    throw std::runtime_error("input_code.channels is required");
  }
  if (!channels.IsMap()) {
    throw std::runtime_error("input_code.channels must be a map");
  }

  std::vector<InputCodeConfig> input_codes;
  read_input_code_channel_map(channels, {}, input_codes);

  if (input_codes.empty()) {
    throw std::runtime_error("input_code.channels must select at least one input code");
  }

  return input_codes;
}

void RadiomasterPocketTeleopInputPlugin::read_input_code_channel_map(
  const YAML::Node & channels, const std::vector<RcCondition> & parent_conditions,
  std::vector<InputCodeConfig> & input_codes)
{
  for (const auto & channel_item : channels) {
    const auto channel_name = channel_item.first.as<std::string>();
    const auto channel = parse_channel_id(channel_name, "input_code");
    const auto positions = channel_item.second;
    const std::string channel_path = "input_code.channels." + channel_name;

    if (!positions || positions.IsNull()) {
      throw std::runtime_error(channel_path + " is required");
    }
    if (!positions.IsMap()) {
      throw std::runtime_error(channel_path + " must be a map");
    }

    for (const auto & position_item : positions) {
      auto conditions = parent_conditions;
      conditions.push_back({channel, position_item.first.as<double>()});

      const auto branch = position_item.second;

      if (branch.IsScalar()) {
        input_codes.push_back({conditions, branch.as<uint16_t>()});
        continue;
      }

      if (branch.IsMap()) {
        read_input_code_channel_map(branch, conditions, input_codes);
        continue;
      }

      throw std::runtime_error(
        "input_code.channels entries must select an input code or nested channel");
    }
  }
}

// Selector code config helpers
namespace
{

using SelectorCodeTable = std::vector<std::pair<double, uint16_t>>;

void require_selector_code_config(const YAML::Node & selector_code)
{
  const auto channel = selector_code["channel"];
  if (!channel || channel.IsNull()) {
    throw std::runtime_error("selector_code.channel is required");
  }

  const auto table = selector_code["table"];
  if (!table || table.IsNull()) {
    throw std::runtime_error("selector_code.table is required");
  }
  if (!table.IsMap()) {
    throw std::runtime_error("selector_code.table must be a map");
  }
}

double read_selector_code_tolerance(const YAML::Node & selector_code, double fallback)
{
  const auto tolerance = read_double(selector_code, "tolerance", fallback);
  if (tolerance <= 0.0) {
    throw std::runtime_error("selector_code.tolerance must be positive");
  }

  return tolerance;
}

std::vector<uint16_t> read_required_selector_input_codes(const YAML::Node & selector_code)
{
  if (!selector_code["required_for_input_codes"]) {
    return {};
  }

  return selector_code["required_for_input_codes"].as<std::vector<uint16_t>>();
}

SelectorCodeTable read_selector_code_table(const YAML::Node & table)
{
  SelectorCodeTable selector_codes;
  for (const auto & table_item : table) {
    selector_codes.emplace_back(table_item.first.as<double>(), table_item.second.as<uint16_t>());
  }

  return selector_codes;
}

void sort_selector_code_table(SelectorCodeTable & table)
{
  std::sort(
    table.begin(),
    table.end(),
    [](const auto & lhs, const auto & rhs) {
      return lhs.first < rhs.first;
    });
}

void require_selector_code_table_spacing(const SelectorCodeTable & table, double tolerance)
{
  for (std::size_t i = 1; i < table.size(); ++i) {
    const auto previous_value = table[i - 1].first;
    const auto current_value = table[i].first;

    if (current_value == previous_value) {
      throw std::runtime_error(
        "selector_code.table has duplicate value " + std::to_string(current_value));
    }

    const double spacing = current_value - previous_value;
    const double minimum_spacing = 2.0 * tolerance;
    if (spacing <= minimum_spacing) {
      throw std::runtime_error(
        "selector_code.table values " + std::to_string(previous_value) + " and " +
        std::to_string(current_value) + " overlap within selector_code.tolerance " +
        std::to_string(tolerance));
    }
  }
}

}  // namespace

RadiomasterPocketTeleopInputPlugin::SelectorCodeConfig
RadiomasterPocketTeleopInputPlugin::read_selector_code_config(const YAML::Node & selector_code)
{
  SelectorCodeConfig config;

  if (!selector_code) {
    return config;
  }

  require_selector_code_config(selector_code);

  config.channel = read_channel_id(selector_code["channel"], "selector_code.channel");
  config.tolerance = read_selector_code_tolerance(selector_code, config.tolerance);
  config.required_for_input_codes = read_required_selector_input_codes(selector_code);
  config.table = read_selector_code_table(selector_code["table"]);

  sort_selector_code_table(config.table);
  require_selector_code_table_spacing(config.table, config.tolerance);

  return config;
}

uint8_t RadiomasterPocketTeleopInputPlugin::read_channel_id(
  const YAML::Node & node,
  const std::string & path)
{
  if (!node || node.IsNull()) {
    throw std::runtime_error(path + " is required");
  }
  if (!node.IsScalar()) {
    throw std::runtime_error(path + " must be a scalar channel id");
  }

  return parse_channel_id(node.as<std::string>(), path);
}

uint8_t RadiomasterPocketTeleopInputPlugin::parse_channel_id(
  const std::string & value,
  const std::string & path)
{
  std::string digits;

  for (const auto ch : value) {
    if (std::isdigit(static_cast<unsigned char>(ch))) {
      digits.push_back(ch);
    }
  }

  if (digits.empty()) {
    throw std::runtime_error(path + " channel '" + value + "' does not contain a channel number");
  }

  const auto channel = std::stoi(digits);

  if (channel < 1 || channel > 16) {
    throw std::runtime_error(path + " channel '" + value + "' must resolve to 1..16");
  }

  return static_cast<uint8_t>(channel);
}

// RC status validation
RadiomasterPocketTeleopInputPlugin::ChannelLookup
RadiomasterPocketTeleopInputPlugin::make_channel_lookup(
  const RcStatus & msg) const
{
  ChannelLookup channels{};
  channels.fill(nullptr);

  for (const auto & channel : msg.channels) {
    if (channel.rc_channel >= 1U && channel.rc_channel < channels.size()) {
      channels[channel.rc_channel] = &channel;
    }
  }

  return channels;
}

bool RadiomasterPocketTeleopInputPlugin::is_status_health_ok(const RcStatus & msg) const
{
  const bool has_healthy_status = msg.status_data_valid && msg.is_control_input_safe;
  if (has_healthy_status) {
    return true;
  }

  const auto * status_valid = msg.status_data_valid ? "true" : "false";
  const auto * tick_fresh = msg.realtime_tick_fresh ? "true" : "false";
  const auto * channels_valid = msg.all_channels_valid ? "true" : "false";
  const auto * hardware_ok = msg.hardware_ok ? "true" : "false";
  const auto * estop_released = msg.estop_released ? "true" : "false";
  const auto * rc_link_ok = msg.rc_link_ok ? "true" : "false";
  const auto * control_safe = msg.is_control_input_safe ? "true" : "false";

  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "[%s] ignored RC status: status_valid=%s tick_fresh=%s channels_valid=%s "
    "hardware_ok=%s estop_released=%s rc_link_ok=%s control_safe=%s "
    "crsf_age=%ums crsf_failsafe=%u crsf_lq=%u%% (topic=%s)",
    name().c_str(),
    status_valid,
    tick_fresh,
    channels_valid,
    hardware_ok,
    estop_released,
    rc_link_ok,
    control_safe,
    static_cast<unsigned>(msg.crsf_last_frame_age_ms),
    static_cast<unsigned>(msg.crsf_failsafe),
    static_cast<unsigned>(msg.crsf_link_quality),
    topic_.c_str());

  return false;
}

bool RadiomasterPocketTeleopInputPlugin::are_channel_values_valid(
  const RcStatus & msg) const
{
  const auto channels = make_channel_lookup(msg);

  return are_required_channels_readable(channels) &&
         is_input_code_decodable(channels) &&
         is_selector_code_decodable_if_needed(channels);
}

bool RadiomasterPocketTeleopInputPlugin::are_required_channels_readable(
  const ChannelLookup & channels) const
{
  for (const auto channel : always_required_channels_) {
    if (read_channel(channels, channel) == nullptr) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[%s] waiting for RC channel %u in status topic (%s)",
        name().c_str(),
        static_cast<unsigned>(channel),
        topic_.c_str());

      return false;
    }

    if (!has_readable_channel(channels, channel)) {
      return false;
    }
  }

  return true;
}

bool RadiomasterPocketTeleopInputPlugin::has_readable_channel(
  const ChannelLookup & channels,
  uint8_t channel) const
{
  const auto rc_channel = read_channel(channels, channel);

  if (rc_channel == nullptr) {
    return false;
  }

  if (!rc_channel->valid) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[%s] ignored RC status: channel %u has no valid HAT sample (topic=%s)",
      name().c_str(),
      static_cast<unsigned>(channel),
      topic_.c_str());
    return false;
  }

  if (!is_normalized_axis(rc_channel->axis)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[%s] ignored RC status: channel %u axis is not normalized "
      "(axis=%f, expected=[-1, 1], topic=%s)",
      name().c_str(),
      static_cast<unsigned>(channel),
      rc_channel->axis,
      topic_.c_str());
    return false;
  }

  return true;
}

const RadiomasterPocketTeleopInputPlugin::RcChannel *
RadiomasterPocketTeleopInputPlugin::read_channel(
  const ChannelLookup & channels,
  uint8_t channel) const
{
  if (!has_configured_channel(channel) || channel >= channels.size()) {
    return nullptr;
  }

  return channels[channel];
}

// Command decoding
float RadiomasterPocketTeleopInputPlugin::axis_value(
  const ChannelLookup & channels,
  const AxisConfig & axis) const
{
  const auto channel = read_channel(channels, axis.channel);

  if (channel == nullptr || !std::isfinite(channel->axis)) {
    return 0.0F;
  }

  auto value = std::clamp(channel->axis, -1.0F, 1.0F);

  if (std::abs(value) <= axis_deadzone_) {
    value = 0.0F;
  }

  if (axis.invert) {
    value *= -1.0F;
  }

  return value;
}

bool RadiomasterPocketTeleopInputPlugin::are_rc_conditions_satisfied(
  const ChannelLookup & channels,
  const std::vector<RcCondition> & conditions) const
{
  for (const auto & condition : conditions) {
    if (!is_rc_condition_satisfied(channels, condition)) {
      return false;
    }
  }

  return true;
}

bool RadiomasterPocketTeleopInputPlugin::is_rc_condition_satisfied(
  const ChannelLookup & channels,
  const RcCondition & condition) const
{
  const auto channel = read_channel(channels, condition.channel);

  if (channel == nullptr) {
    return false;
  }

  return is_rc_us_within_tolerance(channel->rc_us, condition.value, switch_match_tolerance_);
}

bool RadiomasterPocketTeleopInputPlugin::is_input_code_decodable(
  const ChannelLookup & channels) const
{
  for (const auto & input_code : input_codes_) {
    bool prefix_matches = true;

    for (const auto & condition : input_code.conditions) {
      if (prefix_matches && !has_readable_channel(channels, condition.channel)) {
        return false;
      }

      if (!is_rc_condition_satisfied(channels, condition)) {
        prefix_matches = false;
        break;
      }
    }
  }

  return true;
}

bool RadiomasterPocketTeleopInputPlugin::is_selector_code_decodable_if_needed(
  const ChannelLookup & channels) const
{
  if (!has_configured_channel(selector_code_.channel)) {
    return true;
  }

  const auto input_code = select_input_code(channels);

  if (!does_input_code_require_selector_code(input_code)) {
    return true;
  }

  return has_readable_channel(channels, selector_code_.channel);
}

bool RadiomasterPocketTeleopInputPlugin::does_input_code_require_selector_code(
  uint16_t input_code) const
{
  return contains_value(selector_code_.required_for_input_codes, input_code);
}

bool RadiomasterPocketTeleopInputPlugin::is_api_mode_requested(
  const ChannelLookup & channels) const
{
  return !api_mode_conditions_.empty() &&
         are_rc_conditions_satisfied(channels, api_mode_conditions_);
}

uint16_t RadiomasterPocketTeleopInputPlugin::select_input_code(
  const ChannelLookup & channels) const
{
  for (const auto & input_code : input_codes_) {
    if (are_rc_conditions_satisfied(channels, input_code.conditions)) {
      return input_code.code;
    }
  }

  return 0;
}

uint16_t RadiomasterPocketTeleopInputPlugin::select_selector_code(
  const ChannelLookup & channels) const
{
  if (!has_configured_channel(selector_code_.channel)) {
    return 0;
  }

  const auto channel = read_channel(channels, selector_code_.channel);

  if (channel == nullptr) {
    return 0;
  }

  for (const auto & item : selector_code_.table) {
    if (is_rc_us_within_tolerance(channel->rc_us, item.first, selector_code_.tolerance)) {
      return item.second;
    }
  }

  return 0;
}

}  // namespace ai_sapiens_sim2real

PLUGINLIB_EXPORT_CLASS(
  ai_sapiens_sim2real::RadiomasterPocketTeleopInputPlugin,
  ai_sapiens_sim2real::TeleopInputPluginBase)
