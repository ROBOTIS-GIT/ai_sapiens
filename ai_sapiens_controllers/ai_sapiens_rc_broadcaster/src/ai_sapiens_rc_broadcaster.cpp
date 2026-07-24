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
// Author: Eunsung Kim

#include "ai_sapiens_rc_broadcaster/ai_sapiens_rc_broadcaster.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"

namespace ai_sapiens_rc_broadcaster
{

namespace
{

using RcStatus = ai_sapiens_interfaces::msg::RcStatus;

enum StatusIndex : size_t
{
  kHardwareErrorStatus = 0,
  kRealtimeTick,
  kEstopActive,
  kCrsfLastFrameAge,
  kCrsfFailsafe,
  kCrsfRssi1,
  kCrsfLinkQuality,
};

struct StatusValues
{
  bool status_data_valid{false};
  bool realtime_tick_fresh{false};
  bool all_channels_valid{false};
  bool hardware_ok{false};
  bool estop_released{false};
  bool rc_link_ok{false};
  bool is_control_input_safe{false};

  uint8_t hardware_error_status{255};
  uint32_t realtime_tick{0};
  bool estop_active{true};
  uint16_t crsf_last_frame_age_ms{65535};
  uint8_t crsf_failsafe{1};
  uint8_t crsf_rssi_1{0};
  uint8_t crsf_link_quality{0};
};

constexpr std::array<const char *, 7> kStatusItems{{
  "Hardware Error Status",
  "Realtime Tick",
  "E-stop Active",
  "CRSF Last Frame Age",
  "CRSF Failsafe",
  "CRSF RSSI 1",
  "CRSF Link Quality",
}};

constexpr size_t kNoStateInterface = std::numeric_limits<size_t>::max();

size_t find_status_item_index(const std::string & name)
{
  for (size_t i = 0; i < kStatusItems.size(); ++i) {
    if (name == kStatusItems[i]) {
      return i;
    }
  }
  return kNoStateInterface;
}

uint16_t to_u16(double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return 0U;
  }
  return static_cast<uint16_t>(
    std::clamp<double>(std::llround(value), 0.0, 65535.0));
}

uint32_t to_u32(double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return 0U;
  }
  return static_cast<uint32_t>(
    std::clamp<double>(std::llround(value), 0.0, 4294967295.0));
}

uint8_t to_u8(double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return 0U;
  }
  return static_cast<uint8_t>(
    std::clamp<double>(std::llround(value), 0.0, 255.0));
}

rclcpp::Duration publish_period_from_rate(double rate_hz)
{
  if (rate_hz <= 0.0) {
    return rclcpp::Duration(0, 0);
  }
  const auto period_ns =
    static_cast<int64_t>(std::llround(1000000000.0 / rate_hz));
  return rclcpp::Duration::from_nanoseconds(std::max<int64_t>(period_ns, 1));
}

int64_t add_elapsed_time(int64_t elapsed_ns, int64_t period_ns)
{
  if (period_ns <= 0) {
    return elapsed_ns;
  }
  if (elapsed_ns > std::numeric_limits<int64_t>::max() - period_ns) {
    return std::numeric_limits<int64_t>::max();
  }
  return elapsed_ns + period_ns;
}

bool is_finite(double value)
{
  return std::isfinite(value);
}

}  // namespace

controller_interface::CallbackReturn AiSapiensRcBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Failed to initialize RC broadcaster parameters: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AiSapiensRcBroadcaster::read_parameters()
{
  try {
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  sensor_name_ = params_.sensor_name;
  input_topic_name_ =
    params_.input_topic_name.empty() ? params_.topic_name : params_.input_topic_name;
  status_topic_name_ =
    params_.status_topic_name.empty() ? params_.state_topic_name : params_.status_topic_name;
  frame_id_ = params_.frame_id;
  channels_ = params_.channels;
  channel_names_ = params_.channel_names;
  status_interfaces_ = params_.status_interfaces;
  normalize_axes_ = params_.normalize_axes;
  deadzone_ = params_.deadzone;
  publish_buttons_ = params_.publish_buttons;
  button_threshold_ = params_.button_threshold;
  input_publish_rate_ = params_.input_publish_rate;
  status_publish_rate_ = params_.status_publish_rate;
  watchdog_crsf_frame_timeout_ms_ = params_.watchdog_crsf_frame_timeout_ms;
  watchdog_realtime_tick_timeout_ms_ = params_.watchdog_realtime_tick_timeout_ms;
  min_crsf_link_quality_ = params_.min_crsf_link_quality;

  const std::array<double, 7> finite_parameters{{
    params_.publish_rate,
    input_publish_rate_,
    status_publish_rate_,
    deadzone_,
    button_threshold_,
    watchdog_crsf_frame_timeout_ms_,
    watchdog_realtime_tick_timeout_ms_,
  }};
  if (
    !std::all_of(finite_parameters.begin(), finite_parameters.end(), is_finite) ||
    !is_finite(min_crsf_link_quality_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "All numeric parameters must be finite");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (input_topic_name_.empty() || status_topic_name_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "input_topic_name and status_topic_name must be non-empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (
    (params_.publish_rate < 0.0 && params_.publish_rate != -1.0) ||
    input_publish_rate_ < 0.0 || status_publish_rate_ < 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Publish rates must be non-negative; legacy publish_rate may be -1.0");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (deadzone_ < 0.0 || deadzone_ > 500.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "deadzone must be in [0, 500]");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (button_threshold_ < 0.0 || button_threshold_ > 2500.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "button_threshold must be in [0, 2500]");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (
    watchdog_crsf_frame_timeout_ms_ <= 0.0 ||
    watchdog_realtime_tick_timeout_ms_ <= 0.0 ||
    min_crsf_link_quality_ < 0.0 || min_crsf_link_quality_ > 100.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Watchdog timeouts must be positive and min_crsf_link_quality must be in [0, 100]");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.publish_rate >= 0.0) {
    input_publish_rate_ = params_.publish_rate;
    status_publish_rate_ = params_.publish_rate;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AiSapiensRcBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
AiSapiensRcBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}

controller_interface::CallbackReturn AiSapiensRcBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto parameter_result = read_parameters();
  if (parameter_result != controller_interface::CallbackReturn::SUCCESS) {
    return parameter_result;
  }

  state_interface_names_.clear();
  state_interface_names_.reserve(channels_.size() + status_interfaces_.size());
  status_interface_indices_by_item_.assign(kStatusItems.size(), kNoStateInterface);
  for (size_t i = 0; i < channels_.size(); ++i) {
    const auto & channel = channels_[i];
    state_interface_names_.push_back(sensor_name_ + "/" + channel);
  }
  for (const auto & status_interface : status_interfaces_) {
    const auto status_index = find_status_item_index(status_interface);
    if (status_index == kNoStateInterface) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unknown RC status interface '%s'", status_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (status_interface_indices_by_item_[status_index] != kNoStateInterface) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Duplicate RC status interface '%s'", status_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    status_interface_indices_by_item_[status_index] = state_interface_names_.size();
    state_interface_names_.push_back(sensor_name_ + "/" + status_interface);
  }
  for (size_t status_index = 0; status_index < kStatusItems.size(); ++status_index) {
    if (status_interface_indices_by_item_[status_index] == kNoStateInterface) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Missing required K1 HAT status interface '%s'",
        kStatusItems[status_index]);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  input_publish_period_ = publish_period_from_rate(input_publish_rate_);
  status_publish_period_ = publish_period_from_rate(status_publish_rate_);

  try {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    input_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Joy>(input_topic_name_, qos);
    realtime_input_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::Joy>>(input_publisher_);
    status_publisher_ = get_node()->create_publisher<RcStatus>(status_topic_name_, qos);
    realtime_status_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<RcStatus>>(status_publisher_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create publishers: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_input_publisher_->lock();
  auto & input_msg = realtime_input_publisher_->msg_;
  input_msg.header.frame_id = frame_id_;
  input_msg.axes.assign(channels_.size(), 0.0F);
  input_msg.buttons.assign(publish_buttons_ ? channels_.size() : 0U, 0);
  realtime_input_publisher_->unlock();

  realtime_status_publisher_->lock();
  auto & status_msg = realtime_status_publisher_->msg_;
  status_msg.header.frame_id = frame_id_;
  status_msg.sensor_name = sensor_name_;
  status_msg.input_topic_name = input_topic_name_;
  status_msg.status_topic_name = status_topic_name_;
  status_msg.hardware_error_status = 255U;
  status_msg.realtime_tick = 0U;
  status_msg.estop_active = true;
  status_msg.crsf_last_frame_age_ms = 65535U;
  status_msg.crsf_failsafe = 1U;
  status_msg.crsf_link_quality = 0U;
  status_msg.crsf_rssi_1 = 0U;
  status_msg.status_data_valid = false;
  status_msg.realtime_tick_fresh = false;
  status_msg.all_channels_valid = false;
  status_msg.hardware_ok = false;
  status_msg.estop_released = false;
  status_msg.rc_link_ok = false;
  status_msg.is_control_input_safe = false;
  status_msg.channels.resize(channels_.size());
  for (size_t i = 0; i < channels_.size(); ++i) {
    auto & channel = status_msg.channels[i];
    channel.rc_channel = static_cast<uint8_t>(i + 1U);
    channel.name = channel_names_[i];
    channel.valid = false;
    channel.rc_us = 1500U;
    channel.axis = 0.0F;
    channel.button = 0;
    channel.axis_in_deadzone = true;
    channel.button_active = false;
  }
  realtime_status_publisher_->unlock();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured RC broadcaster: sensor=%s input_topic=%s status_topic=%s "
    "channels=%zu status_items=%zu publish_rates={input: %.1f Hz, status: %.1f Hz}",
    sensor_name_.c_str(), input_topic_name_.c_str(), status_topic_name_.c_str(),
    channels_.size(), kStatusItems.size(), input_publish_rate_, status_publish_rate_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AiSapiensRcBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  ordered_state_interfaces_.clear();
  if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, state_interface_names_, "", ordered_state_interfaces_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Could not match all %zu required state interfaces to the %zu assigned interfaces",
      state_interface_names_.size(), state_interfaces_.size());
    ordered_state_interfaces_.clear();
    return controller_interface::CallbackReturn::ERROR;
  }

  input_publish_elapsed_ns_ = input_publish_period_.nanoseconds();
  status_publish_elapsed_ns_ = status_publish_period_.nanoseconds();
  realtime_tick_age_ns_ = 0;
  last_realtime_tick_ = 0U;
  has_last_realtime_tick_ = false;
  active_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AiSapiensRcBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  ordered_state_interfaces_.clear();
  has_last_realtime_tick_ = false;
  realtime_tick_age_ns_ = 0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AiSapiensRcBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  ordered_state_interfaces_.clear();
  state_interface_names_.clear();
  status_interface_indices_by_item_.clear();
  realtime_input_publisher_.reset();
  realtime_status_publisher_.reset();
  input_publisher_.reset();
  status_publisher_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AiSapiensRcBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (
    !active_ || !realtime_input_publisher_ || !realtime_status_publisher_)
  {
    return controller_interface::return_type::OK;
  }

  const auto read_interface = [this](size_t index) -> std::optional<double> {
      if (index >= ordered_state_interfaces_.size()) {
        return std::nullopt;
      }
      const auto value = ordered_state_interfaces_[index].get().get_optional(0);
      if (!value.has_value() || !std::isfinite(value.value())) {
        return std::nullopt;
      }
      return value.value();
    };
  const auto read_status = [this, &read_interface](StatusIndex index) -> std::optional<double> {
      const auto status_index = static_cast<size_t>(index);
      if (status_index >= status_interface_indices_by_item_.size()) {
        return std::nullopt;
      }
      const auto interface_index = status_interface_indices_by_item_[status_index];
      if (interface_index == kNoStateInterface) {
        return std::nullopt;
      }
      return read_interface(interface_index);
    };

  const auto elapsed_ns = std::max<int64_t>(period.nanoseconds(), 0);
  input_publish_elapsed_ns_ = add_elapsed_time(input_publish_elapsed_ns_, elapsed_ns);
  status_publish_elapsed_ns_ = add_elapsed_time(status_publish_elapsed_ns_, elapsed_ns);
  if (has_last_realtime_tick_) {
    realtime_tick_age_ns_ = add_elapsed_time(realtime_tick_age_ns_, elapsed_ns);
  }

  std::array<std::optional<double>, 16> channel_samples{};
  for (size_t i = 0; i < channels_.size() && i < channel_samples.size(); ++i) {
    channel_samples[i] = read_interface(i);
    if (
      channel_samples[i].has_value() &&
      (channel_samples[i].value() < 0.0 || channel_samples[i].value() > 2500.0))
    {
      channel_samples[i].reset();
    }
  }

  const auto hardware_error_sample = read_status(kHardwareErrorStatus);
  const auto realtime_tick_sample = read_status(kRealtimeTick);
  const auto estop_sample = read_status(kEstopActive);
  const auto crsf_last_frame_age_sample = read_status(kCrsfLastFrameAge);
  const auto crsf_failsafe_sample = read_status(kCrsfFailsafe);
  const auto crsf_rssi_1_sample = read_status(kCrsfRssi1);
  const auto crsf_link_quality_sample = read_status(kCrsfLinkQuality);
  const auto sample_in_range = [](const auto & sample, double minimum, double maximum) {
      return sample.has_value() &&
             sample.value() >= minimum && sample.value() <= maximum;
    };

  StatusValues status{};
  status.status_data_valid =
    sample_in_range(hardware_error_sample, 0.0, 255.0) &&
    sample_in_range(realtime_tick_sample, 0.0, 32767.0) &&
    sample_in_range(estop_sample, 0.0, 1.0) &&
    sample_in_range(crsf_last_frame_age_sample, 0.0, 65535.0) &&
    sample_in_range(crsf_failsafe_sample, 0.0, 1.0) &&
    sample_in_range(crsf_rssi_1_sample, 0.0, 255.0) &&
    sample_in_range(crsf_link_quality_sample, 0.0, 100.0);
  status.all_channels_valid = std::all_of(
    channel_samples.begin(), channel_samples.end(),
    [](const auto & sample) {return sample.has_value();});

  if (hardware_error_sample.has_value()) {
    status.hardware_error_status = to_u8(hardware_error_sample.value());
  }
  if (realtime_tick_sample.has_value()) {
    status.realtime_tick = to_u32(realtime_tick_sample.value());
  }
  if (estop_sample.has_value()) {
    status.estop_active = estop_sample.value() != 0.0;
  }
  if (crsf_last_frame_age_sample.has_value()) {
    status.crsf_last_frame_age_ms = to_u16(crsf_last_frame_age_sample.value());
  }
  if (crsf_failsafe_sample.has_value()) {
    status.crsf_failsafe = to_u8(crsf_failsafe_sample.value());
  }
  if (crsf_rssi_1_sample.has_value()) {
    status.crsf_rssi_1 = to_u8(crsf_rssi_1_sample.value());
  }
  if (crsf_link_quality_sample.has_value()) {
    status.crsf_link_quality = to_u8(crsf_link_quality_sample.value());
  }

  if (realtime_tick_sample.has_value()) {
    const auto current_tick = status.realtime_tick;
    if (!has_last_realtime_tick_ || current_tick != last_realtime_tick_) {
      last_realtime_tick_ = current_tick;
      realtime_tick_age_ns_ = 0;
      has_last_realtime_tick_ = true;
    }
  }
  if (has_last_realtime_tick_) {
    const auto tick_age_ms = static_cast<double>(realtime_tick_age_ns_) / 1000000.0;
    status.realtime_tick_fresh =
      tick_age_ms <= watchdog_realtime_tick_timeout_ms_;
  }

  status.hardware_ok = status.status_data_valid && status.hardware_error_status == 0U;
  status.estop_released = status.status_data_valid && !status.estop_active;
  status.rc_link_ok =
    status.status_data_valid &&
    status.realtime_tick_fresh &&
    status.crsf_failsafe == 0U &&
    status.crsf_last_frame_age_ms <= watchdog_crsf_frame_timeout_ms_ &&
    status.crsf_link_quality >= min_crsf_link_quality_;
  status.is_control_input_safe =
    status.hardware_ok &&
    status.estop_released &&
    status.rc_link_ok &&
    status.all_channels_valid;

  const bool input_due =
    input_publish_period_.nanoseconds() == 0 ||
    input_publish_elapsed_ns_ >= input_publish_period_.nanoseconds();
  const bool status_due =
    status_publish_period_.nanoseconds() == 0 ||
    status_publish_elapsed_ns_ >= status_publish_period_.nanoseconds();

  if (input_due && realtime_input_publisher_->trylock()) {
    auto & input_msg = realtime_input_publisher_->msg_;
    input_msg.header.stamp = time;

    for (size_t i = 0; i < channels_.size(); ++i) {
      const double pwm = channel_samples[i].value_or(1500.0);
      input_msg.axes[i] = normalize_axes_ ? normalize_pwm(pwm, deadzone_) : static_cast<float>(pwm);
      if (publish_buttons_) {
        input_msg.buttons[i] = pwm >= button_threshold_ ? 1 : 0;
      }
    }

    realtime_input_publisher_->unlockAndPublish();
    input_publish_elapsed_ns_ = 0;
  }

  if (status_due && realtime_status_publisher_->trylock()) {
    auto & status_msg = realtime_status_publisher_->msg_;
    status_msg.header.stamp = time;

    for (size_t i = 0; i < channels_.size(); ++i) {
      const double pwm = channel_samples[i].value_or(1500.0);
      auto & channel = status_msg.channels[i];
      channel.valid = channel_samples[i].has_value();
      channel.rc_us = to_u16(pwm);
      channel.axis = normalize_axes_ ? normalize_pwm(pwm, deadzone_) : static_cast<float>(pwm);
      channel.button = publish_buttons_ && pwm >= button_threshold_ ? 1 : 0;
      channel.axis_in_deadzone = is_axis_in_deadzone(pwm, deadzone_);
      channel.button_active = channel.button != 0;
    }

    status_msg.hardware_error_status = status.hardware_error_status;
    status_msg.realtime_tick = status.realtime_tick;
    status_msg.estop_active = status.estop_active;
    status_msg.crsf_last_frame_age_ms = status.crsf_last_frame_age_ms;
    status_msg.crsf_failsafe = status.crsf_failsafe;
    status_msg.crsf_link_quality = status.crsf_link_quality;
    status_msg.crsf_rssi_1 = status.crsf_rssi_1;
    status_msg.status_data_valid = status.status_data_valid;
    status_msg.realtime_tick_fresh = status.realtime_tick_fresh;
    status_msg.all_channels_valid = status.all_channels_valid;
    status_msg.hardware_ok = status.hardware_ok;
    status_msg.estop_released = status.estop_released;
    status_msg.rc_link_ok = status.rc_link_ok;
    status_msg.is_control_input_safe = status.is_control_input_safe;

    realtime_status_publisher_->unlockAndPublish();
    status_publish_elapsed_ns_ = 0;
  }

  return controller_interface::return_type::OK;
}

float AiSapiensRcBroadcaster::normalize_pwm(double pwm, double deadzone)
{
  const double centered = pwm - 1500.0;
  if (std::abs(centered) < deadzone) {
    return 0.0F;
  }
  const double normalized = std::clamp(centered / 500.0, -1.0, 1.0);
  return static_cast<float>(normalized);
}

bool AiSapiensRcBroadcaster::is_axis_in_deadzone(double pwm, double deadzone)
{
  return std::abs(pwm - 1500.0) < deadzone;
}

}  // namespace ai_sapiens_rc_broadcaster

PLUGINLIB_EXPORT_CLASS(
  ai_sapiens_rc_broadcaster::AiSapiensRcBroadcaster,
  controller_interface::ControllerInterface)
