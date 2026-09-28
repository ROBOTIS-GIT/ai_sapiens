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

#include "radiomaster_usb_hardware_interface/radiomaster_usb_hardware_interface.hpp"

#include <fcntl.h>
#include <linux/joystick.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"

namespace radiomaster_usb_hardware_interface
{
namespace
{

constexpr const char * kDefaultRcChannels =
  "1500,1500,1500,1500,1000,1500,1000,1000,1000,1000,1000,1000,1500,1500,1500,1500";
constexpr double kEdgeTxHidCenter = 1024.0;
constexpr double kJoydevCorrectionShift = 16384.0;

constexpr const char * kIndividualChannelPrefix = "RC Channel ";
constexpr const char * kGroupChannelPrefix = "Remote 2 CH";

std::string get_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  const std::string & fallback)
{
  const auto item = parameters.find(name);
  return item == parameters.end() ? fallback : item->second;
}

bool parse_bool(const std::string & value)
{
  return value == "true" || value == "True" || value == "1";
}

std::array<double, kRcChannelCount> parse_channel_defaults(const std::string & value)
{
  std::array<double, kRcChannelCount> result{};
  std::stringstream stream(value);
  std::string token;
  std::size_t index = 0;
  while (std::getline(stream, token, ',')) {
    if (index >= result.size()) {
      throw std::runtime_error("rc_channel_defaults must contain exactly 16 values");
    }
    result[index++] = std::stod(token);
  }
  if (index != result.size()) {
    throw std::runtime_error("rc_channel_defaults must contain exactly 16 values");
  }
  if (!std::all_of(result.begin(), result.end(), [](double channel) {
      return std::isfinite(channel) && channel >= 0.0 && channel <= 2500.0;
    }))
  {
    throw std::runtime_error("rc_channel_defaults values must be finite and within [0, 2500]");
  }
  return result;
}

}  // namespace

double axis_to_pwm(double axis, bool reverse_axis)
{
  if (!std::isfinite(axis)) {
    return 1500.0;
  }
  const double clamped = std::clamp(axis, -1.0, 1.0);
  const double directed = reverse_axis ? -clamped : clamped;
  return std::round(1500.0 + directed * 500.0);
}

double raw_axis_to_unit(std::int16_t raw_axis)
{
  if (raw_axis == std::numeric_limits<std::int16_t>::min()) {
    return -1.0;
  }
  return std::clamp(static_cast<double>(raw_axis) / 32767.0, -1.0, 1.0);
}

double corrected_axis_to_unit(std::int16_t corrected_axis, const js_corr & correction)
{
  if (correction.type == JS_CORR_NONE) {
    return std::clamp(
      (static_cast<double>(corrected_axis) - kEdgeTxHidCenter) / kEdgeTxHidCenter,
      -1.0, 1.0);
  }
  if (
    correction.type != JS_CORR_BROKEN || correction.coef[2] == 0 ||
    correction.coef[3] == 0)
  {
    return raw_axis_to_unit(corrected_axis);
  }
  if (corrected_axis <= -32767) {
    return -1.0;
  }
  if (corrected_axis >= 32767) {
    return 1.0;
  }

  double hid_value = 0.5 * (correction.coef[0] + correction.coef[1]);
  if (corrected_axis < 0) {
    hid_value = correction.coef[0] +
      static_cast<double>(corrected_axis) * kJoydevCorrectionShift / correction.coef[2];
  } else if (corrected_axis > 0) {
    hid_value = correction.coef[1] +
      static_cast<double>(corrected_axis) * kJoydevCorrectionShift / correction.coef[3];
  }
  return std::clamp((hid_value - kEdgeTxHidCenter) / kEdgeTxHidCenter, -1.0, 1.0);
}

std::array<double, kRcChannelCount> joy_axes_to_rc_channels(
  const std::array<double, kRequiredJoyAxisCount> & axes,
  const std::array<double, kRcChannelCount> & defaults,
  bool reverse_axes)
{
  auto channels = defaults;
  for (std::size_t axis = 0; axis < 8; ++axis) {
    channels[axis] = axis_to_pwm(axes[axis], reverse_axes);
  }
  channels[10] = axis_to_pwm(axes[8], reverse_axes);
  return channels;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SensorInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(get_logger(), "RadioMaster hardware requires exactly one sensor");
    return hardware_interface::CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors.front().name;

  const auto & parameters = info_.hardware_parameters;
  const auto device = get_parameter(parameters, "device", "/dev/input/js0");
  reverse_axes_ = parse_bool(get_parameter(parameters, "reverse_axes", "false"));

  double reconnect_interval_ms = 0.0;
  try {
    reconnect_interval_ms = std::stod(
      get_parameter(parameters, "reconnect_interval_ms", "1000.0"));
    channel_defaults_ = parse_channel_defaults(
      get_parameter(parameters, "rc_channel_defaults", kDefaultRcChannels));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Invalid RadioMaster hardware parameter: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    device.empty() || !std::isfinite(reconnect_interval_ms) ||
    reconnect_interval_ms <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(), "device must be non-empty and reconnect_interval_ms must be positive");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto group_device = get_parameter(parameters, "group_device", "");
  if (!validate_channel_interfaces(kIndividualChannelPrefix) ||
    (!group_device.empty() && !validate_channel_interfaces(kGroupChannelPrefix)))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  individual_receiver_ = std::make_unique<Receiver>(device, reconnect_interval_ms, get_logger());
  if (!group_device.empty()) {
    group_receiver_ = std::make_unique<Receiver>(group_device, reconnect_interval_ms, get_logger());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RadiomasterUsbHardwareInterface::validate_channel_interfaces(const std::string & prefix) const
{
  const auto & interfaces = info_.sensors.front().state_interfaces;
  for (std::size_t channel = 1; channel <= kRcChannelCount; ++channel) {
    const auto name = prefix + std::to_string(channel);
    if (std::none_of(interfaces.begin(), interfaces.end(), [&name](const auto & interface) {
        return interface.name == name;
      }))
    {
      RCLCPP_ERROR(get_logger(), "Missing RadioMaster channel interface: %s", name.c_str());
      return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  publish_safe_states();
  individual_receiver_->configure();
  if (group_receiver_) {group_receiver_->configure();}
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  publish_safe_states();
  individual_receiver_->activate();
  if (group_receiver_) {group_receiver_->activate();}
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  individual_receiver_->deactivate();
  if (group_receiver_) {group_receiver_->deactivate();}
  publish_safe_states();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  individual_receiver_->reset();
  if (group_receiver_) {group_receiver_->reset();}
  publish_safe_states();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RadiomasterUsbHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now_ns = steady_now_ns();
  const auto individual = individual_receiver_->read(now_ns);
  const auto group = group_receiver_ ? group_receiver_->read(now_ns) : ReceiverSample{};
  publish_inputs(individual, group);
  return hardware_interface::return_type::OK;
}

RadiomasterUsbHardwareInterface::Receiver::Receiver(
  const std::string & device, double reconnect_interval_ms, const rclcpp::Logger & logger)
: device_(device),
  reconnect_interval_ns_(static_cast<std::int64_t>(reconnect_interval_ms * 1.0e6)),
  logger_(logger)
{
}

RadiomasterUsbHardwareInterface::Receiver::~Receiver()
{
  close_device(false);
}

void RadiomasterUsbHardwareInterface::Receiver::configure()
{
  next_reconnect_ns_ = 0;
  try_open_device(steady_now_ns());
  RCLCPP_INFO(logger_, "Configured RadioMaster USB receiver: device=%s", device_.c_str());
}

void RadiomasterUsbHardwareInterface::Receiver::activate()
{
  try_open_device(steady_now_ns());
}

void RadiomasterUsbHardwareInterface::Receiver::deactivate()
{
  close_device(false);
}

void RadiomasterUsbHardwareInterface::Receiver::reset()
{
  close_device(false);
  axes_.fill(0.0);
  axes_initialized_.fill(false);
  realtime_tick_ = 0;
  next_reconnect_ns_ = 0;
  open_failure_reported_ = false;
}

RadiomasterUsbHardwareInterface::ReceiverSample
RadiomasterUsbHardwareInterface::Receiver::read(std::int64_t now_ns)
{
  if (!try_open_device(now_ns) || !read_device_events() || !all_required_axes_initialized()) {
    return {};
  }
  realtime_tick_ = (realtime_tick_ + 1U) % 32768U;
  return {axes_, realtime_tick_, true};
}

bool RadiomasterUsbHardwareInterface::Receiver::try_open_device(std::int64_t now_ns)
{
  if (joystick_fd_ >= 0) {
    return true;
  }
  if (now_ns < next_reconnect_ns_) {
    return false;
  }
  next_reconnect_ns_ = now_ns + reconnect_interval_ns_;

  joystick_fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (joystick_fd_ < 0) {
    if (!open_failure_reported_) {
      RCLCPP_WARN(
        logger_, "Cannot open RadioMaster joystick %s: %s",
        device_.c_str(), std::strerror(errno));
      open_failure_reported_ = true;
    }
    return false;
  }

  std::uint8_t axis_count = 0;
  if (
    ioctl(joystick_fd_, JSIOCGAXES, &axis_count) < 0 ||
    axis_count < kRequiredJoyAxisCount)
  {
    RCLCPP_ERROR(
      logger_, "Joystick %s exposes %u axes; at least %zu are required",
      device_.c_str(), static_cast<unsigned int>(axis_count), kRequiredJoyAxisCount);
    close_device(false);
    return false;
  }

  if (!read_joydev_correction(axis_count)) {
    close_device(false);
    return false;
  }

  std::array<char, 128> device_name{};
  if (ioctl(joystick_fd_, JSIOCGNAME(device_name.size()), device_name.data()) < 0) {
    std::strncpy(device_name.data(), "unknown", device_name.size() - 1);
  }

  axes_.fill(0.0);
  axes_initialized_.fill(false);
  realtime_tick_ = 0;
  open_failure_reported_ = false;
  RCLCPP_INFO(
    logger_, "Opened RadioMaster joystick %s (%s, %u axes)",
    device_.c_str(), device_name.data(), static_cast<unsigned int>(axis_count));
  return true;
}

bool RadiomasterUsbHardwareInterface::Receiver::read_joydev_correction(std::uint8_t axis_count)
{
  std::vector<js_corr> original(axis_count);
  if (ioctl(joystick_fd_, JSIOCGCORR, original.data()) < 0) {
    RCLCPP_ERROR(
      logger_, "Cannot read joystick correction for %s: %s",
      device_.c_str(), std::strerror(errno));
    return false;
  }

  joydev_correction_.assign(axis_count, js_corr{});
  for (auto & correction : joydev_correction_) {
    correction.type = JS_CORR_NONE;
  }

  original_joydev_correction_ = original;
  if (ioctl(joystick_fd_, JSIOCSCORR, joydev_correction_.data()) < 0) {
    RCLCPP_ERROR(
      logger_, "Cannot disable joystick correction for %s: %s",
      device_.c_str(), std::strerror(errno));
    original_joydev_correction_.clear();
    joydev_correction_.clear();
    return false;
  }

  return true;
}

bool RadiomasterUsbHardwareInterface::Receiver::read_device_events()
{
  pollfd descriptor{};
  descriptor.fd = joystick_fd_;
  descriptor.events = POLLIN | POLLERR | POLLHUP;
  const int poll_result = poll(&descriptor, 1, 0);
  if (poll_result < 0) {
    if (errno == EINTR) {
      return true;
    }
    close_device(true);
    return false;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    close_device(true);
    return false;
  }
  if ((descriptor.revents & POLLIN) == 0) {
    return true;
  }

  while (true) {
    js_event event{};
    const ssize_t bytes = ::read(joystick_fd_, &event, sizeof(event));
    if (bytes == static_cast<ssize_t>(sizeof(event))) {
      const std::uint8_t event_type = event.type & ~JS_EVENT_INIT;
      if (event_type == JS_EVENT_AXIS) {
        process_axis_event(event.number, event.value);
      }
      continue;
    }
    if (bytes < 0 && errno == EINTR) {
      continue;
    }
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;
    }
    close_device(true);
    return false;
  }
}

void RadiomasterUsbHardwareInterface::Receiver::process_axis_event(
  std::uint8_t axis, std::int16_t value)
{
  if (axis >= axes_.size()) {
    return;
  }
  axes_[axis] = corrected_axis_to_unit(value, joydev_correction_[axis]);
  axes_initialized_[axis] = true;
}

void RadiomasterUsbHardwareInterface::Receiver::close_device(bool report_disconnect)
{
  if (joystick_fd_ < 0) {
    return;
  }

  if (!original_joydev_correction_.empty()) {
    if (ioctl(joystick_fd_, JSIOCSCORR, original_joydev_correction_.data()) < 0) {
      RCLCPP_WARN(
        logger_, "Cannot restore joystick correction for %s: %s",
        device_.c_str(), std::strerror(errno));
    }
    original_joydev_correction_.clear();
  }

  ::close(joystick_fd_);
  joystick_fd_ = -1;
  joydev_correction_.clear();
  axes_initialized_.fill(false);
  next_reconnect_ns_ = steady_now_ns() + reconnect_interval_ns_;
  if (report_disconnect) {
    RCLCPP_ERROR(logger_, "RadioMaster joystick disconnected: %s", device_.c_str());
  }
}

bool RadiomasterUsbHardwareInterface::Receiver::all_required_axes_initialized() const
{
  return std::all_of(
    axes_initialized_.begin(), axes_initialized_.end(), [](bool initialized) {
      return initialized;
    });
}

void RadiomasterUsbHardwareInterface::publish_safe_states()
{
  publish_inputs({}, {});
}

void RadiomasterUsbHardwareInterface::publish_inputs(
  const ReceiverSample & individual, const ReceiverSample & group)
{
  publish_channels(
    kIndividualChannelPrefix, individual.ready ?
    joy_axes_to_rc_channels(individual.axes, channel_defaults_, reverse_axes_) : channel_defaults_);

  if (group_receiver_) {
    // Invalid group channels block only the group input; individual status stays independent.
    std::array<double, kRcChannelCount> channels{};
    channels.fill(std::numeric_limits<double>::quiet_NaN());
    if (group.ready) {
      channels = joy_axes_to_rc_channels(group.axes, channel_defaults_, reverse_axes_);
    }
    publish_channels(kGroupChannelPrefix, channels);
  }
  publish_individual_status(individual);
}

void RadiomasterUsbHardwareInterface::publish_channels(
  const std::string & prefix, const std::array<double, kRcChannelCount> & channels)
{
  for (std::size_t index = 0; index < channels.size(); ++index) {
    set_state(sensor_name_ + "/" + prefix + std::to_string(index + 1), channels[index]);
  }
}

void RadiomasterUsbHardwareInterface::publish_individual_status(const ReceiverSample & individual)
{
  // The current HAT interface has one status set, driven by the individual receiver.
  const bool ready = individual.ready;
  set_state(sensor_name_ + "/Hardware Error Status", ready ? 0.0 : 1.0);
  set_state(sensor_name_ + "/Realtime Tick", ready ? individual.realtime_tick : 0.0);
  set_state(sensor_name_ + "/E-stop Active", ready ? 0.0 : 1.0);
  set_state(sensor_name_ + "/CRSF Failsafe", ready ? 0.0 : 1.0);
  set_state(sensor_name_ + "/CRSF Link Quality", ready ? 100.0 : 0.0);
  set_state(sensor_name_ + "/CRSF RSSI 1", ready ? 100.0 : 0.0);
  set_state(sensor_name_ + "/CRSF Last Frame Age", ready ? 0.0 : 65535.0);
}

std::int64_t RadiomasterUsbHardwareInterface::steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace radiomaster_usb_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  radiomaster_usb_hardware_interface::RadiomasterUsbHardwareInterface,
  hardware_interface::SensorInterface)
