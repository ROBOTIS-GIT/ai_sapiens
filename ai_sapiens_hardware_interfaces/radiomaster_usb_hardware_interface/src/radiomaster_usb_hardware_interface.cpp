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
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace radiomaster_usb_hardware_interface
{
namespace
{

constexpr const char * kDefaultRcChannels =
  "1500,1500,1500,1500,1000,1500,1000,1000,1000,1000,1000,1000,1500,1500,1500,1500";
constexpr double kEdgeTxHidCenter = 1024.0;
constexpr double kJoydevCorrectionShift = 16384.0;

std::unordered_map<int, std::vector<js_corr>> & original_joydev_corrections()
{
  static std::unordered_map<int, std::vector<js_corr>> corrections;
  return corrections;
}

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

  try {
    reconnect_interval_ms_ = std::stod(
      get_parameter(parameters, "reconnect_interval_ms", "1000.0"));
    channel_defaults_ = parse_channel_defaults(
      get_parameter(parameters, "rc_channel_defaults", kDefaultRcChannels));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Invalid RadioMaster hardware parameter: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    device.empty() || !std::isfinite(reconnect_interval_ms_) ||
    reconnect_interval_ms_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(), "device must be non-empty and reconnect_interval_ms must be positive");
    return hardware_interface::CallbackReturn::ERROR;
  }

  receivers_.push_back(Receiver{device, "RC Channel ", channel_defaults_});
  const auto group_device = get_parameter(parameters, "group_device", "");
  if (!group_device.empty()) {
    std::array<double, kRcChannelCount> unavailable{};
    unavailable.fill(std::numeric_limits<double>::quiet_NaN());
    receivers_.push_back(Receiver{group_device, "Remote 2 CH", unavailable});
  }

  // Fail at configuration time if the declared interfaces do not match either receiver.
  const auto & interfaces = info_.sensors.front().state_interfaces;
  for (const auto & receiver : receivers_) {
    for (std::size_t channel = 1; channel <= kRcChannelCount; ++channel) {
      const auto name = receiver.channel_prefix + std::to_string(channel);
      if (std::none_of(interfaces.begin(), interfaces.end(), [&name](const auto & interface) {
          return interface.name == name;
        }))
      {
        RCLCPP_ERROR(get_logger(), "Missing RadioMaster channel interface: %s", name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

RadiomasterUsbHardwareInterface::~RadiomasterUsbHardwareInterface()
{
  for (auto & receiver : receivers_) {
    close_device(receiver, false);
  }
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  publish_safe_states();
  for (auto & receiver : receivers_) {
    receiver.next_reconnect_ns = 0;
    try_open_device(receiver, steady_now_ns());
    RCLCPP_INFO(
      get_logger(), "Configured RadioMaster USB hardware: sensor=%s channels=%s device=%s",
      sensor_name_.c_str(), receiver.channel_prefix.c_str(), receiver.device.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  publish_safe_states();
  for (auto & receiver : receivers_) {
    try_open_device(receiver, steady_now_ns());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & receiver : receivers_) {
    close_device(receiver, false);
  }
  publish_safe_states();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RadiomasterUsbHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  for (auto & receiver : receivers_) {
    close_device(receiver, false);
    receiver.axes.fill(0.0);
    receiver.axes_initialized.fill(false);
    receiver.realtime_tick = 0;
    receiver.next_reconnect_ns = 0;
    receiver.open_failure_reported = false;
  }
  publish_safe_states();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RadiomasterUsbHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now_ns = steady_now_ns();
  for (auto & receiver : receivers_) {
    receiver.ready = read_receiver(receiver, now_ns);
    publish_channels(
      receiver, receiver.ready ?
      joy_axes_to_rc_channels(receiver.axes, channel_defaults_, reverse_axes_) :
      receiver.unavailable_channels);
  }
  publish_status(receivers_.front());
  return hardware_interface::return_type::OK;
}

bool RadiomasterUsbHardwareInterface::read_receiver(Receiver & receiver, std::int64_t now_ns)
{
  if (!try_open_device(receiver, now_ns) || !read_device_events(receiver) ||
    !all_required_axes_initialized(receiver))
  {
    return false;
  }
  receiver.realtime_tick = (receiver.realtime_tick + 1U) % 32768U;
  return true;
}

bool RadiomasterUsbHardwareInterface::try_open_device(Receiver & receiver, std::int64_t now_ns)
{
  if (receiver.joystick_fd >= 0) {
    return true;
  }
  if (now_ns < receiver.next_reconnect_ns) {
    return false;
  }
  receiver.next_reconnect_ns = now_ns + static_cast<std::int64_t>(reconnect_interval_ms_ * 1.0e6);

  receiver.joystick_fd = open(receiver.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (receiver.joystick_fd < 0) {
    if (!receiver.open_failure_reported) {
      RCLCPP_WARN(
        get_logger(), "Cannot open RadioMaster joystick %s: %s",
        receiver.device.c_str(), std::strerror(errno));
      receiver.open_failure_reported = true;
    }
    return false;
  }

  std::uint8_t axis_count = 0;
  if (
    ioctl(receiver.joystick_fd, JSIOCGAXES, &axis_count) < 0 ||
    axis_count < kRequiredJoyAxisCount)
  {
    RCLCPP_ERROR(
      get_logger(), "Joystick %s exposes %u axes; at least %zu are required",
      receiver.device.c_str(), static_cast<unsigned int>(axis_count), kRequiredJoyAxisCount);
    close_device(receiver, false);
    return false;
  }

  if (!read_joydev_correction(receiver, axis_count)) {
    close_device(receiver, false);
    return false;
  }

  std::array<char, 128> device_name{};
  if (ioctl(receiver.joystick_fd, JSIOCGNAME(device_name.size()), device_name.data()) < 0) {
    std::strncpy(device_name.data(), "unknown", device_name.size() - 1);
  }

  receiver.axes.fill(0.0);
  receiver.axes_initialized.fill(false);
  receiver.realtime_tick = 0;
  receiver.open_failure_reported = false;
  RCLCPP_INFO(
    get_logger(), "Opened RadioMaster joystick %s (%s, %u axes)",
    receiver.device.c_str(), device_name.data(), static_cast<unsigned int>(axis_count));
  return true;
}

bool RadiomasterUsbHardwareInterface::read_joydev_correction(
  Receiver & receiver, std::uint8_t axis_count)
{
  std::vector<js_corr> original(axis_count);
  if (ioctl(receiver.joystick_fd, JSIOCGCORR, original.data()) < 0) {
    RCLCPP_ERROR(
      get_logger(), "Cannot read joystick correction for %s: %s",
      receiver.device.c_str(), std::strerror(errno));
    return false;
  }

  receiver.joydev_correction.assign(axis_count, js_corr{});
  for (auto & correction : receiver.joydev_correction) {
    correction.type = JS_CORR_NONE;
  }

  auto & original_corrections = original_joydev_corrections();
  original_corrections.insert_or_assign(receiver.joystick_fd, std::move(original));
  if (ioctl(receiver.joystick_fd, JSIOCSCORR, receiver.joydev_correction.data()) < 0) {
    RCLCPP_ERROR(
      get_logger(), "Cannot disable joystick correction for %s: %s",
      receiver.device.c_str(), std::strerror(errno));
    original_corrections.erase(receiver.joystick_fd);
    receiver.joydev_correction.clear();
    return false;
  }

  return true;
}

bool RadiomasterUsbHardwareInterface::read_device_events(Receiver & receiver)
{
  pollfd descriptor{};
  descriptor.fd = receiver.joystick_fd;
  descriptor.events = POLLIN | POLLERR | POLLHUP;
  const int poll_result = poll(&descriptor, 1, 0);
  if (poll_result < 0) {
    if (errno == EINTR) {
      return true;
    }
    close_device(receiver, true);
    return false;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    close_device(receiver, true);
    return false;
  }
  if ((descriptor.revents & POLLIN) == 0) {
    return true;
  }

  while (true) {
    js_event event{};
    const ssize_t bytes = ::read(receiver.joystick_fd, &event, sizeof(event));
    if (bytes == static_cast<ssize_t>(sizeof(event))) {
      const std::uint8_t event_type = event.type & ~JS_EVENT_INIT;
      if (event_type == JS_EVENT_AXIS) {
        process_axis_event(receiver, event.number, event.value);
      }
      continue;
    }
    if (bytes < 0 && errno == EINTR) {
      continue;
    }
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;
    }
    close_device(receiver, true);
    return false;
  }
}

void RadiomasterUsbHardwareInterface::process_axis_event(
  Receiver & receiver, std::uint8_t axis, std::int16_t value)
{
  if (axis >= receiver.axes.size()) {
    return;
  }
  receiver.axes[axis] = corrected_axis_to_unit(value, receiver.joydev_correction[axis]);
  receiver.axes_initialized[axis] = true;
}

void RadiomasterUsbHardwareInterface::close_device(
  Receiver & receiver, bool report_disconnect)
{
  if (receiver.joystick_fd < 0) {
    return;
  }

  auto & original_corrections = original_joydev_corrections();
  const auto original = original_corrections.find(receiver.joystick_fd);
  if (original != original_corrections.end()) {
    if (ioctl(receiver.joystick_fd, JSIOCSCORR, original->second.data()) < 0) {
      RCLCPP_WARN(
        get_logger(), "Cannot restore joystick correction for %s: %s",
        receiver.device.c_str(), std::strerror(errno));
    }
    original_corrections.erase(original);
  }

  ::close(receiver.joystick_fd);
  receiver.joystick_fd = -1;
  receiver.joydev_correction.clear();
  receiver.axes_initialized.fill(false);
  receiver.next_reconnect_ns = steady_now_ns() +
    static_cast<std::int64_t>(reconnect_interval_ms_ * 1.0e6);
  if (report_disconnect) {
    RCLCPP_ERROR(get_logger(), "RadioMaster joystick disconnected: %s", receiver.device.c_str());
  }
}

bool RadiomasterUsbHardwareInterface::all_required_axes_initialized(
  const Receiver & receiver) const
{
  return std::all_of(
    receiver.axes_initialized.begin(), receiver.axes_initialized.end(), [](bool initialized) {
      return initialized;
    });
}

void RadiomasterUsbHardwareInterface::publish_safe_states()
{
  for (auto & receiver : receivers_) {
    receiver.ready = false;
    publish_channels(receiver, receiver.unavailable_channels);
  }
  publish_status(receivers_.front());
}

void RadiomasterUsbHardwareInterface::publish_channels(
  const Receiver & receiver, const std::array<double, kRcChannelCount> & channels)
{
  for (std::size_t index = 0; index < channels.size(); ++index) {
    set_state(
      sensor_name_ + "/" + receiver.channel_prefix + std::to_string(index + 1), channels[index]);
  }
}

void RadiomasterUsbHardwareInterface::publish_status(const Receiver & individual)
{
  // Keep the existing individual receiver's status on the shared HAT interfaces.
  // An unavailable group receiver publishes invalid channels, so its broadcaster
  // rejects that input without invalidating the individual receiver.
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
