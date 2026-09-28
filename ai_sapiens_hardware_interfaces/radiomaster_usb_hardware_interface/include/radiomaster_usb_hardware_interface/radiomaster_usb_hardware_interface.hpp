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

#ifndef RADIOMASTER_USB_HARDWARE_INTERFACE__RADIOMASTER_USB_HARDWARE_INTERFACE_HPP_
#define RADIOMASTER_USB_HARDWARE_INTERFACE__RADIOMASTER_USB_HARDWARE_INTERFACE_HPP_

#include <linux/joystick.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace radiomaster_usb_hardware_interface
{

constexpr std::size_t kRcChannelCount = 16;
constexpr std::size_t kRequiredJoyAxisCount = 9;

double axis_to_pwm(double axis, bool reverse_axis);
double raw_axis_to_unit(std::int16_t raw_axis);
double corrected_axis_to_unit(std::int16_t corrected_axis, const js_corr & correction);

std::array<double, kRcChannelCount> joy_axes_to_rc_channels(
  const std::array<double, kRequiredJoyAxisCount> & axes,
  const std::array<double, kRcChannelCount> & defaults,
  bool reverse_axes);

class RadiomasterUsbHardwareInterface : public hardware_interface::SensorInterface
{
public:
  ~RadiomasterUsbHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Receiver
  {
    std::string device;
    std::string channel_prefix;
    std::array<double, kRcChannelCount> unavailable_channels{};
    std::array<double, kRequiredJoyAxisCount> axes{};
    std::array<bool, kRequiredJoyAxisCount> axes_initialized{};
    std::vector<js_corr> joydev_correction{};
    int joystick_fd{-1};
    std::uint32_t realtime_tick{0};
    std::int64_t next_reconnect_ns{0};
    bool open_failure_reported{false};
    bool ready{false};
  };

  bool read_receiver(Receiver & receiver, std::int64_t now_ns);
  bool try_open_device(Receiver & receiver, std::int64_t now_ns);
  bool read_joydev_correction(Receiver & receiver, std::uint8_t axis_count);
  bool read_device_events(Receiver & receiver);
  void process_axis_event(Receiver & receiver, std::uint8_t axis, std::int16_t value);
  void close_device(Receiver & receiver, bool report_disconnect);
  bool all_required_axes_initialized(const Receiver & receiver) const;
  void publish_safe_states();
  void publish_channels(
    const Receiver & receiver, const std::array<double, kRcChannelCount> & channels);
  void publish_status(const Receiver & individual);
  static std::int64_t steady_now_ns();

  std::string sensor_name_{"hat"};
  double reconnect_interval_ms_{1000.0};
  bool reverse_axes_{false};
  std::array<double, kRcChannelCount> channel_defaults_{};
  std::vector<Receiver> receivers_;
};

}  // namespace radiomaster_usb_hardware_interface

#endif  // RADIOMASTER_USB_HARDWARE_INTERFACE__RADIOMASTER_USB_HARDWARE_INTERFACE_HPP_
