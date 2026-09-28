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

#include <gtest/gtest.h>

#include <linux/joystick.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_component_params.hpp"
#include "radiomaster_usb_hardware_interface/radiomaster_usb_hardware_interface.hpp"

namespace
{

struct FakeJoystick
{
  bool connected{true};
  int opens{0};
  int closes{0};
  std::deque<js_event> events;
  std::array<js_corr, 9> correction{};
};

constexpr int kFirstFakeFd = 100000;
std::array<FakeJoystick, 2> joysticks;

FakeJoystick * joystick(int fd)
{
  return fd >= kFirstFakeFd && fd < kFirstFakeFd + 2 ? &joysticks[fd - kFirstFakeFd] : nullptr;
}

void queue_axes(std::size_t receiver, std::int16_t value)
{
  for (std::uint8_t axis = 0; axis < 9; ++axis) {
    js_event event{};
    event.type = JS_EVENT_AXIS | JS_EVENT_INIT;
    event.number = axis;
    event.value = value;
    joysticks[receiver].events.push_back(event);
  }
}

}  // namespace

// Link-time wrappers substitute joystick devices without changing the production API.
extern "C"
{
int __real_open(const char *, int, ...);
int __real_ioctl(int, unsigned long, ...);  // NOLINT(runtime/int)
int __real_poll(pollfd *, nfds_t, int);
ssize_t __real_read(int, void *, size_t);
int __real_close(int);

int __wrap_open(const char * path, int flags)
{
  for (std::size_t i = 0; i < joysticks.size(); ++i) {
    if (std::string(path) == "/test/js" + std::to_string(i)) {
      if (!joysticks[i].connected) {errno = ENODEV; return -1;}
      ++joysticks[i].opens;
      return kFirstFakeFd + static_cast<int>(i);
    }
  }
  return __real_open(path, flags);
}

int __wrap_open64(const char * path, int flags)
{
  return __wrap_open(path, flags);
}

int __wrap_ioctl(int fd, unsigned long request, void * data)  // NOLINT(runtime/int)
{
  if (!joystick(fd)) {return __real_ioctl(fd, request, data);}
  if (request == JSIOCGAXES) {
    *static_cast<std::uint8_t *>(data) = 9;
  } else if (request == JSIOCGCORR) {
    std::memcpy(data, joystick(fd)->correction.data(), sizeof(js_corr) * 9);
  } else if (request == JSIOCSCORR) {
    std::memcpy(joystick(fd)->correction.data(), data, sizeof(js_corr) * 9);
  } else if (request == JSIOCGNAME(128)) {
    std::strcpy(static_cast<char *>(data), "Test RadioMaster");
  } else {
    errno = EINVAL;
    return -1;
  }
  return 0;
}

int __wrap_poll(pollfd * fds, nfds_t count, int timeout)
{
  auto * device = count == 1 ? joystick(fds[0].fd) : nullptr;
  if (!device) {return __real_poll(fds, count, timeout);}
  fds[0].revents = !device->connected ? POLLHUP : (device->events.empty() ? 0 : POLLIN);
  return fds[0].revents ? 1 : 0;
}

ssize_t __wrap_read(int fd, void * data, size_t size)
{
  auto * device = joystick(fd);
  if (!device) {return __real_read(fd, data, size);}
  if (device->events.empty()) {errno = EAGAIN; return -1;}
  if (size != sizeof(js_event)) {errno = EINVAL; return -1;}
  std::memcpy(data, &device->events.front(), sizeof(js_event));
  device->events.pop_front();
  return sizeof(js_event);
}

int __wrap_close(int fd)
{
  auto * device = joystick(fd);
  if (!device) {return __real_close(fd);}
  ++device->closes;
  return 0;
}
}  // extern "C"

namespace
{

using Hardware = radiomaster_usb_hardware_interface::RadiomasterUsbHardwareInterface;
using CallbackReturn = hardware_interface::CallbackReturn;

class DualReceiverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override {joysticks = {};}

  void initialize(bool group_enabled = true)
  {
    hardware_interface::HardwareComponentParams params;
    params.clock = std::make_shared<rclcpp::Clock>();
    auto & info = params.hardware_info;
    info.name = "test_hat";
    info.type = "sensor";
    info.hardware_parameters = {{"device", "/test/js0"}, {"reconnect_interval_ms", "1"}};
    if (group_enabled) {info.hardware_parameters["group_device"] = "/test/js1";}
    hardware_interface::ComponentInfo sensor;
    sensor.name = "hat";
    sensor.type = "sensor";
    for (const auto * name : {"Hardware Error Status", "Realtime Tick", "E-stop Active",
        "CRSF Failsafe", "CRSF Link Quality", "CRSF RSSI 1", "CRSF Last Frame Age"})
    {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      sensor.state_interfaces.push_back(interface);
    }
    for (const auto * prefix : {"RC Channel ", "Remote 2 CH"}) {
      if (!group_enabled && std::string(prefix) == "Remote 2 CH") {continue;}
      for (int channel = 1; channel <= 16; ++channel) {
        hardware_interface::InterfaceInfo interface;
        interface.name = std::string(prefix) + std::to_string(channel);
        sensor.state_interfaces.push_back(interface);
      }
    }
    info.sensors.push_back(sensor);
    ASSERT_EQ(hardware_.init(params), CallbackReturn::SUCCESS);
    const auto interfaces = hardware_.on_export_state_interfaces();
    EXPECT_EQ(interfaces.size(), group_enabled ? 39U : 23U);
    ASSERT_EQ(hardware_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }

  void read()
  {
    ASSERT_EQ(hardware_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      hardware_interface::return_type::OK);
  }

  double state(const std::string & name) {return hardware_.get_state("hat/" + name);}

  Hardware hardware_;
};

TEST_F(DualReceiverTest, MapsTwoIndependentUsbInputsToTheRealHatChannelNames)
{
  queue_axes(0, 0);
  queue_axes(1, 2048);
  initialize();
  read();
  EXPECT_DOUBLE_EQ(state("RC Channel 1"), 1000.0);
  EXPECT_DOUBLE_EQ(state("Remote 2 CH1"), 2000.0);
  EXPECT_DOUBLE_EQ(state("RC Channel 11"), 1000.0);
  EXPECT_DOUBLE_EQ(state("Remote 2 CH11"), 2000.0);
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 0.0);
  const auto tick = state("Realtime Tick");
  read();  // A stationary, connected joystick does not produce new events.
  EXPECT_GT(state("Realtime Tick"), tick);
  EXPECT_DOUBLE_EQ(state("Remote 2 CH1"), 2000.0);
}

TEST_F(DualReceiverTest, MissingGroupDoesNotInvalidateIndividualAndCanRecover)
{
  joysticks[1].connected = false;
  queue_axes(0, 1024);
  initialize();
  read();
  EXPECT_DOUBLE_EQ(state("RC Channel 1"), 1500.0);
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 0.0);
  EXPECT_TRUE(std::isnan(state("Remote 2 CH1")));
  joysticks[1].connected = true;
  queue_axes(1, 2048);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  read();
  EXPECT_DOUBLE_EQ(state("Remote 2 CH1"), 2000.0);
  EXPECT_EQ(joysticks[0].opens, 1);
}

TEST_F(DualReceiverTest, GroupDisconnectInvalidatesOnlyGroupChannelsAndRequiresFreshInitialization)
{
  queue_axes(0, 1024);
  queue_axes(1, 2048);
  initialize();
  read();
  joysticks[1].connected = false;
  read();
  EXPECT_TRUE(std::isnan(state("Remote 2 CH1")));
  EXPECT_DOUBLE_EQ(state("RC Channel 1"), 1500.0);
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 0.0);
  joysticks[1].connected = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  read();  // Reopening alone must not make the old channels valid.
  EXPECT_TRUE(std::isnan(state("Remote 2 CH1")));
  queue_axes(1, 0);
  read();
  EXPECT_DOUBLE_EQ(state("Remote 2 CH1"), 1000.0);
  EXPECT_EQ(joysticks[1].closes, 1);
}

TEST_F(DualReceiverTest, IndividualDisconnectKeepsExistingFailsafeEvenWhenGroupIsHealthy)
{
  queue_axes(0, 1024);
  queue_axes(1, 2048);
  initialize();
  read();
  joysticks[0].connected = false;
  read();
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 1.0);
  EXPECT_DOUBLE_EQ(state("E-stop Active"), 1.0);
  EXPECT_DOUBLE_EQ(state("Realtime Tick"), 0.0);
  EXPECT_DOUBLE_EQ(state("Remote 2 CH1"), 2000.0);
}

TEST_F(DualReceiverTest, SingleUsbKeepsLegacyInterfacesAndLifecycle)
{
  queue_axes(0, 2048);
  initialize(false);
  read();
  EXPECT_DOUBLE_EQ(state("RC Channel 1"), 2000.0);
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 0.0);
  EXPECT_EQ(joysticks[1].opens, 0);
  ASSERT_EQ(hardware_.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(state("RC Channel 1"), 1500.0);
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 1.0);
  ASSERT_EQ(hardware_.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  queue_axes(0, 0);
  ASSERT_EQ(hardware_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  read();
  EXPECT_DOUBLE_EQ(state("RC Channel 1"), 1000.0);
}

TEST_F(DualReceiverTest, EachReceiverRestoresItsOwnCorrectionOnDisconnectAndCleanup)
{
  for (std::size_t receiver = 0; receiver < joysticks.size(); ++receiver) {
    for (std::size_t axis = 0; axis < 9; ++axis) {
      joysticks[receiver].correction[axis].type = JS_CORR_BROKEN;
      joysticks[receiver].correction[axis].coef[0] = static_cast<int>(100 * receiver + axis);
    }
    queue_axes(receiver, 1024);
  }
  const auto individual_original = joysticks[0].correction;
  const auto group_original = joysticks[1].correction;
  initialize();
  read();
  for (const auto & device : joysticks) {
    for (const auto & correction : device.correction) {
      EXPECT_EQ(correction.type, JS_CORR_NONE);
    }
  }

  joysticks[1].connected = false;
  read();
  for (std::size_t axis = 0; axis < 9; ++axis) {
    EXPECT_EQ(joysticks[1].correction[axis].type, group_original[axis].type);
    EXPECT_EQ(joysticks[1].correction[axis].coef[0], group_original[axis].coef[0]);
    EXPECT_EQ(joysticks[0].correction[axis].type, JS_CORR_NONE);
  }

  ASSERT_EQ(hardware_.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  for (std::size_t axis = 0; axis < 9; ++axis) {
    EXPECT_EQ(joysticks[0].correction[axis].type, individual_original[axis].type);
    EXPECT_EQ(joysticks[0].correction[axis].coef[0], individual_original[axis].coef[0]);
  }
  EXPECT_EQ(joysticks[0].closes, 1);
  EXPECT_EQ(joysticks[1].closes, 1);
}

TEST_F(DualReceiverTest, ReconnectionAndLifecycleResetDoNotRewindAcceptedTicks)
{
  queue_axes(0, 1024);
  queue_axes(1, 1024);
  initialize();
  for (int i = 0; i < 2078; ++i) {
    read();
  }
  ASSERT_DOUBLE_EQ(state("Realtime Tick"), 2078.0);

  joysticks[0].connected = false;
  read();
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 1.0);
  joysticks[0].connected = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  read();
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 1.0);  // Still waiting for all axis initialization events.
  queue_axes(0, 1024);
  read();
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 0.0);
  EXPECT_DOUBLE_EQ(state("Realtime Tick"), 2079.0);

  ASSERT_EQ(hardware_.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  queue_axes(0, 1024);
  queue_axes(1, 1024);
  ASSERT_EQ(hardware_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  read();
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 0.0);
  EXPECT_DOUBLE_EQ(state("Realtime Tick"), 2080.0);
}

TEST_F(DualReceiverTest, TickStillRollsOverAtTheHatCounterLimit)
{
  queue_axes(0, 1024);
  initialize(false);
  for (int i = 0; i < 32767; ++i) {
    read();
  }
  ASSERT_DOUBLE_EQ(state("Realtime Tick"), 32767.0);
  read();
  EXPECT_DOUBLE_EQ(state("Realtime Tick"), 0.0);
  EXPECT_DOUBLE_EQ(state("CRSF Failsafe"), 0.0);
  read();
  EXPECT_DOUBLE_EQ(state("Realtime Tick"), 1.0);
}

}  // namespace
