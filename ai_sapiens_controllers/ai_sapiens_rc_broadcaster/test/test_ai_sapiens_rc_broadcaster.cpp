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

#include <gmock/gmock.h>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ai_sapiens_interfaces/msg/rc_status.hpp"
#include "ai_sapiens_rc_broadcaster/ai_sapiens_rc_broadcaster.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace
{

using Broadcaster = ai_sapiens_rc_broadcaster::AiSapiensRcBroadcaster;
using CallbackReturn = controller_interface::CallbackReturn;
using RcStatus = ai_sapiens_interfaces::msg::RcStatus;
using hardware_interface::LoanedStateInterface;
using hardware_interface::StateInterface;

constexpr size_t kChannelCount = 16;
constexpr size_t kStatusCount = 7;
constexpr size_t kInterfaceCount = kChannelCount + kStatusCount;

const std::array<const char *, kChannelCount> kChannels{{
  "RC Channel 1", "RC Channel 2", "RC Channel 3", "RC Channel 4",
  "RC Channel 5", "RC Channel 6", "RC Channel 7", "RC Channel 8",
  "RC Channel 9", "RC Channel 10", "RC Channel 11", "RC Channel 12",
  "RC Channel 13", "RC Channel 14", "RC Channel 15", "RC Channel 16",
}};

const std::array<const char *, kStatusCount> kStatusInterfaces{{
  "Hardware Error Status",
  "Realtime Tick",
  "E-stop Active",
  "CRSF Failsafe",
  "CRSF Link Quality",
  "CRSF RSSI 1",
  "CRSF Last Frame Age",
}};

class AiSapiensRcBroadcasterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    broadcaster_ = std::make_unique<Broadcaster>();
    subscriber_node_ = std::make_shared<rclcpp::Node>("rc_broadcaster_test_subscriber");
    executor_.add_node(subscriber_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(subscriber_node_);
    broadcaster_.reset();
    subscriber_node_.reset();
  }

  controller_interface::return_type initialize(
    const std::vector<rclcpp::Parameter> & additional_overrides = {})
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_ai_sapiens_rc_broadcaster";
    params.robot_description = "";
    params.controller_manager_update_rate = 1000;
    params.node_namespace = "";
    params.node_options = broadcaster_->define_custom_node_options();

    std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("sensor_name", "hat"),
      rclcpp::Parameter("input_topic_name", "/test/rc_input"),
      rclcpp::Parameter("status_topic_name", "/test/rc_status"),
      rclcpp::Parameter("frame_id", "hat"),
      rclcpp::Parameter("input_publish_rate", 0.0),
      rclcpp::Parameter("status_publish_rate", 0.0),
    };
    overrides.insert(overrides.end(), additional_overrides.begin(), additional_overrides.end());
    params.node_options.parameter_overrides(overrides);
    return broadcaster_->init(params);
  }

  void create_state_interfaces(bool wrong_last_interface = false)
  {
    state_values_.fill(1500.0);
    state_values_[kChannelCount + 0] = 0.0;
    state_values_[kChannelCount + 1] = 1.0;
    state_values_[kChannelCount + 2] = 0.0;
    state_values_[kChannelCount + 3] = 0.0;
    state_values_[kChannelCount + 4] = 75.0;
    state_values_[kChannelCount + 5] = 42.0;
    state_values_[kChannelCount + 6] = 10.0;

    state_handles_.clear();
    state_handles_.reserve(kInterfaceCount);
    for (size_t i = 0; i < kChannelCount; ++i) {
      state_handles_.emplace_back("hat", kChannels[i], &state_values_[i]);
    }
    for (size_t i = 0; i < kStatusCount; ++i) {
      const std::string interface_name =
        wrong_last_interface && i + 1 == kStatusCount ? "Wrong Status" : kStatusInterfaces[i];
      state_handles_.emplace_back(
        "hat", interface_name, &state_values_[kChannelCount + i]);
    }

    std::vector<LoanedStateInterface> loaned_interfaces;
    loaned_interfaces.reserve(state_handles_.size());
    for (auto & interface : state_handles_) {
      loaned_interfaces.emplace_back(interface);
    }
    broadcaster_->assign_interfaces({}, std::move(loaned_interfaces));
  }

  void configure_and_activate()
  {
    ASSERT_EQ(
      broadcaster_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    create_state_interfaces();
    ASSERT_EQ(
      broadcaster_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }

  template<typename MessageT>
  std::shared_ptr<MessageT> update_until_message(
    const std::string & topic,
    const rclcpp::Duration & period = rclcpp::Duration::from_seconds(0.001))
  {
    std::shared_ptr<MessageT> received;
    auto subscription = subscriber_node_->create_subscription<MessageT>(
      topic, rclcpp::QoS(10).reliable(),
      [&received](const std::shared_ptr<MessageT> message) {received = message;});

    for (size_t attempt = 0; attempt < 50 && !received; ++attempt) {
      broadcaster_->update(rclcpp::Time(1000000000), period);
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    executor_.spin_some();
    return received;
  }

  std::unique_ptr<Broadcaster> broadcaster_;
  std::shared_ptr<rclcpp::Node> subscriber_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::array<double, kInterfaceCount> state_values_{};
  std::vector<StateInterface> state_handles_;
};

TEST_F(AiSapiensRcBroadcasterTest, RejectsInvalidNumericParameter)
{
  ASSERT_EQ(
    initialize({rclcpp::Parameter("button_threshold", 3000.0)}),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    broadcaster_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(AiSapiensRcBroadcasterTest, ExposesExpectedStateInterfaces)
{
  ASSERT_EQ(initialize(), controller_interface::return_type::OK);
  ASSERT_EQ(
    broadcaster_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  const auto configuration = broadcaster_->state_interface_configuration();
  EXPECT_EQ(configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(configuration.names.size(), kInterfaceCount);
  EXPECT_EQ(configuration.names.front(), "hat/RC Channel 1");
  EXPECT_EQ(configuration.names[kChannelCount], "hat/Hardware Error Status");
  EXPECT_EQ(configuration.names.back(), "hat/CRSF Last Frame Age");
}

TEST_F(AiSapiensRcBroadcasterTest, RejectsMissingStateInterface)
{
  ASSERT_EQ(initialize(), controller_interface::return_type::OK);
  ASSERT_EQ(
    broadcaster_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  create_state_interfaces(true);
  EXPECT_EQ(
    broadcaster_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(AiSapiensRcBroadcasterTest, PublishesNormalizedInputAndSafeStatus)
{
  ASSERT_EQ(initialize(), controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(configure_and_activate());
  state_values_[0] = 1000.0;
  state_values_[1] = 2000.0;

  const auto input = update_until_message<sensor_msgs::msg::Joy>("/test/rc_input");
  ASSERT_NE(input, nullptr);
  ASSERT_EQ(input->axes.size(), kChannelCount);
  ASSERT_EQ(input->buttons.size(), kChannelCount);
  EXPECT_FLOAT_EQ(input->axes[0], -1.0F);
  EXPECT_FLOAT_EQ(input->axes[1], 1.0F);
  EXPECT_EQ(input->buttons[0], 0);
  EXPECT_EQ(input->buttons[1], 1);

  const auto status = update_until_message<RcStatus>("/test/rc_status");
  ASSERT_NE(status, nullptr);
  EXPECT_EQ(status->header.frame_id, "hat");
  EXPECT_TRUE(status->status_data_valid);
  EXPECT_TRUE(status->realtime_tick_fresh);
  EXPECT_TRUE(status->all_channels_valid);
  EXPECT_TRUE(status->hardware_ok);
  EXPECT_TRUE(status->estop_released);
  EXPECT_TRUE(status->rc_link_ok);
  EXPECT_TRUE(status->is_control_input_safe);
  EXPECT_EQ(status->crsf_link_quality, 75);
}

TEST_F(AiSapiensRcBroadcasterTest, InvalidChannelProducesNeutralUnsafeOutput)
{
  ASSERT_EQ(initialize(), controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(configure_and_activate());
  state_values_[3] = std::numeric_limits<double>::quiet_NaN();

  const auto status = update_until_message<RcStatus>("/test/rc_status");
  ASSERT_NE(status, nullptr);
  ASSERT_EQ(status->channels.size(), kChannelCount);
  EXPECT_FALSE(status->channels[3].valid);
  EXPECT_EQ(status->channels[3].rc_us, 1500);
  EXPECT_FLOAT_EQ(status->channels[3].axis, 0.0F);
  EXPECT_FALSE(status->all_channels_valid);
  EXPECT_FALSE(status->is_control_input_safe);
}

TEST_F(AiSapiensRcBroadcasterTest, FrozenRealtimeTickUsesMonotonicUpdatePeriod)
{
  ASSERT_EQ(
    initialize({rclcpp::Parameter("watchdog_realtime_tick_timeout_ms", 10.0)}),
    controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(configure_and_activate());

  auto first_status = update_until_message<RcStatus>("/test/rc_status");
  ASSERT_NE(first_status, nullptr);
  ASSERT_TRUE(first_status->realtime_tick_fresh);

  std::shared_ptr<RcStatus> stale_status;
  auto subscription = subscriber_node_->create_subscription<RcStatus>(
    "/test/rc_status", rclcpp::QoS(10).reliable(),
    [&stale_status](const RcStatus::SharedPtr message) {
      if (!message->realtime_tick_fresh) {
        stale_status = message;
      }
    });
  for (size_t attempt = 0; attempt < 20 && !stale_status; ++attempt) {
    broadcaster_->update(
      rclcpp::Time(1000000000), rclcpp::Duration::from_seconds(0.02));
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  ASSERT_NE(stale_status, nullptr);
  EXPECT_FALSE(stale_status->rc_link_ok);
  EXPECT_FALSE(stale_status->is_control_input_safe);
}

}  // namespace
