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

#include <chrono>
#include <limits>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "ai_sapiens_sim2real/sensor_handles/input_gap_monitor.hpp"

namespace ai_sapiens_sim2real
{
using namespace std::chrono_literals;

TEST(InputGapState, WarnsOnceAndReportsFullGapOnRecovery)
{
  const InputGapState::Time start{};
  InputGapState state(1000.0, start);
  EXPECT_FALSE(state.check_gap(start + 49ms).is_warning);
  EXPECT_TRUE(state.check_gap(start + 50ms).is_warning);
  EXPECT_FALSE(state.check_gap(start + 80ms).is_warning);
  const auto recovered = state.record_reception(start + 123ms);
  EXPECT_FALSE(recovered.is_warning);
  EXPECT_TRUE(recovered.is_recovery);
  EXPECT_DOUBLE_EQ(recovered.gap_ms, 123.0);
  EXPECT_FALSE(state.record_reception(start + 124ms).is_recovery);
  EXPECT_TRUE(state.check_gap(start + 174ms).is_warning);
}

TEST(InputGapState, DetectsGapEvenIfTimerWasBlocked)
{
  const InputGapState::Time start{};
  InputGapState state(100.0, start);
  const auto event = state.record_reception(start + 200ms);
  EXPECT_TRUE(event.is_warning);
  EXPECT_TRUE(event.is_recovery);
  EXPECT_DOUBLE_EQ(event.gap_ms, 200.0);
}

TEST(InputGapState, SlowTopicUsesFiveExpectedPeriods)
{
  const InputGapState::Time start{};
  InputGapState state(10.0, start);
  EXPECT_FALSE(state.record_reception(start + 100ms).is_warning);
  EXPECT_FALSE(state.check_gap(start + 599ms).is_warning);
  EXPECT_TRUE(state.check_gap(start + 600ms).is_warning);
}

TEST(InputGapState, RawReceptionDoesNotHideRejectedData)
{
  const InputGapState::Time start{};
  InputGapState raw(100.0, start);
  InputGapState accepted(100.0, start);
  for (int tick = 1; tick <= 10; ++tick) {
    EXPECT_FALSE(raw.record_reception(start + tick * 10ms).is_warning);
  }
  EXPECT_TRUE(accepted.check_gap(start + 100ms).is_warning);
  EXPECT_TRUE(accepted.record_reception(start + 110ms).is_recovery);
}

TEST(InputGapMonitor, TimerWarnsBeforeRecoveryAndLabelsBothStreams)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("input_gap_monitor_test");
    InputGapMonitor monitor(node, "/test/input", 1000.0);
    monitor.received_raw();
    monitor.received_accepted();
    testing::internal::CaptureStderr();
    std::this_thread::sleep_for(80ms);
    for (int i = 0; i < 5; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
    const auto warning = testing::internal::GetCapturedStderr();
    EXPECT_NE(warning.find("[Input gap] topic=/test/input stream=raw"), std::string::npos);
    EXPECT_NE(warning.find("[Input gap] topic=/test/input stream=accepted"), std::string::npos);
    EXPECT_EQ(warning.find("[Input resumed]"), std::string::npos);

    testing::internal::CaptureStderr();
    monitor.received_raw();
    monitor.received_accepted();
    const auto immediate = testing::internal::GetCapturedStderr();
    EXPECT_TRUE(immediate.empty());
    testing::internal::CaptureStderr();
    for (int i = 0; i < 5; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
    const auto recovery = testing::internal::GetCapturedStderr();
    EXPECT_NE(recovery.find("[Input resumed] topic=/test/input stream=raw"), std::string::npos);
    EXPECT_NE(recovery.find("[Input resumed] topic=/test/input stream=accepted"),
        std::string::npos);
    EXPECT_EQ(recovery.find("[Input gap]"), std::string::npos);
    EXPECT_NE(recovery.find("detected_at="), std::string::npos);
    EXPECT_NE(recovery.find("log_delay="), std::string::npos);
  }
  rclcpp::shutdown();
}

TEST(InputGapMonitor, InputExecutorNeverPrintsQueuedWarnings)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("deferred_input_gap_test");
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::executors::SingleThreadedExecutor inputs;
    inputs.add_callback_group(group, node->get_node_base_interface());
    InputGapMonitor monitor(node, "/test/deferred", 1000.0, group);

    testing::internal::CaptureStderr();
    std::this_thread::sleep_for(80ms);
    for (int i = 0; i < 5; ++i) {
      inputs.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    monitor.received_raw();
    monitor.received_accepted();
    EXPECT_TRUE(testing::internal::GetCapturedStderr().empty());

    testing::internal::CaptureStderr();
    for (int i = 0; i < 5; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
    const auto output = testing::internal::GetCapturedStderr();
    const auto warning = output.find("[Input gap]");
    const auto recovery = output.find("[Input resumed]");
    ASSERT_NE(warning, std::string::npos);
    ASSERT_NE(recovery, std::string::npos);
    EXPECT_LT(warning, recovery);
  }
  rclcpp::shutdown();
}

TEST(InputGapMonitor, BoundedQueueReportsOverflowAfterOutputResumes)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("input_gap_overflow_test");
    InputGapMonitor monitor(node, "/test/overflow", 1000.0);
    testing::internal::CaptureStderr();
    for (int i = 0; i < 70; ++i) {
      std::this_thread::sleep_for(55ms);
      monitor.received_raw();
    }
    EXPECT_TRUE(testing::internal::GetCapturedStderr().empty());

    testing::internal::CaptureStderr();
    for (int i = 0; i < 5; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
    const auto output = testing::internal::GetCapturedStderr();
    EXPECT_NE(output.find("dropped="), std::string::npos);
    EXPECT_NE(output.find("oldest events; queue full"), std::string::npos);
    EXPECT_NE(output.find("[Input resumed]"), std::string::npos);
  }
  rclcpp::shutdown();
}

TEST(InputGapState, RejectsInvalidExpectedRates)
{
  EXPECT_THROW(InputGapState(0.0), std::invalid_argument);
  EXPECT_THROW(InputGapState(-1.0), std::invalid_argument);
  EXPECT_THROW(InputGapState(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(InputGapState(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}
}  // namespace ai_sapiens_sim2real
