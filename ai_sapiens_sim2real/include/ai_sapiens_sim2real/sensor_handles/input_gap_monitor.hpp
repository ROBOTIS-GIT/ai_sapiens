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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__INPUT_GAP_MONITOR_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__INPUT_GAP_MONITOR_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/sensor_handles/input_gap_state.hpp"

namespace ai_sapiens_sim2real
{

// Input callbacks record bounded events; the ordinary executor writes logs.
class InputGapMonitor
{
public:
  InputGapMonitor(
    const rclcpp::Node::SharedPtr & node, const std::string & topic, double expected_rate_hz,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  void received_raw();
  void received_accepted();

private:
  enum class Stream {Raw, Accepted};
  struct DiagnosticEvent
  {
    Stream stream{Stream::Raw};
    InputGapState::Event gap;
    InputGapState::Time detected_at{};
    int64_t unix_ns{0};
  };

  static constexpr size_t kMaxPendingEvents = 64;
  struct PendingEvents
  {
    std::array<DiagnosticEvent, kMaxPendingEvents> events{};
    size_t count{0};
    uint64_t dropped{0};
  };

  void check_gaps();
  void receive(InputGapState & state, Stream stream);
  // Caller holds mutex_; no formatting, logging, or dynamic allocation here.
  void enqueue(Stream stream, const InputGapState::Event & event, InputGapState::Time now);
  std::optional<PendingEvents> take_pending_events();
  void flush_logs();
  void report(const DiagnosticEvent & event);

  rclcpp::Logger logger_;
  std::string topic_;
  std::mutex mutex_;
  InputGapState raw_;
  InputGapState accepted_;
  std::array<DiagnosticEvent, kMaxPendingEvents> pending_{};
  size_t first_event_{0};
  size_t pending_count_{0};
  uint64_t dropped_events_{0};
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__INPUT_GAP_MONITOR_HPP_
