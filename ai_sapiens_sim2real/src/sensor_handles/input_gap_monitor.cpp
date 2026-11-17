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

#include "ai_sapiens_sim2real/sensor_handles/input_gap_monitor.hpp"

namespace ai_sapiens_sim2real
{

InputGapMonitor::InputGapMonitor(
  const rclcpp::Node::SharedPtr & node, const std::string & topic, double expected_rate_hz,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: logger_(node->get_logger()), topic_(topic),
  raw_(expected_rate_hz), accepted_(expected_rate_hz)
{
  check_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(10), [this]() {check_gaps();}, callback_group);
  // Default callback group: formatting and output never run in the input executor.
  log_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(10), [this]() {flush_logs();});
}

void InputGapMonitor::received_raw() {receive(raw_, Stream::Raw);}
void InputGapMonitor::received_accepted() {receive(accepted_, Stream::Accepted);}

void InputGapMonitor::check_gaps()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = InputGapState::Clock::now();
  enqueue(Stream::Raw, raw_.check_gap(now), now);
  enqueue(Stream::Accepted, accepted_.check_gap(now), now);
}

void InputGapMonitor::receive(InputGapState & state, Stream stream)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = InputGapState::Clock::now();
  enqueue(stream, state.record_reception(now), now);
}

void InputGapMonitor::enqueue(
  Stream stream, const InputGapState::Event & event, InputGapState::Time now)
{
  if (!event.is_warning && !event.is_recovery) {
    return;
  }

  // Retain the newest events if the ordinary executor is stalled for a long time.
  if (pending_count_ == kMaxPendingEvents) {
    first_event_ = (first_event_ + 1) % kMaxPendingEvents;
    --pending_count_;
    ++dropped_events_;
  }

  const auto index = (first_event_ + pending_count_) % kMaxPendingEvents;
  const auto unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  pending_[index] = {stream, event, now, unix_ns};
  ++pending_count_;
}

std::optional<InputGapMonitor::PendingEvents> InputGapMonitor::take_pending_events()
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);

  if (!lock.owns_lock()) {
    return std::nullopt;
  }

  PendingEvents batch;
  batch.count = pending_count_;
  batch.dropped = dropped_events_;

  for (size_t i = 0; i < batch.count; ++i) {
    batch.events[i] = pending_[(first_event_ + i) % kMaxPendingEvents];
  }

  first_event_ = 0;
  pending_count_ = 0;
  dropped_events_ = 0;
  return batch;
}

void InputGapMonitor::flush_logs()
{
  const auto batch = take_pending_events();

  if (!batch) {
    return;
  }

  if (batch->dropped != 0) {
    RCLCPP_WARN(logger_, "[Input diagnostics] topic=%s dropped=%llu oldest events; queue full",
      topic_.c_str(), static_cast<unsigned long long>(batch->dropped));
  }

  for (size_t i = 0; i < batch->count; ++i) {
    report(batch->events[i]);
  }
}

void InputGapMonitor::report(const DiagnosticEvent & event)
{
  const char * stream = event.stream == Stream::Raw ? "raw" : "accepted";
  const auto seconds = static_cast<long long>(event.unix_ns / 1'000'000'000);
  const auto nanoseconds = static_cast<long long>(event.unix_ns % 1'000'000'000);
  const double delay_ms = std::chrono::duration<double, std::milli>(
    InputGapState::Clock::now() - event.detected_at).count();

  if (event.gap.is_warning) {
    RCLCPP_WARN(logger_,
      "[Input gap] topic=%s stream=%s gap=%.1fms detected_at=%lld.%09lld log_delay=%.1fms",
      topic_.c_str(), stream, event.gap.gap_ms, seconds, nanoseconds, delay_ms);
  }

  if (event.gap.is_recovery) {
    RCLCPP_INFO(logger_,
      "[Input resumed] topic=%s stream=%s gap=%.1fms detected_at=%lld.%09lld log_delay=%.1fms",
      topic_.c_str(), stream, event.gap.gap_ms, seconds, nanoseconds, delay_ms);
  }
}

}  // namespace ai_sapiens_sim2real
