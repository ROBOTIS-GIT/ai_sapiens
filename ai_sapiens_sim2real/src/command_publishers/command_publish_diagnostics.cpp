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

#include "ai_sapiens_sim2real/command_publishers/command_publish_diagnostics.hpp"

#include <cinttypes>

#include <rclcpp/logging.hpp>

namespace ai_sapiens_sim2real
{

void CommandPublishDiagnostics::record_first_failure(bool latched, CommandPublishError error)
{
  report_recovery_ = unreported_events_.load(std::memory_order_relaxed) == 0 &&
    std::chrono::steady_clock::now() >= next_failure_log_at_;
  if (report_recovery_) {
    enqueue(Kind::FirstFailure, 1, latched, error);
  }
}

void CommandPublishDiagnostics::record_latch(std::uint64_t failures, CommandPublishError error)
{
  enqueue(Kind::Latched, failures, true, error);
  report_recovery_ = true;
}

void CommandPublishDiagnostics::record_recovery(std::uint64_t failures, bool latched)
{
  if (report_recovery_) {
    enqueue(Kind::Recovered, failures, latched);
    report_recovery_ = false;
    next_failure_log_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  }
}

void CommandPublishDiagnostics::record_emergency_failure(CommandPublishError error)
{
  enqueue(Kind::EmergencyFailure, 0, false, error);
}

void CommandPublishDiagnostics::enqueue(
  Kind kind, std::uint64_t failures, bool latched, CommandPublishError error)
{
  Event event{kind, failures, latched, error.code, {}};
  if (!error.message.empty()) {
    error.message.copy(event.message.data(), event.message.size() - 1);
  }
  unreported_events_.fetch_add(1, std::memory_order_relaxed);
  if (!events_.push(event)) {
    unreported_events_.fetch_sub(1, std::memory_order_relaxed);
  }
}

void CommandPublishDiagnostics::flush(
  const rclcpp::Logger & logger, const std::string & topic, unsigned int max_retries)
{
  const auto pending = unreported_events_.load(std::memory_order_relaxed);
  Event event;
  for (unsigned int i = 0; i < pending && events_.pop(event); ++i) {
    log_event(event, logger, topic.c_str(), max_retries);
    unreported_events_.fetch_sub(1, std::memory_order_relaxed);
  }
}

void CommandPublishDiagnostics::log_event(
  const Event & event, const rclcpp::Logger & logger, const char * topic,
  unsigned int max_retries)
{
  const auto code = event.error_code ? ", rcl=" + std::to_string(*event.error_code) : "";
  const char * result = event.error_code ? "failed" : "unconfirmed";
  switch (event.kind) {
    case Kind::FirstFailure:
      if (event.latched) {
        RCLCPP_WARN(logger, "Joint command publish %s while Damping is latched "
          "(topic=%s%s): %s", result, topic, code.c_str(), event.message.data());
      } else {
        RCLCPP_WARN(logger, "Joint command publish %s on attempt 1 (topic=%s%s): %s. "
          "Will retry on the next control tick, up to %u more attempts",
          result, topic, code.c_str(), event.message.data(), max_retries);
      }
      break;
    case Kind::Latched:
      RCLCPP_ERROR(logger, "Damping latched after %" PRIu64 " consecutive joint command publish "
        "failures (topic=%s%s): %s", event.failures, topic, code.c_str(), event.message.data());
      break;
    case Kind::Recovered:
      RCLCPP_INFO(logger, "Joint command publish recovered on attempt %" PRIu64
        " (retry %" PRIu64 ", topic=%s)%s", event.failures + 1, event.failures, topic,
        event.latched ? ". Damping remains latched" : "");
      break;
    case Kind::EmergencyFailure:
      RCLCPP_ERROR(logger, "Emergency Damping publish %s (topic=%s%s): %s",
        result, topic, code.c_str(), event.message.data());
      break;
  }
}

}  // namespace ai_sapiens_sim2real
