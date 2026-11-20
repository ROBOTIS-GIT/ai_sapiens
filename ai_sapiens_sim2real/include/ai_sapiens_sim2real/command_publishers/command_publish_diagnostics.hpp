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

#ifndef AI_SAPIENS_SIM2REAL__COMMAND_PUBLISHERS__COMMAND_PUBLISH_DIAGNOSTICS_HPP_
#define AI_SAPIENS_SIM2REAL__COMMAND_PUBLISHERS__COMMAND_PUBLISH_DIAGNOSTICS_HPP_

#include <rcutils/error_handling.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include <rclcpp/logger.hpp>
#include <realtime_tools/lock_free_queue.hpp>

namespace ai_sapiens_sim2real
{

struct CommandPublishError
{
  std::string_view message;
  std::optional<int> code;
};

class CommandPublishDiagnostics
{
public:
  void record_first_failure(bool latched, CommandPublishError error);
  void record_latch(std::uint64_t failures, CommandPublishError error);
  void record_recovery(std::uint64_t failures, bool latched);
  void record_emergency_failure(CommandPublishError error);

  // Consumed by the node executor and once more after the control loop stops.
  void flush(const rclcpp::Logger & logger, const std::string & topic, unsigned int max_retries);

private:
  enum class Kind {FirstFailure, Latched, Recovered, EmergencyFailure};

  struct Event
  {
    Kind kind;
    std::uint64_t failures;
    bool latched;
    std::optional<int> error_code;
    std::array<char, RCUTILS_ERROR_MESSAGE_MAX_LENGTH> message;
  };

  void enqueue(Kind kind, std::uint64_t failures, bool latched, CommandPublishError error = {});
  static void log_event(
    const Event & event, const rclcpp::Logger & logger, const char * topic,
    unsigned int max_retries);

  // One ordinary failure/recovery pair, the latch and its recovery, and the final emergency.
  realtime_tools::LockFreeSPSCQueue<Event, 5> events_;
  // Includes the event currently being printed by the consumer.
  std::atomic<unsigned int> unreported_events_{0};
  bool report_recovery_{false};
  std::chrono::steady_clock::time_point next_failure_log_at_{
    std::chrono::steady_clock::time_point::min()};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__COMMAND_PUBLISHERS__COMMAND_PUBLISH_DIAGNOSTICS_HPP_
