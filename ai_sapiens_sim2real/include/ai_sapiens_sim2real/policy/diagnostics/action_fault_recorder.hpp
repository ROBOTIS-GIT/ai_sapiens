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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__ACTION_FAULT_RECORDER_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__ACTION_FAULT_RECORDER_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/logger.hpp>
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_record.hpp"

namespace ai_sapiens_sim2real
{
// One writer per controller, shared by all loaded policies. No filesystem IO
// or serialization in enqueue(); an unavailable queue drops diagnostics only.
class ActionFaultRecorder
{
public:
  struct Options
  {
    std::filesystem::path directory;
    size_t max_storage_bytes{256 * 1024 * 1024};
    size_t max_queue_bytes{32 * 1024 * 1024};
    size_t max_queue_records{16};
  };
  ActionFaultRecorder(Options options, rclcpp::Logger logger);
  ~ActionFaultRecorder();
  static std::filesystem::path default_directory();
  uint64_t next_incident() {return ++incident_sequence_;}
  std::string enqueue(ActionFaultRecord record) noexcept;
  // Stable planned path; materialized by the writer only for the first record.
  const std::filesystem::path & session_directory() const {return session_directory_;}

private:
  void write_loop() noexcept;
  void write_record(const ActionFaultRecord & record);
  void create_session_directory();
  Options options_;
  rclcpp::Logger logger_;
  std::filesystem::path session_directory_;
  std::atomic<uint64_t> incident_sequence_{0};
  std::atomic<bool> is_enabled_{true};
  std::atomic<size_t> dropped_records_{0};
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<ActionFaultRecord> queue_;
  size_t queued_bytes_{0}, stored_bytes_{0};
  bool is_stopping_{false};
  bool is_session_created_{false};  // Writer thread only.
  std::thread worker_;
};
}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__ACTION_FAULT_RECORDER_HPP_
