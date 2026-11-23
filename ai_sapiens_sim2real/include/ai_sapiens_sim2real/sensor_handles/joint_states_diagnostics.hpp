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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__JOINT_STATES_DIAGNOSTICS_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__JOINT_STATES_DIAGNOSTICS_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ai_sapiens_sim2real
{

enum class JointSampleStatus {Valid, MissingJoint, MissingValues, InvalidValues};

// The serial input callback records snapshots; the node executor formats and logs them.
class JointStatesDiagnostics
{
public:
  JointStatesDiagnostics(
    const rclcpp::Node::SharedPtr & node, const std::string & topic,
    const std::vector<std::string> & joint_names);

  void record_incomplete(
    const std::vector<JointSampleStatus> & joint_status,
    size_t complete_joints);
  void record_ready();

private:
  using Clock = std::chrono::steady_clock;
  static constexpr size_t kMaxMissingJointsToLog = 8;

  struct MissingJoint
  {
    size_t joint_index{0};
    JointSampleStatus status{JointSampleStatus::MissingJoint};
  };

  struct IncompleteSample
  {
    size_t complete_joints{0};
    std::array<MissingJoint, kMaxMissingJointsToLog> missing_joints{};
    size_t missing_count{0};
    Clock::time_point detected_at{};
    int64_t unix_ns{0};
    bool has_more_missing_joints{false};
  };

  IncompleteSample capture_incomplete_sample(
    const std::vector<JointSampleStatus> & joint_status,
    size_t complete_joints, Clock::time_point now) const;
  bool store_pending_warning(const IncompleteSample & sample);
  std::optional<IncompleteSample> take_pending_warning();
  std::string build_missing_joint_summary(const IncompleteSample & sample) const;
  void report_incomplete(const IncompleteSample & sample);
  void flush();

  rclcpp::Logger logger_;
  std::string topic_;
  std::vector<std::string> joint_names_;
  Clock::time_point next_warning_at_{};  // Input callback only.
  std::mutex mutex_;
  std::optional<IncompleteSample> pending_warning_;
  std::atomic<bool> is_ready_{false};
  bool has_logged_ready_{false};  // Node executor only.
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__JOINT_STATES_DIAGNOSTICS_HPP_
