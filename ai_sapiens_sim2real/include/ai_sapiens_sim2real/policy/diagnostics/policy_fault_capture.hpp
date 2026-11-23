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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__POLICY_FAULT_CAPTURE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__POLICY_FAULT_CAPTURE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/clock.hpp>

#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_record.hpp"

namespace ai_sapiens_sim2real
{
class ActionFaultRecorder;

// Control-thread capture, independent of action validation and retry decisions.
// Copies the actual ONNX input before inference.
class PolicyFaultCapture
{
public:
  PolicyFaultCapture(
    std::shared_ptr<ActionFaultRecorder> recorder,
    std::shared_ptr<const FaultPolicyMetadata> metadata);
  void reset() noexcept;
  bool take_error() noexcept;
  void begin_capture(
    double episode_time, const SensorData & sensors,
    const std::optional<SensorData> & previous, double interval,
    const FaultExecutionContext & context = {}) noexcept;
  void capture_input(const std::unordered_map<std::string, std::vector<float>> & inputs) noexcept;
  void capture_output(const std::vector<float> & output) noexcept;
  std::string finish_capture(
    const PolicyActionResult & result, const PolicyActionDecision & decision) noexcept;

private:
  void disable() noexcept;
  void reset_current_snapshot_for_reuse() noexcept;
  void complete_current_snapshot(
    const PolicyActionResult & result, const PolicyActionDecision & decision);
  void store_current_as_previous_accepted();
  std::string record_incident_attempt(const PolicyActionDecision & decision);

  std::shared_ptr<ActionFaultRecorder> recorder_;
  std::shared_ptr<const FaultPolicyMetadata> metadata_;
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
  FaultSnapshot current_;
  std::optional<FaultSnapshot> previous_accepted_;
  std::atomic<bool> error_pending_{false};
  uint64_t incident_{0};
  size_t attempt_{0};
  size_t captured_call_count_{0};
  bool is_enabled_{true};
};
}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__POLICY_FAULT_CAPTURE_HPP_
