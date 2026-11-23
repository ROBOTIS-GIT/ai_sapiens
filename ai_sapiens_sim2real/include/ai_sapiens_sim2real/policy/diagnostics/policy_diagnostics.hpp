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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__POLICY_DIAGNOSTICS_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__POLICY_DIAGNOSTICS_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/logger.hpp>

#include "ai_sapiens_sim2real/policy/diagnostics/policy_fault_capture.hpp"
#include "ai_sapiens_sim2real/policy/policy_action_result.hpp"

namespace ai_sapiens_sim2real
{
// Owns diagnostic context and presentation, never control or retry decisions.
// Terminal diagnostics remain available without a file recorder.
class PolicyDiagnostics
{
public:
  PolicyDiagnostics(FaultPolicyMetadata metadata, rclcpp::Logger logger);
  void set_recorder(std::shared_ptr<ActionFaultRecorder> recorder) noexcept;
  void reset() noexcept;
  void accumulate_sensor_interval(double period) noexcept;
  void begin_capture(
    double episode_time, const SensorData & sensors,
    const FaultExecutionContext & context = {}) noexcept;
  void capture_input(const std::unordered_map<std::string, std::vector<float>> & inputs) noexcept;
  void capture_output(const std::vector<float> & output) noexcept;
  void finish_capture(
    const PolicyActionResult & result, const SensorData & sensors,
    const PolicyActionDecision & decision) noexcept;
  void flush_diagnostics();

private:
  void log_result(
    const PolicyActionResult & result, const SensorData & sensors,
    const PolicyActionDecision & decision, const std::string & path) const;
  void log_action_limit(
    const PolicyActionResult & result, const SensorData & sensors,
    const PolicyActionDecision & decision, const std::string & path) const;
  std::shared_ptr<const FaultPolicyMetadata> metadata_;
  rclcpp::Logger logger_;
  std::unique_ptr<PolicyFaultCapture> capture_;
  std::optional<SensorData> previous_sensors_;
  std::atomic<bool> report_error_pending_{false};
  double sensor_interval_{0.0};
  std::string active_state_;
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__POLICY_DIAGNOSTICS_HPP_
