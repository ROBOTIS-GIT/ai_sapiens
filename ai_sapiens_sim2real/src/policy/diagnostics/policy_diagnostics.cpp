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

#include "ai_sapiens_sim2real/policy/diagnostics/policy_diagnostics.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <rclcpp/logging.hpp>
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_diagnostics.hpp"

namespace ai_sapiens_sim2real
{
PolicyDiagnostics::PolicyDiagnostics(
  FaultPolicyMetadata metadata, rclcpp::Logger logger)
: metadata_(std::make_shared<FaultPolicyMetadata>(std::move(metadata))),
  logger_(std::move(logger)) {}

void PolicyDiagnostics::set_recorder(std::shared_ptr<ActionFaultRecorder> recorder) noexcept
{
  capture_.reset();

  if (!recorder) {return;}

  // Configuration IO happens only during startup attachment, never in update().
  try {
    auto metadata = std::make_shared<FaultPolicyMetadata>(*metadata_);
    std::ifstream config(metadata->config_path);
    metadata->config_yaml.assign(std::istreambuf_iterator<char>(config), {});
    capture_ = std::make_unique<PolicyFaultCapture>(
      std::move(recorder), std::move(metadata), logger_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Cannot prepare fault observation capture: %s", e.what());
  }
}

void PolicyDiagnostics::reset() noexcept
{
  if (capture_) {capture_->reset();}
  previous_sensors_.reset();
  sensor_interval_ = 0.0;
}

void PolicyDiagnostics::accumulate_sensor_interval(double period) noexcept
{
  if (std::isfinite(period) && period > 0.0) {sensor_interval_ += period;}
}

void PolicyDiagnostics::begin_capture(
  double episode_time, const SensorData & sensors, const FaultExecutionContext & context) noexcept
{
  try {
    active_state_ = context.state.empty() ? metadata_->state : context.state;
  } catch (...) {
    active_state_.clear();
  }

  if (capture_) {
    capture_->begin_capture(episode_time, sensors, previous_sensors_, sensor_interval_, context);
  }
}

void PolicyDiagnostics::capture_input(
  const std::unordered_map<std::string, std::vector<float>> & inputs) noexcept
{
  if (capture_) {capture_->capture_input(inputs);}
}

void PolicyDiagnostics::capture_output(const std::vector<float> & output) noexcept
{
  if (capture_) {capture_->capture_output(output);}
}

void PolicyDiagnostics::finish_capture(
  const PolicyActionResult & result, const SensorData & sensors,
  const PolicyActionDecision & decision) noexcept
{
  try {
    std::string path;

    if (capture_) {
      path = capture_->finish_capture(result, decision);
    }

    log_result(result, sensors, decision, path);
    previous_sensors_ = sensors;
    sensor_interval_ = 0.0;
  } catch (...) {
    // Diagnostic allocation/formatting failures must never change control.
    previous_sensors_.reset();
    sensor_interval_ = 0.0;
    RCLCPP_ERROR(logger_, "Cannot report policy diagnostics; control continues");
  }
}

void PolicyDiagnostics::log_result(
  const PolicyActionResult & result, const SensorData & sensors,
  const PolicyActionDecision & decision, const std::string & path) const
{
  if (result.status == PolicyActionStatus::ActionRejected) {
    log_action_limit(result, sensors, decision, path);
    return;
  }

  if (result.status == PolicyActionStatus::InferenceFailed) {
    RCLCPP_WARN(logger_,
      "Policy '%s' inference failed (%zu/%zu consecutive unusable actions): %s",
      (active_state_.empty() ? metadata_->state : active_state_).c_str(), decision.failures, decision.max_failures, result.error.c_str());

    if (decision.outcome == PolicyActionOutcome::Damping) {
      RCLCPP_ERROR(logger_,
        "Policy '%s' failed to produce a usable action; requesting damping failsafe",
        (active_state_.empty() ? metadata_->state : active_state_).c_str());
    }
  } else if (decision.previous_failures != 0) {
    RCLCPP_INFO(logger_,
      "Policy '%s' recovered after %zu unusable action(s); accepting target and resetting retries",
      (active_state_.empty() ? metadata_->state : active_state_).c_str(), decision.previous_failures);
  }

  if (!path.empty()) {RCLCPP_INFO(logger_, "Inference diagnostics queued: %s", path.c_str());}
}

void PolicyDiagnostics::log_action_limit(
  const PolicyActionResult & result, const SensorData & sensors,
  const PolicyActionDecision & decision, const std::string & path) const
{
  std::ostringstream report;
  report << "ACTION LIMIT | " << decision.failures << "/" << decision.max_failures
         << " | " << action_outcome_label(decision.outcome)
         << "\n\n--- ACTION: rejected target ---\n"
         << "Processed position [rad, policy coordinates]; change limit="
         << metadata_->action_change_limit * 180.0f / 3.14159265f << " deg"
         << " AND |raw action| >= " << metadata_->raw_action_boundary
         << " (" << boundary_check_label(result.validation.boundary_check) << ")\n"
         << "  " << std::left << std::setw(16) << "joint"
         << " " << std::right << std::setw(10) << "processed" << "   previous      delta flag\n"
         << format_action_fault_actions(result.processed, metadata_->policy_joints,
      result.validation, true);
  const auto jumps = format_action_fault_sensors(sensors,
    previous_sensors_, metadata_->controller_joints, sensor_interval_, true);

  if (!jumps.empty()) {
    report << "\n--- SENSOR: feedback jumps ---\n"
           << "Physical sensor coordinates; JUMP only\n" << jumps;
  }

  report << "\n[FILE] " <<
    (path.empty() ? "not queued (recording unavailable)" : path + " (queued)");

  if (decision.outcome == PolicyActionOutcome::Damping) {
    RCLCPP_ERROR(logger_, "%s", report.str().c_str());
  } else {
    RCLCPP_WARN(logger_, "%s", report.str().c_str());
  }
}

}  // namespace ai_sapiens_sim2real
