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

#include "ai_sapiens_sim2real/policy/policy_fault_capture.hpp"

#include <utility>
#include <rclcpp/logging.hpp>
#include "ai_sapiens_sim2real/policy/action_fault_recorder.hpp"

namespace ai_sapiens_sim2real
{
PolicyFaultCapture::PolicyFaultCapture(
  std::shared_ptr<ActionFaultRecorder> recorder,
  std::shared_ptr<const FaultPolicyMetadata> metadata, rclcpp::Logger logger)
: recorder_(std::move(recorder)), metadata_(std::move(metadata)), logger_(std::move(logger)) {}

void PolicyFaultCapture::reset() noexcept
{
  current_ = {};
  previous_accepted_.reset();
  incident_ = 0;
  attempt_ = 0;
}

void PolicyFaultCapture::disable() noexcept
{
  is_enabled_ = false;
  RCLCPP_ERROR(logger_,
      "Observation capture disabled after diagnostic capture error; control continues");
}

void PolicyFaultCapture::begin_capture(
  double episode_time, const SensorData & sensors,
  const std::optional<SensorData> & previous, double interval) noexcept
{
  if (!is_enabled_) {return;}
  try {
    current_ = {};
    current_.unix_time_ns = system_clock_.now().nanoseconds();
    current_.episode_time = episode_time;
    current_.sensors = sensors;
    current_.previous_sensors = previous;
    current_.sensor_interval = interval;
  } catch (...) {
    disable();
  }
}

void PolicyFaultCapture::capture_input(
  const std::unordered_map<std::string, std::vector<float>> & inputs) noexcept
{
  if (!is_enabled_) {return;}
  try {
    current_.calls.push_back({inputs, {}});
  } catch (...) {
    disable();
  }
}

void PolicyFaultCapture::capture_output(const std::vector<float> & output) noexcept
{
  if (!is_enabled_) {return;}
  try {
    current_.calls.back().output = output;
  } catch (...) {
    disable();
  }
}

std::string PolicyFaultCapture::finish_capture(
  const PolicyActionResult & result, const PolicyActionDecision & decision) noexcept
{
  if (!is_enabled_) {return {};}

  try {
    current_.processed_action = result.processed;
    current_.validation = result.validation;
    current_.error = result.error;
    current_.consecutive_failures = decision.failures;
    current_.outcome = decision.outcome;
    const bool is_accepted = decision.outcome == PolicyActionOutcome::Accepted;

    if (is_accepted && incident_ == 0) {
      previous_accepted_ = std::move(current_);
      return {};
    }

    if (incident_ == 0) {
      incident_ = recorder_->next_incident();
      attempt_ = 0;
    }

    ActionFaultRecord record;
    record.metadata = metadata_;
    record.incident = incident_;
    record.attempt = ++attempt_;
    record.current = std::move(current_);

    if (attempt_ == 1) {record.previous_accepted = previous_accepted_;}
    if (is_accepted) {previous_accepted_ = record.current;}
    const auto path = recorder_->enqueue(std::move(record));

    if (decision.outcome != PolicyActionOutcome::Retry) {
      incident_ = 0;
      attempt_ = 0;
    }

    return path;
  } catch (...) {
    disable();
    return {};
  }
}
}  // namespace ai_sapiens_sim2real
