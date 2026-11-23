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

#include "ai_sapiens_sim2real/policy/diagnostics/policy_fault_capture.hpp"

#include <utility>

#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_recorder.hpp"

namespace ai_sapiens_sim2real
{
namespace
{
void copy_inputs(
  const std::unordered_map<std::string, std::vector<float>> & source,
  std::unordered_map<std::string, std::vector<float>> & destination)
{
  for (auto it = destination.begin(); it != destination.end(); ) {
    if (source.find(it->first) == source.end()) {
      it = destination.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto & [name, values] : source) {
    auto & stored_values = destination[name];
    stored_values.assign(values.begin(), values.end());
  }
}
}  // namespace

PolicyFaultCapture::PolicyFaultCapture(
  std::shared_ptr<ActionFaultRecorder> recorder,
  std::shared_ptr<const FaultPolicyMetadata> metadata)
: recorder_(std::move(recorder)), metadata_(std::move(metadata)) {}

void PolicyFaultCapture::reset() noexcept
{
  reset_current_snapshot_for_reuse();
  current_.previous_sensors.reset();
  previous_accepted_.reset();
  incident_ = 0;
  attempt_ = 0;
}

bool PolicyFaultCapture::take_error() noexcept
{
  return error_pending_.exchange(false, std::memory_order_acq_rel);
}

void PolicyFaultCapture::disable() noexcept
{
  is_enabled_ = false;
  error_pending_.store(true, std::memory_order_release);
}

void PolicyFaultCapture::reset_current_snapshot_for_reuse() noexcept
{
  // Preserve dynamic sensor and inference-call storage. begin_capture()
  // overwrites sensor values, while captured_call_count_ marks the active
  // prefix of the retained call buffer.
  current_.context.state.clear();
  current_.context.behavior.clear();
  current_.context.motion_file.clear();
  current_.context.motion_frame.reset();
  current_.unix_time_ns = 0;
  current_.episode_time = 0.0;
  current_.sensor_interval = 0.0;
  current_.processed_action.clear();
  current_.validation.boundary_check = BoundaryCheckState::Disabled;
  current_.validation.joints.clear();
  current_.error.clear();
  current_.outcome = PolicyActionOutcome::Retry;
  current_.consecutive_failures = 0;
  captured_call_count_ = 0;
}

void PolicyFaultCapture::begin_capture(
  double episode_time, const SensorData & sensors,
  const std::optional<SensorData> & previous, double interval,
  const FaultExecutionContext & context) noexcept
{
  if (!is_enabled_) {return;}
  try {
    reset_current_snapshot_for_reuse();
    current_.unix_time_ns = system_clock_.now().nanoseconds();
    current_.episode_time = episode_time;
    current_.context = context;
    current_.sensors = sensors;
    if (previous) {
      if (current_.previous_sensors) {
        *current_.previous_sensors = *previous;
      } else {
        current_.previous_sensors.emplace(*previous);
      }
    } else {
      current_.previous_sensors.reset();
    }
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
    if (captured_call_count_ == current_.calls.size()) {
      current_.calls.emplace_back();
    }

    auto & call = current_.calls[captured_call_count_++];
    copy_inputs(inputs, call.inputs);
    call.output.clear();
  } catch (...) {
    disable();
  }
}

void PolicyFaultCapture::capture_output(const std::vector<float> & output) noexcept
{
  if (!is_enabled_) {return;}
  try {
    if (captured_call_count_ == 0) {
      disable();
      return;
    }

    auto & stored_output = current_.calls[captured_call_count_ - 1].output;
    stored_output.assign(output.begin(), output.end());
  } catch (...) {
    disable();
  }
}

void PolicyFaultCapture::complete_current_snapshot(
  const PolicyActionResult & result, const PolicyActionDecision & decision)
{
  current_.calls.resize(captured_call_count_);
  current_.processed_action = result.processed;
  current_.validation = result.validation;
  current_.error = result.error;
  current_.consecutive_failures = decision.failures;
  current_.outcome = decision.outcome;
}

void PolicyFaultCapture::store_current_as_previous_accepted()
{
  if (previous_accepted_) {
    std::swap(*previous_accepted_, current_);
  } else {
    previous_accepted_.emplace(std::move(current_));
  }
}

std::string PolicyFaultCapture::record_incident_attempt(
  const PolicyActionDecision & decision)
{
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
  if (decision.outcome == PolicyActionOutcome::Accepted) {
    previous_accepted_ = record.current;
  }

  const auto path = recorder_->enqueue(std::move(record));
  if (decision.outcome != PolicyActionOutcome::Retry) {
    incident_ = 0;
    attempt_ = 0;
  }

  return path;
}

std::string PolicyFaultCapture::finish_capture(
  const PolicyActionResult & result, const PolicyActionDecision & decision) noexcept
{
  if (!is_enabled_) {return {};}

  try {
    complete_current_snapshot(result, decision);

    if (decision.outcome == PolicyActionOutcome::Accepted && incident_ == 0) {
      store_current_as_previous_accepted();
      return {};
    }

    return record_incident_attempt(decision);
  } catch (...) {
    disable();
    return {};
  }
}
}  // namespace ai_sapiens_sim2real
