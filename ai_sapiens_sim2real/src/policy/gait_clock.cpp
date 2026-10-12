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

#include "ai_sapiens_sim2real/policy/gait_clock.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

void GaitClock::advance(float & phase, const SharedControlData & shared)
{
  if (hold_first_step_ && first_step_) {
    first_step_ = false;
    return;
  }
  first_step_ = false;
  phase = update_(phase, shared);
}

namespace
{

float next_phase(float phase, float phase_increment)
{
  return std::fmod(phase + phase_increment, 1.0f);
}

GaitPhaseUpdate free_running(float increment)
{
  return [increment](float phase, const SharedControlData &) {
           return next_phase(phase, increment);
         };
}

GaitPhaseUpdate reset_when_command_is_stationary(float increment, float threshold)
{
  return [increment, threshold](float phase, const SharedControlData & shared) {
           return shared.mode.velocity_commands.norm() > threshold ?
                  next_phase(phase, increment) : 0.0f;
         };
}

float require_period(const YAML::Node & params)
{
  if (!params || params.IsNull()) {
    throw std::runtime_error("gait_phase observation requires params");
  }
  if (!params.IsMap()) {
    throw std::runtime_error("gait_phase observation params must be a map");
  }

  const auto period_node = params["period"];
  if (!period_node || period_node.IsNull()) {
    throw std::runtime_error("gait_phase observation requires params.period (seconds per cycle)");
  }

  const float period = period_node.as<float>();
  if (period <= 0.0f) {
    throw std::runtime_error("gait_phase period must be > 0");
  }

  return period;
}

std::optional<float> command_threshold(const YAML::Node & params)
{
  const auto threshold_node = params["command_threshold"];
  if (!threshold_node || threshold_node.IsNull()) {
    return std::nullopt;
  }

  const float threshold = threshold_node.as<float>();
  if (!std::isfinite(threshold) || threshold < 0.0f) {
    throw std::runtime_error("gait_phase command_threshold must be finite and >= 0");
  }
  return threshold;
}

void validate_offset(const YAML::Node & params)
{
  const auto offset_node = params["offset"];
  if (!offset_node || offset_node.IsNull()) {
    return;
  }

  const float offset = offset_node.as<float>();
  if (!std::isfinite(offset) || offset < 0.0f || offset >= 1.0f) {
    throw std::runtime_error("gait_phase offset must be finite and in [0, 1)");
  }
}

}  // namespace

std::optional<GaitClock> make_gait_clock(const YAML::Node & observations, double step_dt)
{
  const auto term = observations["gait_phase"];
  if (!term) {
    return std::nullopt;
  }

  const auto params = term["params"];
  const float increment = static_cast<float>(step_dt) / require_period(params);
  validate_offset(params);

  const auto threshold = command_threshold(params);
  if (threshold.has_value()) {
    // DWAQ evaluates its first observation at phase zero, then advances by one
    // policy step. Keep that ordering even though PolicyRuntime calls advance()
    // immediately before constructing the observation.
    return GaitClock(reset_when_command_is_stationary(increment, *threshold), true);
  }

  return GaitClock(free_running(increment));
}

}  // namespace ai_sapiens_sim2real
