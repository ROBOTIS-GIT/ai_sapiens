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

void GaitClock::advance(float & phase, const SharedControlData & shared) const
{
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

// Standing-aware rules and their config dispatch are deactivated until the
// training export emits these params. Kept here; to re-enable, uncomment this
// block and call select_update(params, increment) from make_gait_clock below
// instead of free_running(increment).

/*
bool is_standing(const SharedControlData & shared, float lin_threshold, float yaw_threshold)
{
  const auto & cmd = shared.mode.velocity_commands;
  return std::hypot(cmd.x(), cmd.y()) < lin_threshold && std::abs(cmd.z()) < yaw_threshold;
}

GaitPhaseUpdate freeze_when_standing(float increment, float lin_thr, float yaw_thr)
{
  return [increment, lin_thr, yaw_thr](float phase, const SharedControlData & shared) {
           return is_standing(shared, lin_thr, yaw_thr) ? phase : next_phase(phase, increment);
         };
}

GaitPhaseUpdate reset_when_standing(float increment, float lin_thr, float yaw_thr)
{
  return [increment, lin_thr, yaw_thr](float phase, const SharedControlData & shared) {
           return is_standing(shared, lin_thr, yaw_thr) ? 0.0f : next_phase(phase, increment);
         };
}

float read_threshold(const YAML::Node & params, const char * key)
{
  const auto node = params[key];
  return (node && !node.IsNull()) ? node.as<float>() : 0.0f;
}

GaitPhaseUpdate select_update(const YAML::Node & params, float increment)
{
  const std::string mode =
    (params["update"] && !params["update"].IsNull()) ?
    params["update"].as<std::string>() : "free_running";

  if (mode == "free_running") {
    return free_running(increment);
  }

  const float lin_thr = read_threshold(params, "lin_thresh");
  const float yaw_thr = read_threshold(params, "yaw_thresh");
  if (mode == "freeze_when_standing") {
    return freeze_when_standing(increment, lin_thr, yaw_thr);
  }
  if (mode == "reset_when_standing") {
    return reset_when_standing(increment, lin_thr, yaw_thr);
  }

  throw std::runtime_error("gait_phase: unknown update rule '" + mode + "'");
}

*/

}  // namespace

std::optional<GaitClock> make_gait_clock(const YAML::Node & observations, double step_dt)
{
  const auto term = observations["gait_phase"];
  if (!term) {
    return std::nullopt;
  }

  const auto params = term["params"];
  const float increment = static_cast<float>(step_dt) / require_period(params);

  return GaitClock(free_running(increment));
}

}  // namespace ai_sapiens_sim2real
