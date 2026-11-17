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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

void GaitClock::advance(PhaseState & state, const SharedControlData & shared) const
{
  state = update_(state, shared);
}

namespace
{

struct PhaseTiming
{
  float period;
  float swing_ratio;
};

struct ForwardBoundary
{
  float speed;
  bool inclusive;

  bool crossed_by(float command) const {return inclusive ? command >= speed : command > speed;}
};

struct LowSpeedRegion
{
  float planar_max;
  float yaw_max;
  float lateral_max;

  bool contains(const Eigen::Vector3f & command) const
  {
    return command.head<2>().norm() <= planar_max && std::abs(command.z()) <= yaw_max &&
           std::abs(command.y()) < lateral_max;
  }
};

struct PhaseActivationConfig
{
  LowSpeedRegion bounds;
  float planar_fade_width;
  float yaw_fade_width;
  float lateral_fade_width;
  float moving_fade_width;
  float rise_tau;
  float stand_threshold;
};

struct AdaptivePhaseConfig
{
  std::vector<PhaseTiming> timings;
  std::vector<ForwardBoundary> boundaries;
  std::optional<LowSpeedRegion> low_speed_region;
  float transition_tau;
  float reset_phase;
  std::optional<PhaseActivationConfig> activation;
};

// Configuration parsing and validation.

void require_phase_field(bool valid, const char * key, const char * requirement)
{
  if (!valid) {
    throw std::runtime_error("gait_phase." + std::string(key) + " " + requirement);
  }
}

template<typename T>
T read_phase_value(const YAML::Node & params, const char * key)
{
  require_phase_field(static_cast<bool>(params[key]), key, "is required");
  try {
    return params[key].as<T>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("Invalid gait_phase." + std::string(key) + ": " + error.what());
  }
}

float read_phase_scalar(const YAML::Node & params, const char * key)
{
  const float value = read_phase_value<float>(params, key);
  require_phase_field(std::isfinite(value), key, "must be finite");
  return value;
}

float read_positive_phase_scalar(const YAML::Node & params, const char * key)
{
  const float value = read_phase_scalar(params, key);
  require_phase_field(value > 0, key, "must be positive");
  return value;
}

std::vector<float> read_phase_vector(const YAML::Node & params, const char * key)
{
  auto values = read_phase_value<std::vector<float>>(params, key);
  for (float value : values) {
    require_phase_field(std::isfinite(value), key, "must contain finite values");
  }
  return values;
}

void validate_adaptive_phase_export(const YAML::Node & params)
{
  if (params["schema"]) {
    const auto schema = read_phase_value<std::string>(params, "schema");
    require_phase_field(schema == "k1_settle_march_phase_v1_command_only" ||
      schema == "k1_settle_march_phase_v1" || schema == "k1_sharedlow_four_range_phase_v1",
      "schema", "is unsupported");
  }
  for (const auto key : {"min_height", "min_upright", "recovery_delay_mask",
    "deployment_walk_period_scale"})
  {
    require_phase_field(!params[key], key, "is unsupported");
  }
  require_phase_field(read_phase_value<std::string>(params, "update") == "integrated_free_running",
    "update", "must be integrated_free_running");
  require_phase_field(read_phase_value<bool>(params, "stand_mode_priority"),
    "stand_mode_priority", "must be true");
  const std::vector<std::string> order = {
    "sin_left", "cos_left", "sin_right", "cos_right", "swing_left", "swing_right"};
  require_phase_field(read_phase_value<std::vector<std::string>>(params, "observation_order") == order,
    "observation_order", "must be [sin_left, cos_left, sin_right, cos_right, swing_left, swing_right]");
  // Only validate the contract here; observations encode raw state outside the clock.
  const auto offsets = read_phase_vector(params, "offsets");
  require_phase_field(offsets.size() == 2, "offsets", "must contain two values");
  for (float offset : offsets) {
    require_phase_field(offset >= 0 && offset < 1, "offsets", "must be in [0, 1)");
  }
  require_phase_field(read_phase_scalar(params, "stand_command_norm_plus_yaw_threshold") >= 0,
    "stand_command_norm_plus_yaw_threshold", "must be nonnegative");
}

std::vector<PhaseTiming> read_phase_timings(const YAML::Node & params)
{
  const auto periods = read_phase_vector(params, "periods");
  const auto swings = read_phase_vector(params, "swing_ratios");
  require_phase_field(!periods.empty(), "periods", "must not be empty");
  require_phase_field(swings.size() == periods.size(), "swing_ratios", "must match gait_phase.periods");
  std::vector<PhaseTiming> timings;
  for (std::size_t i = 0; i < periods.size(); ++i) {
    require_phase_field(periods[i] > 0, "periods", "must be positive");
    require_phase_field(swings[i] > 0 && swings[i] < 1, "swing_ratios", "must be in (0, 1)");
    timings.push_back({periods[i], swings[i]});
  }
  return timings;
}

std::vector<ForwardBoundary> read_forward_boundaries(const YAML::Node & params)
{
  const auto speeds = read_phase_vector(params, "forward_thresholds");
  // Preserve existing private exports; new exports provide each training comparator.
  const auto inclusive = params["forward_threshold_inclusive"] ?
    read_phase_value<std::vector<bool>>(params, "forward_threshold_inclusive") :
    std::vector<bool>(speeds.size(), true);
  require_phase_field(inclusive.size() == speeds.size(),
    "forward_threshold_inclusive", "must match forward_thresholds");
  std::vector<ForwardBoundary> boundaries;
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    require_phase_field(speeds[i] >= 0 && (i == 0 || speeds[i] > speeds[i - 1]),
      "forward_thresholds", "must be nonnegative and strictly increasing");
    boundaries.push_back({speeds[i], inclusive[i]});
  }
  return boundaries;
}

std::optional<LowSpeedRegion> read_low_speed_region(const YAML::Node & params)
{
  if (params["low_speed_priority"] && read_phase_value<bool>(params, "low_speed_priority")) {
    return LowSpeedRegion{
      read_positive_phase_scalar(params, "low_planar_max_inclusive"),
      read_positive_phase_scalar(params, "low_yaw_max_inclusive"),
      read_positive_phase_scalar(params, "low_lateral_max_exclusive")};
  }
  for (const auto key : {"low_planar_max_inclusive", "low_yaw_max_inclusive",
    "low_lateral_max_exclusive"})
  {
    require_phase_field(!params[key], key, "requires low_speed_priority");
  }
  return std::nullopt;
}

std::optional<PhaseActivationConfig> read_phase_activation(const YAML::Node & params)
{
  const auto region = params["activation_region"];
  if (!region) {return std::nullopt;}
  require_phase_field(region.IsMap(), "activation_region", "must be a map");
  require_phase_field(read_phase_value<std::string>(region, "kind") == "low_only_smoothstep",
    "activation_region.kind", "must be low_only_smoothstep");
  const PhaseActivationConfig config{
    {read_positive_phase_scalar(region, "planar_max"),
      read_positive_phase_scalar(region, "yaw_max"),
      read_positive_phase_scalar(region, "lateral_max")},
    read_positive_phase_scalar(region, "planar_fade_width"),
    read_positive_phase_scalar(region, "yaw_fade_width"),
    read_positive_phase_scalar(region, "lateral_fade_width"),
    read_positive_phase_scalar(region, "moving_fade_width"),
    read_positive_phase_scalar(region, "activation_rise_tau"),
    read_phase_scalar(params, "stand_command_norm_plus_yaw_threshold")};
  require_phase_field(config.planar_fade_width <= config.bounds.planar_max &&
    config.yaw_fade_width <= config.bounds.yaw_max &&
    config.lateral_fade_width <= config.bounds.lateral_max,
    "activation_region", "fade widths must not exceed their bounds");
  return config;
}

AdaptivePhaseConfig parse_adaptive_phase_config(const YAML::Node & params)
{
  validate_adaptive_phase_export(params);
  const AdaptivePhaseConfig config{
    read_phase_timings(params), read_forward_boundaries(params), read_low_speed_region(params),
    read_positive_phase_scalar(params, "transition_tau"), read_phase_scalar(params, "reset_phase"),
    read_phase_activation(params)};
  const auto range_count = config.boundaries.size() + 1 + config.low_speed_region.has_value();
  require_phase_field(config.timings.size() == range_count,
    "periods", "must match forward ranges plus optional low region");
  if (params["range_order"]) {
    require_phase_field(read_phase_value<std::vector<std::string>>(params, "range_order").size() ==
      range_count, "range_order", "must match periods");
  }
  require_phase_field(config.reset_phase >= 0 && config.reset_phase < 1,
    "reset_phase", "must be in [0, 1)");
  return config;
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
  if (!std::isfinite(period) || period <= 0.0f) {
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

// Phase calculation and clock construction.

float next_phase(float phase, float phase_increment)
{
  return std::fmod(phase + phase_increment, 1.0f);
}

GaitClock make_free_running_phase_clock(float increment)
{
  return GaitClock([increment](PhaseState state, const SharedControlData &) {
      state.phase = next_phase(state.phase, increment);
      return state;
    });
}

GaitClock make_command_reset_phase_clock(float increment, float threshold)
{
  return GaitClock([increment, threshold](PhaseState state, const SharedControlData & shared) {
      // The command-reset clock's first observation is at phase zero. This is a rule detail,
      // not a branch in the common clock.
      if (state.first_step) {
        state.first_step = false;
        return state;
      }
      state.phase = shared.mode.velocity_commands.norm() > threshold ?
        next_phase(state.phase, increment) : 0.0f;
      return state;
    });
}

float smoothstep(float value)
{
  value = std::clamp(value, 0.0f, 1.0f);
  return value * value * (3.0f - 2.0f * value);
}

float activation_target(const Eigen::Vector3f & command, const PhaseActivationConfig & config)
{
  const float planar = command.head<2>().norm();
  const float yaw = std::abs(command.z());
  return smoothstep((planar + yaw - config.stand_threshold) / config.moving_fade_width) *
         smoothstep((config.bounds.planar_max - planar) / config.planar_fade_width) *
         smoothstep((config.bounds.yaw_max - yaw) / config.yaw_fade_width) *
         smoothstep((config.bounds.lateral_max - std::abs(command.y())) /
           config.lateral_fade_width);
}

const PhaseTiming & select_phase_timing(
  const Eigen::Vector3f & command, const AdaptivePhaseConfig & config)
{
  if (config.low_speed_region && config.low_speed_region->contains(command)) {
    return config.timings.front();
  }
  std::size_t category = config.low_speed_region ? 1 : 0;
  for (const auto & boundary : config.boundaries) {
    category += boundary.crossed_by(command.x());
  }
  return config.timings[category];
}

PhaseState advance_adaptive_phase(
  PhaseState state, const Eigen::Vector3f & command, const AdaptivePhaseConfig & config,
  float dt, float alpha)
{
  if (state.first_step) {
    state.first_step = false;
    return state;
  }
  if (config.activation) {
    const float target = activation_target(command, *config.activation);
    const float rise_alpha = 1.0f - std::exp(-dt / config.activation->rise_tau);
    // Fade out immediately to the command envelope; rise gradually on re-entry.
    state.activation = std::min(
      state.activation + rise_alpha * (target - state.activation), target);
  }
  const auto & target = select_phase_timing(command, config);
  state.period += alpha * (target.period - state.period);
  state.swing_ratio += alpha * (target.swing_ratio - state.swing_ratio);
  state.phase = next_phase(state.phase, dt / state.period);
  return state;
}

GaitClock make_adaptive_phase_clock(const AdaptivePhaseConfig & config, double step_dt)
{
  const auto & first = config.timings.front();
  const PhaseState initial{config.reset_phase, first.period, first.swing_ratio, true,
    config.activation ? 0.0f : 1.0f};
  const float dt = static_cast<float>(step_dt);
  const float alpha = 1.0f - std::exp(-dt / config.transition_tau);
  return GaitClock([config, dt, alpha](PhaseState state, const SharedControlData & shared) {
      return advance_adaptive_phase(state, shared.mode.velocity_commands, config, dt, alpha);
    }, initial, /*resolve_command_before_update=*/true);
}

}  // namespace

std::optional<GaitClock> make_gait_clock(const YAML::Node & observations, double step_dt)
{
  // Existing exports call this observation "gait_phase"; retain that YAML ABI.
  const auto term = observations["gait_phase"];
  if (!term) {
    return std::nullopt;
  }

  const auto params = term["params"];
  if (!std::isfinite(step_dt) || step_dt <= 0) {
    throw std::runtime_error("gait_phase requires positive finite step_dt");
  }
  if (!params || !params.IsMap()) {throw std::runtime_error("gait_phase requires params map");}
  if (params["periods"] || params["schema"]) {
    return make_adaptive_phase_clock(parse_adaptive_phase_config(params), step_dt);
  }
  require_phase_field(!params["activation_region"], "activation_region", "requires periods");
  const float increment = static_cast<float>(step_dt) / require_period(params);
  validate_offset(params);

  const auto threshold = command_threshold(params);
  if (threshold.has_value()) {
    // The command-reset rule evaluates its first observation at phase zero, then advances by one
    // policy step. Keep that ordering even though PolicyRuntime calls advance()
    // immediately before constructing the observation.
    return make_command_reset_phase_clock(increment, *threshold);
  }

  return make_free_running_phase_clock(increment);
}

}  // namespace ai_sapiens_sim2real
