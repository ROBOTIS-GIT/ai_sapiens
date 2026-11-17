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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "ai_sapiens_sim2real/policy/gait_clock.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

using ai_sapiens_sim2real::make_gait_clock;
using ai_sapiens_sim2real::SharedControlData;
using ai_sapiens_sim2real::PhaseState;

namespace
{
YAML::Node observations_with_period(double period)
{
  YAML::Node observations;
  observations["gait_phase"]["params"]["period"] = period;
  return observations;
}
}  // namespace

TEST(GaitClock, NoGaitPhaseObservationYieldsNullopt)
{
  YAML::Node observations;  // no gait_phase term
  EXPECT_FALSE(make_gait_clock(observations, 0.02).has_value());
}

TEST(GaitClock, MissingPeriodThrows)
{
  YAML::Node observations;
  observations["gait_phase"]["params"]["unrelated"] = 1.0;
  EXPECT_THROW(make_gait_clock(observations, 0.02), std::runtime_error);
}

TEST(GaitClock, NonPositivePeriodThrows)
{
  EXPECT_THROW(make_gait_clock(observations_with_period(0.0), 0.02), std::runtime_error);
  EXPECT_THROW(make_gait_clock(observations_with_period(-1.0), 0.02), std::runtime_error);
}

TEST(GaitClock, ResetSetsPhaseToZero)
{
  auto clock = make_gait_clock(observations_with_period(0.8), 0.02);
  ASSERT_TRUE(clock.has_value());
  PhaseState phase{.5f};
  clock->reset(phase);
  EXPECT_FLOAT_EQ(phase.phase, 0.0f);
}

TEST(GaitClock, FreeRunningAdvancesByStepOverPeriod)
{
  auto clock = make_gait_clock(observations_with_period(0.8), 0.02);
  ASSERT_TRUE(clock.has_value());
  SharedControlData shared;
  PhaseState phase;
  clock->advance(phase, shared);
  EXPECT_NEAR(phase.phase, 0.02f / 0.8f, 1e-6f);  // 0.025 per step
}

TEST(GaitClock, PhaseWrapsPastOne)
{
  // increment = step/period = 0.02/0.05 = 0.4 per step
  auto clock = make_gait_clock(observations_with_period(0.05), 0.02);
  ASSERT_TRUE(clock.has_value());
  SharedControlData shared;
  PhaseState phase;
  clock->advance(phase, shared);  // 0.4
  clock->advance(phase, shared);  // 0.8
  clock->advance(phase, shared);  // fmod(1.2, 1.0) = 0.2
  EXPECT_NEAR(phase.phase, 0.2f, 1e-5f);
  EXPECT_LT(phase.phase, 1.0f);
  EXPECT_GE(phase.phase, 0.0f);
}

TEST(GaitClock, CommandResetClockHoldsFirstStepAndResetsAtStand)
{
  auto config = observations_with_period(.8);
  config["gait_phase"]["params"]["offset"] = .5;
  config["gait_phase"]["params"]["command_threshold"] = .1;
  auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  shared.mode.velocity_commands = Eigen::Vector3f(.2, 0, 0);
  PhaseState phase;
  clock->advance(phase, shared);
  EXPECT_FLOAT_EQ(phase.phase, 0);
  clock->advance(phase, shared);
  EXPECT_NEAR(phase.phase, .025, 1e-7);
  shared.mode.velocity_commands.setZero();
  clock->advance(phase, shared);
  EXPECT_FLOAT_EQ(phase.phase, 0);
  shared.mode.velocity_commands.y() = .2;
  clock->advance(phase, shared);
  EXPECT_NEAR(phase.phase, .025, 1e-7);
  clock->reset(phase);
  clock->advance(phase, shared);
  EXPECT_FLOAT_EQ(phase.phase, 0);
  config["gait_phase"]["params"]["offset"] = 1;
  EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
}


TEST(GaitClock, AdaptivePeriodResetAndIndependentInstances)
{
  auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  auto clock = make_gait_clock(config, .02);
  auto other = make_gait_clock(config, .02);
  SharedControlData shared;  // No simulator height/recovery feedback exists or is needed.
  shared.mode.velocity_commands = Eigen::Vector3f(.2, 0, 0);
  PhaseState phase;
  PhaseState other_phase;
  clock->reset(phase);
  other->reset(other_phase);
  clock->advance(phase, shared);
  EXPECT_TRUE(clock->resolve_command_before_update());
  EXPECT_FLOAT_EQ(phase.phase, 0);
  EXPECT_FLOAT_EQ(phase.swing_ratio, .36f);
  clock->advance(phase, shared);
  EXPECT_NEAR(phase.phase, .02 / 1.16, 1e-7);
  for (float x : {2.0f, 2.5f}) {
    shared.mode.velocity_commands.x() = x;
    for (int i = 0; i < 250; ++i) {
      clock->advance(phase, shared);
    }
    float before = phase.phase;
    clock->advance(phase, shared);
    EXPECT_NEAR(std::fmod(phase.phase - before + 1.0f, 1.0f),
      .02 / (x < 2.4 ? .8 : .65), 1e-6);
    EXPECT_NEAR(phase.swing_ratio, .6, 1e-6);
  }
  shared.mode.velocity_commands.setZero();
  const auto before_stop = phase.phase;
  clock->advance(phase, shared);
  EXPECT_NE(phase.phase, before_stop);  // Free-running internally, masked output at stand.
  shared.mode.velocity_commands = Eigen::Vector3f(.06, 0, -.06);
  clock->advance(phase, shared);
  shared.mode.velocity_commands = Eigen::Vector3f(.1, 0, 0);
  clock->advance(phase, shared);
  shared.mode.velocity_commands.x() = .2;
  other->advance(other_phase, shared);
  EXPECT_FLOAT_EQ(other_phase.swing_ratio, .36f);
  clock->reset(phase);
  clock->advance(phase, shared);
  EXPECT_FLOAT_EQ(phase.phase, 0);
  EXPECT_FLOAT_EQ(phase.swing_ratio, .36f);
  clock->advance(phase, shared);
  EXPECT_NEAR(phase.phase, .02 / 1.16, 1e-7);
}

TEST(GaitClock, AdaptiveClockRejectsIncompatibleContracts)
{
  const auto original = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  auto config = YAML::Clone(original);
  config["gait_phase"]["params"]["schema"] = "unknown_clock";
  EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  for (const auto key : {"min_height", "min_upright", "recovery_delay_mask",
    "deployment_walk_period_scale"})
  {
    config = YAML::Clone(original);
    config["gait_phase"]["params"][key] = 1;
    EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  }
  config = YAML::Clone(original);
  config["gait_phase"]["params"]["periods"][0] = 0;
  EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  EXPECT_THROW(make_gait_clock(original, 0), std::runtime_error);
  EXPECT_THROW(make_gait_clock(original, std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
}

// Frozen command-reset update semantics. Exercise command boundaries,
// wraparound and re-entry while another implementation advances independently.
TEST(GaitClock, CommandResetMatchesPreviousRuleAcrossTransitionsAndReentry)
{
  auto config = observations_with_period(.8);
  config["gait_phase"]["params"]["command_threshold"] = .1;
  config["gait_phase"]["params"]["offset"] = .5;
  auto clock = make_gait_clock(config, .02);
  auto adaptive = make_gait_clock(
    YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml"), .02);
  SharedControlData shared;
  PhaseState actual;
  PhaseState adaptive_phase;
  adaptive->reset(adaptive_phase);
  float previous = 0;
  bool first = true;
  const float increment = .02f / .8f;
  for (int i = 0; i < 2400; ++i) {
    SCOPED_TRACE(i);
    if (i % 401 == 0) {
      clock->reset(actual);
      previous = 0;
      first = true;
    }
    switch ((i / 80) % 8) {
      case 0:
        shared.mode.velocity_commands = Eigen::Vector3f(.2f, 0, 0);
        break;
      case 1:
        shared.mode.velocity_commands.setZero();
        break;
      case 2:
        shared.mode.velocity_commands = Eigen::Vector3f(.1f, 0, 0);
        break;
      case 3:
        shared.mode.velocity_commands = Eigen::Vector3f(
          std::nextafter(.1f, 1.0f), 0, 0);
        break;
      case 4:
        shared.mode.velocity_commands = Eigen::Vector3f(-.2f, 0, 0);
        break;
      case 5:
        shared.mode.velocity_commands = Eigen::Vector3f(0, .2f, 0);
        break;
      case 6:
        shared.mode.velocity_commands = Eigen::Vector3f(0, 0, -.2f);
        break;
      default:
        shared.mode.velocity_commands = Eigen::Vector3f(.06f, 0, .06f);
        break;
    }
    if (first) {
      first = false;
    } else {
      previous = shared.mode.velocity_commands.norm() > .1f ?
        std::fmod(previous + increment, 1.0f) : 0.0f;
    }
    adaptive->advance(adaptive_phase, shared);
    clock->advance(actual, shared);
    EXPECT_FLOAT_EQ(actual.phase, previous);
    EXPECT_FALSE(clock->resolve_command_before_update());
  }
}

TEST(GaitClock, SameRuleCanAdvanceIndependentEpisodeStates)
{
  auto clock = make_gait_clock(
    YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml"), .02);
  SharedControlData shared;
  shared.mode.velocity_commands.x() = 2.8f;
  PhaseState a;
  PhaseState b;
  clock->reset(a);
  clock->reset(b);
  for (int i = 0; i < 200; ++i) {
    clock->advance(a, shared);
  }
  const auto a_before = a;
  clock->advance(b, shared);
  EXPECT_EQ(b.phase, 0);
  EXPECT_EQ(b.period, 1.16f);
  EXPECT_EQ(b.swing_ratio, .36f);
  clock->reset(b);
  EXPECT_EQ(a.phase, a_before.phase);
  EXPECT_EQ(a.period, a_before.period);
  EXPECT_EQ(a.swing_ratio, a_before.swing_ratio);
}

TEST(GaitClock, AdaptiveUsesYamlParametersWithoutSchema)
{
  auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  auto params = config["gait_phase"]["params"];
  params.remove("schema");
  params["periods"] = YAML::Load("[1.0, 0.6, 0.4]");
  params["forward_thresholds"] = YAML::Load("[0.5, 1.0]");
  params["swing_ratios"] = YAML::Load("[0.3, 0.4, 0.5]");
  params["transition_tau"] = .1;
  params["reset_phase"] = .125;
  const auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  PhaseState state;
  for (float speed : {.5f, .5001f, .9999f, 1.f}) {
    clock->reset(state);
    EXPECT_FLOAT_EQ(state.phase, .125f);
    EXPECT_FLOAT_EQ(state.period, 1.f);
    shared.mode.velocity_commands.x() = speed;
    clock->advance(state, shared);
    EXPECT_FLOAT_EQ(state.phase, .125f);
    clock->advance(state, shared);
    const int i = speed < .5f ? 0 : (speed < 1.f ? 1 : 2);
    const float periods[] = {1.f, .6f, .4f};
    const float swings[] = {.3f, .4f, .5f};
    const float alpha = 1.f - std::exp(-.02f / .1f);
    const float period = 1.f + alpha * (periods[i] - 1.f);
    EXPECT_FLOAT_EQ(state.period, period);
    EXPECT_FLOAT_EQ(state.swing_ratio, .3f + alpha * (swings[i] - .3f));
    EXPECT_FLOAT_EQ(state.phase, .125f + .02f / period);
  }
  for (const auto * schema : {"k1_settle_march_phase_v1", "k1_settle_march_phase_v1_command_only"}) {
    params["schema"] = schema;
    EXPECT_NO_THROW(make_gait_clock(config, .02));
  }
}

TEST(GaitClock, AdaptiveRejectsInvalidYamlParameters)
{
  const auto original = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  for (const auto * invalid : {
      "{periods: [1, 0.8]}", "{periods: [1, -1, 0.5]}",
      "{periods: [1, .nan, 0.5]}", "{forward_thresholds: [2, 1]}",
      "{forward_thresholds: [1, 1]}", "{swing_ratios: [0.3, 1, 0.5]}",
      "{transition_tau: 0}", "{transition_tau: .inf}",
      "{reset_phase: 1}", "{offsets: [0, -0.5]}",
      "{stand_command_norm_plus_yaw_threshold: -1}",
      "{observation_order: [sin_left]}", "{update: unknown}"})
  {
    SCOPED_TRACE(invalid);
    auto config = YAML::Clone(original);
    for (const auto & item : YAML::Load(invalid)) {
      config["gait_phase"]["params"][item.first.as<std::string>()] = item.second;
    }
    EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  }
}

TEST(GaitClock, AdaptiveBoundariesSelectNextRangeAtEquality)
{
  const auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  const auto clock = make_gait_clock(config, .02);
  struct Case
  {
    float speed;
    float target_period;
  };
  const Case cases[] = {
    {std::nextafter(1.6f, 0.f), 1.16f}, {1.6f, .8f},
    {std::nextafter(1.6f, 3.f), .8f},
    {std::nextafter(2.4f, 0.f), .8f}, {2.4f, .65f},
    {std::nextafter(2.4f, 3.f), .65f}};
  for (const auto & item : cases) {
    SCOPED_TRACE(item.speed);
    SharedControlData shared;
    shared.mode.velocity_commands.x() = item.speed;
    PhaseState state;
    clock->reset(state);
    clock->advance(state, shared);  // Preserve the entry observation.
    clock->advance(state, shared);
    const float expected = 1.16f + (1.f - std::exp(-.02f / .25f)) *
      (item.target_period - 1.16f);
    EXPECT_FLOAT_EQ(state.period, expected);
    for (int i = 0; i < 300; ++i) {
      clock->advance(state, shared);
    }
    EXPECT_NEAR(state.period, item.target_period, 1e-6);
  }
}

TEST(GaitClock, AdaptiveConfigurationErrorsIdentifyTheField)
{
  const auto original = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  struct Case
  {
    const char * key;
    const char * value;
  };
  const Case cases[] = {
    {"periods", "[1, broken, 0.5]"}, {"periods", "[1, 0.5]"},
    {"forward_thresholds", "[2, 1]"}, {"transition_tau", "0"},
    {"reset_phase", "1"}, {"offsets", "[0, .nan]"},
    {"stand_mode_priority", "false"}, {"observation_order", "[sin_left]"}};
  for (const auto & item : cases) {
    SCOPED_TRACE(item.key);
    auto config = YAML::Clone(original);
    config["gait_phase"]["params"][item.key] = YAML::Load(item.value);
    try {
      (void)make_gait_clock(config, .02);
      FAIL() << "Expected a configuration error";
    } catch (const std::runtime_error & error) {
      EXPECT_NE(std::string(error.what()).find(std::string("gait_phase.") + item.key),
        std::string::npos) << error.what();
    }
  }
  auto config = YAML::Clone(original);
  config["gait_phase"]["params"].remove("transition_tau");
  try {
    (void)make_gait_clock(config, .02);
    FAIL() << "Expected a missing-field error";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("gait_phase.transition_tau"), std::string::npos);
  }
}

TEST(GaitClock, AdaptiveSupportsAnyNumberOfForwardRanges)
{
  for (std::size_t count : {1u, 2u, 4u, 7u}) {
    SCOPED_TRACE(count);
    auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
    auto params = config["gait_phase"]["params"];
    params.remove("schema");
    std::vector<float> periods(count), swings(count), thresholds;
    for (std::size_t i = 0; i < count; ++i) {
      periods[i] = 1.f + .2f * i;
      swings[i] = .2f + .05f * i;
      if (i) {thresholds.push_back(static_cast<float>(i));}
    }
    params["periods"] = periods;
    params["swing_ratios"] = swings;
    params["forward_thresholds"] = thresholds;
    const auto clock = make_gait_clock(config, .02);
    SharedControlData shared;
    for (std::size_t i = 0; i < count; ++i) {
      PhaseState state;
      clock->reset(state);
      shared.mode.velocity_commands.x() = i ? static_cast<float>(i) : -2.f;
      clock->advance(state, shared);
      EXPECT_EQ(state.phase, 0);
      clock->advance(state, shared);
      const float alpha = 1.f - std::exp(-.02f / .25f);
      EXPECT_FLOAT_EQ(state.period, periods[0] + alpha * (periods[i] - periods[0]));
      EXPECT_FLOAT_EQ(state.swing_ratio, swings[0] + alpha * (swings[i] - swings[0]));
    }
  }
}

TEST(GaitClock, OptionalLowRegionAndExportedBoundaryComparators)
{
  auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) +
    "/../adaptive_phase/phase_contract.yaml");
  // Deliberately distinguish all periods: production low/fast both use0.8s.
  config["gait_phase"]["params"]["periods"] = YAML::Load("[0.9, 1.16, 0.8, 0.65]");
  const auto clock = make_gait_clock(config, .02);
  struct Case {Eigen::Vector3f command; float period;};
  const Case cases[] = {
    {{.4f, 0, .4f}, .9f}, {{-.4f, 0, -.4f}, .9f},
    {{.40001f, 0, 0}, 1.16f}, {{0, 0, .40001f}, 1.16f},
    {{0, .29999f, 0}, .9f}, {{0, .3f, 0}, 1.16f}, {{0, -.3f, 0}, 1.16f},
    {{.3f, .2f, 0}, .9f}, {{.4f, .2f, 0}, 1.16f},
    {{1.6f, 0, 0}, 1.16f}, {{1.60001f, 0, 0}, .8f}, {{2.4f, 0, 0}, .65f}};
  for (const auto & item : cases) {
    SharedControlData shared;
    shared.mode.velocity_commands = item.command;
    PhaseState state;
    clock->reset(state);
    clock->advance(state, shared);
    clock->advance(state, shared);
    EXPECT_FLOAT_EQ(state.period, .9f + (1.f - std::exp(-.02f / .25f)) * (item.period - .9f));
  }
  // Low priority is independent of the number of forward ranges, including none.
  auto params = config["gait_phase"]["params"];
  params["periods"] = YAML::Load("[0.9, 1.16]");
  params["swing_ratios"] = YAML::Load("[0.4, 0.36]");
  params["forward_thresholds"] = YAML::Load("[]");
  params["forward_threshold_inclusive"] = YAML::Load("[]");
  params.remove("range_order");
  EXPECT_NO_THROW(make_gait_clock(config, .02));
  params["periods"] = YAML::Load("[0.9, 1.16, 0.8, 0.65, 0.5]");
  params["swing_ratios"] = YAML::Load("[0.4, 0.36, 0.6, 0.6, 0.6]");
  params["forward_thresholds"] = YAML::Load("[1.6, 2.4, 3.0]");
  params["forward_threshold_inclusive"] = YAML::Load("[false, true, false]");
  EXPECT_NO_THROW(make_gait_clock(config, .02));
}

TEST(GaitClock, AdaptiveRejectsInvalidRangeShapesAndLowBounds)
{
  const auto original = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) +
    "/../adaptive_phase/phase_contract.yaml");
  for (const auto * invalid : {
      "{periods: []}", "{periods: [0.8, 1.16, 0.8]}",
      "{swing_ratios: [0.4]}", "{forward_threshold_inclusive: [true]}",
      "{forward_thresholds: [.nan, 2.4]}", "{range_order: [low]}",
      "{low_planar_max_inclusive: 0}", "{low_yaw_max_inclusive: -.1}",
      "{low_lateral_max_exclusive: .inf}", "{low_speed_priority: false}"})
  {
    SCOPED_TRACE(invalid);
    auto config = YAML::Clone(original);
    for (const auto & item : YAML::Load(invalid)) {
      config["gait_phase"]["params"][item.first.as<std::string>()] = item.second;
    }
    EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  }
  for (const auto key : {"low_planar_max_inclusive", "low_yaw_max_inclusive",
    "low_lateral_max_exclusive"})
  {
    auto config = YAML::Clone(original);
    config["gait_phase"]["params"].remove(key);
    EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  }
}
