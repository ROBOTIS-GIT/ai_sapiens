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
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/policy/gait_clock.hpp"
#include "ai_sapiens_sim2real/observation/observations.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

using ai_sapiens_sim2real::make_gait_clock;
using ai_sapiens_sim2real::ObservationContext;
using ai_sapiens_sim2real::ObservationManager;
using ai_sapiens_sim2real::ObservationRegistry;
using ai_sapiens_sim2real::PolicyJointContext;
using ai_sapiens_sim2real::SharedControlData;

TEST(GaitObservation, PreservesFixedPhaseLayoutsAndStandBehavior)
{
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  auto params = YAML::Load("{period: 0.8}");
  EXPECT_EQ(observe(context, params), (std::vector<float>{0, 0}));
  shared.mode.velocity_commands.x() = .2;
  shared.policy.phase_state.phase = .25;
  auto obs = observe(context, params);
  ASSERT_EQ(obs.size(), 2u);
  EXPECT_NEAR(obs[0], 1, 1e-6);
  EXPECT_NEAR(obs[1], 0, 1e-6);
  params["offset"] = .5;
  params["command_threshold"] = .1;
  obs = observe(context, params);
  ASSERT_EQ(obs.size(), 4u);
  EXPECT_NEAR(obs[0], 1, 1e-6);
  EXPECT_NEAR(obs[1], -1, 1e-6);
  EXPECT_NEAR(obs[2], 0, 1e-6);
  EXPECT_NEAR(obs[3], 0, 1e-6);
  shared.mode.velocity_commands.setZero();
  EXPECT_EQ(observe(context, params), (std::vector<float>{0, 0, 1, 1}));
}

TEST(GaitObservation, BilateralPhaseWithSwingAppliesCommandOnlyStandGate)
{
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  shared.mode.velocity_commands = Eigen::Vector3f(.06, 0, .06);
  shared.policy.phase_state.phase = .25f;
  shared.policy.phase_state.swing_ratio = .36f;
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  auto params = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) +
    "/phase_contract.yaml")["gait_phase"]["params"];
  const auto obs = observe(context, params);
  ASSERT_EQ(obs.size(), 6u);
  EXPECT_NEAR(obs[0], 1, 1e-6);
  EXPECT_NEAR(obs[1], 0, 1e-6);
  EXPECT_NEAR(obs[2], -1, 1e-6);
  EXPECT_NEAR(obs[3], 0, 1e-6);
  EXPECT_FLOAT_EQ(obs[4], .36f);
  EXPECT_FLOAT_EQ(obs[5], .36f);
  for (float x : {0.0f, .1f}) {
    shared.mode.velocity_commands = Eigen::Vector3f(x, 0, 0);
    EXPECT_EQ(observe(context, params), std::vector<float>(6, 0));
  }
}

// Frozen equations from the reference adaptive clock/observation rule.
// Verify entry-history seeding and every output across gait changes and resets.
// The intentionally changed exact lower boundary is covered by the clock tests.
TEST(GaitObservation, AdaptiveMatchesPreviousRuleAwayFromChangedBoundary)
{
  auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  float old_phase = 0;
  float old_period = 1.16f;
  float old_swing = .36f;
  bool first = true;
  const auto reference_observation = [&]() {
      std::vector<float> output(6, 0);
      const auto & c = shared.mode.velocity_commands;
      if (!(std::hypot(c.x(), c.y()) + std::abs(c.z()) > .1f)) {
        return output;
      }
      constexpr float two_pi = 6.283185307179586f;
      const float right = std::fmod(old_phase + .5f, 1.0f);
      output = {std::sin(two_pi * old_phase), std::cos(two_pi * old_phase),
        std::sin(two_pi * right), std::cos(two_pi * right), old_swing, old_swing};
      return output;
    };
  const Eigen::Vector3f commands[] = {
    {.2f, 0, 0}, {1.5999f, 0, 0}, {1.6001f, 0, 0}, {2.3999f, 0, 0},
    {2.4f, 0, 0}, {2.8f, 0, 0}, {0, 0, 0}, {.1f, 0, 0},
    {.06f, 0, -.06f}, {-.2f, 0, 0}, {0, .2f, 0}, {0, 0, -.4f}};
  for (int i = 0; i < 2400; ++i) {
    SCOPED_TRACE(i);
    shared.mode.velocity_commands = commands[(i / 50) % 12];
    if (i % 401 == 0) {
      clock->reset(shared.policy.phase_state);
      old_phase = 0;
      old_period = 1.16f;
      old_swing = .36f;
      first = true;
      EXPECT_EQ(observe(context, config["gait_phase"]["params"]), reference_observation());
    }
    if (first) {
      first = false;
    } else {
      const float x = shared.mode.velocity_commands.x();
      const int category = (x > 1.6f) + (x >= 2.4f);
      const float periods[] = {1.16f, .8f, .65f};
      const float swings[] = {.36f, .6f, .6f};
      const float alpha = 1.0f - std::exp(-.02f / .25f);
      old_period += alpha * (periods[category] - old_period);
      old_swing += alpha * (swings[category] - old_swing);
      old_phase = std::fmod(old_phase + .02f / old_period, 1.0f);
    }
    clock->advance(shared.policy.phase_state, shared);
    EXPECT_EQ(observe(context, config["gait_phase"]["params"]), reference_observation());
  }
}

TEST(GaitObservation, AdaptiveUsesYamlOffsetsAndStandThreshold)
{
  auto params = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) +
    "/phase_contract.yaml")["gait_phase"]["params"];
  params.remove("schema");
  params["offsets"] = YAML::Load("[0.25, 0.75]");
  params["stand_command_norm_plus_yaw_threshold"] = .2;
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  shared.mode.velocity_commands.x() = .15;
  EXPECT_EQ(observe(context, params), std::vector<float>(6, 0));
  shared.mode.velocity_commands.x() = .3;
  shared.policy.phase_state.swing_ratio = .4;
  const auto result = observe(context, params);
  ASSERT_EQ(result.size(), 6u);
  EXPECT_NEAR(result[0], 1, 1e-6);
  EXPECT_NEAR(result[1], 0, 1e-6);
  EXPECT_NEAR(result[2], -1, 1e-6);
  EXPECT_NEAR(result[3], 0, 1e-6);
  EXPECT_FLOAT_EQ(result[4], .4f);
}

TEST(GaitObservation, AdaptiveMatchesTrainingFixtureAcrossLowAndForwardRanges)
{
  const std::string directory = std::string(PHASE_TEST_DATA_DIR) + "/../adaptive_phase/";
  auto config = YAML::LoadFile(directory + "phase_contract.yaml");
  auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  const auto observe = ObservationRegistry::get_registry().at("gait_phase");
  clock->reset(shared.policy.phase_state);
  std::ifstream fixture(directory + "clock_fixture.txt");
  ASSERT_TRUE(fixture.is_open());
  float x, y, yaw;
  int rows = 0;
  while (fixture >> x >> y >> yaw) {
    std::vector<float> expected(6);
    for (float & value : expected) {ASSERT_TRUE(static_cast<bool>(fixture >> value));}
    shared.mode.velocity_commands = Eigen::Vector3f(x, y, yaw);
    clock->advance(shared.policy.phase_state, shared);
    const auto actual = observe(context, config["gait_phase"]["params"]);
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {
      EXPECT_NEAR(actual[i], expected[i], 2e-4f) << "row=" << rows << " channel=" << i;
    }
    ++rows;
  }
  EXPECT_EQ(rows, 4480);
}

namespace
{
YAML::Node low_only_phase_config()
{
  auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  auto params = config["gait_phase"]["params"];
  params.remove("schema");
  params["periods"] = YAML::Load("[0.8]");
  params["swing_ratios"] = YAML::Load("[0.4]");
  params["forward_thresholds"] = YAML::Load("[]");
  params.remove("forward_threshold_inclusive");
  params["stand_command_norm_plus_yaw_threshold"] = .05;
  params["activation_region"] = YAML::Load(R"(
kind: low_only_smoothstep
planar_max: 0.4
yaw_max: 0.4
lateral_max: 0.3
planar_fade_width: 0.05
yaw_fade_width: 0.05
lateral_fade_width: 0.05
moving_fade_width: 0.01
activation_rise_tau: 0.25
)");
  return config;
}
}  // namespace

TEST(GaitObservation, LowOnlyEnvelopeMatchesTrainingRiseExitAndReset)
{
  auto config = low_only_phase_config();
  auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  clock->reset(shared.policy.phase_state);
  // Known smoothstep weights: full, halfway at each upper fade, half at onset,
  // outside each bound, then re-entry. Include both directions and pure turns.
  const Eigen::Vector3f commands[] = {
    {.2f, 0, 0}, {-.2f, 0, 0}, {0, 0, .2f}, {0, 0, -.2f},
    {.375f, 0, 0}, {0, 0, -.375f}, {0, -.275f, 0}, {.055f, 0, 0},
    {.4f, 0, 0}, {0, 0, .4f}, {0, .3f, 0}, {1.0f, 0, 0},
    {0, 0, 0}, {.05f, 0, 0}, {0, .2f, 0}};
  const float targets[] = {1, 1, 1, 1, .5f, .5f, .5f, .5f, 0, 0, 0, 0, 0, 0, 1};
  float weight = 0, phase = 0;
  bool first = true;
  for (int step = 0; step < 1800; ++step) {
    SCOPED_TRACE(step);
    const int segment = (step / 30) % 15;
    shared.mode.velocity_commands = commands[segment];
    if (step % 257 == 0) {
      clock->reset(shared.policy.phase_state);
      weight = phase = 0;
      first = true;
      EXPECT_EQ(observe(context, config["gait_phase"]["params"]), std::vector<float>(6, 0));
    }
    if (first) {
      first = false;
    } else {
      // Closed-form one-step rise; decreasing targets take effect immediately.
      const float target = targets[segment];
      weight = target < weight ? target : target + (weight - target) * std::exp(-.02f / .25f);
      phase = std::fmod(phase + .02f / .8f, 1.0f);
    }
    clock->advance(shared.policy.phase_state, shared);
    EXPECT_NEAR(shared.policy.phase_state.phase, phase, 1e-6f);
    const auto actual = observe(context, config["gait_phase"]["params"]);
    constexpr float two_pi = 6.283185307179586f;
    const float right = std::fmod(phase + .5f, 1.0f);
    const std::vector<float> expected = {
      weight * std::sin(two_pi * phase), weight * std::cos(two_pi * phase),
      weight * std::sin(two_pi * right), weight * std::cos(two_pi * right),
      weight * .4f, weight * .4f};
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {EXPECT_NEAR(actual[i], expected[i], 2e-6f);}
  }
}

TEST(GaitObservation, LowOnlyRejectsInvalidActivationContracts)
{
  for (const auto * field : {"planar_max", "yaw_max", "lateral_max", "planar_fade_width",
      "yaw_fade_width", "lateral_fade_width", "moving_fade_width", "activation_rise_tau"})
  {
    for (const auto * value : {"0", "-1", ".nan", ".inf"}) {
      SCOPED_TRACE(std::string(field) + "=" + value);
      auto config = low_only_phase_config();
      config["gait_phase"]["params"]["activation_region"][field] = YAML::Load(value);
      EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
    }
    auto config = low_only_phase_config();
    config["gait_phase"]["params"]["activation_region"].remove(field);
    EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  }
  for (const auto * yaml : {"{kind: unknown}", "[]", "null",
      "{kind: low_only_smoothstep}"})
  {
    auto config = low_only_phase_config();
    config["gait_phase"]["params"]["activation_region"] = YAML::Load(yaml);
    EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  }
  for (const auto * field : {"planar_fade_width", "yaw_fade_width", "lateral_fade_width"}) {
    auto config = low_only_phase_config();
    config["gait_phase"]["params"]["activation_region"][field] = 1;
    EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
  }
  auto config = YAML::Load("gait_phase: {params: {period: 0.8, activation_region: {}}}");
  EXPECT_THROW(make_gait_clock(config, .02), std::runtime_error);
}

TEST(GaitObservation, AdaptiveIntegratesWithOfficialTermMajorHistory)
{
  const auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) +
    "/../adaptive_phase/phase_contract.yaml");
  auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  PolicyJointContext joints;
  shared.mode.velocity_commands.x() = .2f;
  clock->reset(shared.policy.phase_state);
  ObservationContext context{shared, joints};
  const auto observe = ObservationRegistry::get_registry().at("gait_phase");
  const auto initial = observe(context, config["gait_phase"]["params"]);
  ObservationManager manager(config, &shared, joints, nullptr);
  manager.reset();
  ASSERT_EQ(manager.get_observation_size(), 30u);
  std::vector<float> expected;
  for (int frame = 0; frame < 5; ++frame) {
    expected.insert(expected.end(), initial.begin(), initial.end());
  }
  EXPECT_EQ(manager.compute(), expected);
  clock->advance(shared.policy.phase_state, shared);  // First update holds reset state.
  clock->advance(shared.policy.phase_state, shared);
  const auto next = observe(context, config["gait_phase"]["params"]);
  expected.erase(expected.begin(), expected.begin() + 6);
  expected.insert(expected.end(), next.begin(), next.end());
  EXPECT_EQ(manager.compute(), expected);
}
