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

#include <stdexcept>

#include "ai_sapiens_sim2real/policy/gait_clock.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

using ai_sapiens_sim2real::make_gait_clock;
using ai_sapiens_sim2real::SharedControlData;

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
  float phase = 0.5f;
  clock->reset(phase);
  EXPECT_FLOAT_EQ(phase, 0.0f);
}

TEST(GaitClock, FreeRunningAdvancesByStepOverPeriod)
{
  auto clock = make_gait_clock(observations_with_period(0.8), 0.02);
  ASSERT_TRUE(clock.has_value());
  SharedControlData shared;
  float phase = 0.0f;
  clock->advance(phase, shared);
  EXPECT_NEAR(phase, 0.02f / 0.8f, 1e-6f);  // 0.025 per step
}

TEST(GaitClock, PhaseWrapsPastOne)
{
  // increment = step/period = 0.02/0.05 = 0.4 per step
  auto clock = make_gait_clock(observations_with_period(0.05), 0.02);
  ASSERT_TRUE(clock.has_value());
  SharedControlData shared;
  float phase = 0.0f;
  clock->advance(phase, shared);  // 0.4
  clock->advance(phase, shared);  // 0.8
  clock->advance(phase, shared);  // fmod(1.2, 1.0) = 0.2
  EXPECT_NEAR(phase, 0.2f, 1e-5f);
  EXPECT_LT(phase, 1.0f);
  EXPECT_GE(phase, 0.0f);
}
