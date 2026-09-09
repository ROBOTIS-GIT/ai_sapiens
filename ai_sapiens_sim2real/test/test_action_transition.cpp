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
// Author: Woojin Wie, Kiwoong Park

#include <gtest/gtest.h>

#include <limits>

#include "ai_sapiens_sim2real/policy/action_transition.hpp"

using ai_sapiens_sim2real::ActionTransition;

TEST(ActionTransition, StartsAtPublishedTargetAndTracksChangingPolicyTarget)
{
  ActionTransition transition;
  transition.resize(2);
  transition.begin({0.5f, -0.5f}, {0, 1}, 0.2);
  EXPECT_FLOAT_EQ(transition.position(0, 2.0f), 0.5f);
  EXPECT_FLOAT_EQ(transition.position(1, 2.0f), -0.5f);
  transition.advance(0.05);
  EXPECT_FLOAT_EQ(transition.position(0, 1.5f), 0.65625f);
  transition.advance(0.05);
  EXPECT_FLOAT_EQ(transition.position(0, 2.5f), 1.5f);
  transition.advance(0.1);
  EXPECT_FLOAT_EQ(transition.position(0, 3.0f), 3.0f);
  EXPECT_FLOAT_EQ(transition.position(1, -2.0f), -2.0f);
}

TEST(ActionTransition, UsesControlPeriodBetweenInferenceTicks)
{
  ActionTransition transition;
  transition.resize(1);
  transition.begin({0.0f}, {0}, 0.2);
  transition.advance(0.004);
  const float first = transition.position(0, 1.0f);
  transition.advance(0.006);
  const float second = transition.position(0, 1.0f);
  EXPECT_GT(first, 0.0f);
  EXPECT_GT(second, first);
  EXPECT_NEAR(second, 0.00725f, 1e-7f);
  transition.advance(10.0);
  EXPECT_FLOAT_EQ(transition.position(0, 1.0f), 1.0f);
}

TEST(ActionTransition, MapsSubsetAndRestartsFromLatestPublishedCommand)
{
  ActionTransition transition;
  transition.resize(2);
  transition.begin({10.0f, 20.0f, 30.0f}, {2, 0}, 0.2);
  EXPECT_FLOAT_EQ(transition.position(0, 0.0f), 30.0f);
  EXPECT_FLOAT_EQ(transition.position(1, 0.0f), 10.0f);
  transition.advance(0.1);
  transition.begin({5.0f, 20.0f, 15.0f}, {2, 0}, 0.2);
  EXPECT_FLOAT_EQ(transition.position(0, -5.0f), 15.0f);
  EXPECT_FLOAT_EQ(transition.position(1, -5.0f), 5.0f);
}

TEST(ActionTransition, DisabledAndZeroDurationReturnExactNewTarget)
{
  ActionTransition transition;
  transition.resize(1);
  EXPECT_FLOAT_EQ(transition.position(0, 0.7f), 0.7f);
  transition.begin({0.5f}, {0}, 0.0);
  EXPECT_FLOAT_EQ(transition.position(0, 0.7f), 0.7f);
  transition.begin({0.5f}, {0}, 0.2);
  transition.reset();
  EXPECT_FLOAT_EQ(transition.position(0, 0.7f), 0.7f);
}

TEST(ActionTransition, RejectsInvalidDurationAndIgnoresInvalidElapsedPeriods)
{
  ActionTransition transition;
  transition.resize(1);
  EXPECT_THROW(transition.begin({0.0f}, {0}, -1.0), std::runtime_error);
  EXPECT_THROW(transition.begin({0.0f}, {0}, std::numeric_limits<double>::infinity()),
    std::runtime_error);
  transition.begin({0.5f}, {0}, 0.2);
  transition.advance(-1.0);
  transition.advance(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FLOAT_EQ(transition.position(0, 2.0f), 0.5f);
}

TEST(ActionTransition, GainsSharePositionProgressInBothDirections)
{
  ActionTransition transition;
  transition.resize(2);
  transition.begin({0.0f, 0.0f}, {0, 1}, 0.2);
  transition.capture_gains({40.0f, 100.0f}, {4.0f, 10.0f}, {0, 1});
  EXPECT_FLOAT_EQ(transition.stiffness(0, 100.0f), 40.0f);
  EXPECT_FLOAT_EQ(transition.damping(1, 4.0f), 10.0f);
  transition.advance(0.05);
  const float alpha = transition.position(0, 1.0f);
  EXPECT_FLOAT_EQ(transition.stiffness(0, 100.0f), 40.0f + alpha * 60.0f);
  EXPECT_FLOAT_EQ(transition.stiffness(1, 40.0f), 100.0f - alpha * 60.0f);
  EXPECT_FLOAT_EQ(transition.damping(0, 10.0f), 4.0f + alpha * 6.0f);
  EXPECT_FLOAT_EQ(transition.damping(1, 4.0f), 10.0f - alpha * 6.0f);
  transition.advance(0.15);
  EXPECT_FLOAT_EQ(transition.stiffness(0, 100.0f), 100.0f);
  EXPECT_FLOAT_EQ(transition.damping(1, 4.0f), 4.0f);
}

TEST(ActionTransition, GainRestartUsesPublishedIntermediateValuesAndJointMapping)
{
  ActionTransition transition;
  transition.resize(1);
  transition.begin({0.0f, 1.0f}, {1}, 0.2);
  transition.capture_gains({999.0f, 40.0f}, {99.0f, 4.0f}, {1});
  transition.advance(0.1);
  const float published_kp = transition.stiffness(0, 100.0f);
  const float published_kd = transition.damping(0, 10.0f);
  EXPECT_FLOAT_EQ(published_kp, 70.0f);
  EXPECT_FLOAT_EQ(published_kd, 7.0f);
  transition.begin({0.0f, 1.5f}, {1}, 0.2);
  transition.capture_gains({999.0f, published_kp}, {99.0f, published_kd}, {1});
  EXPECT_FLOAT_EQ(transition.stiffness(0, 20.0f), published_kp);
  EXPECT_FLOAT_EQ(transition.damping(0, 2.0f), published_kd);
  transition.advance(0.1);
  EXPECT_FLOAT_EQ(transition.stiffness(0, 20.0f), 45.0f);
  EXPECT_FLOAT_EQ(transition.damping(0, 2.0f), 4.5f);
}

TEST(ActionTransition, DisabledGainsSwitchImmediatelyAndEqualGainsStayConstant)
{
  ActionTransition transition;
  transition.resize(1);
  EXPECT_FLOAT_EQ(transition.stiffness(0, 80.0f), 80.0f);
  EXPECT_FLOAT_EQ(transition.damping(0, 8.0f), 8.0f);
  transition.begin({0.0f}, {0}, 0.2);
  transition.capture_gains({80.0f}, {8.0f}, {0});
  transition.advance(0.05);
  EXPECT_FLOAT_EQ(transition.stiffness(0, 80.0f), 80.0f);
  EXPECT_FLOAT_EQ(transition.damping(0, 8.0f), 8.0f);
  transition.reset();
  EXPECT_FLOAT_EQ(transition.stiffness(0, 20.0f), 20.0f);
  EXPECT_FLOAT_EQ(transition.damping(0, 2.0f), 2.0f);
  transition.begin({0.0f}, {0}, 0.0);
  EXPECT_FLOAT_EQ(transition.stiffness(0, 60.0f), 60.0f);
  EXPECT_FLOAT_EQ(transition.damping(0, 6.0f), 6.0f);
}
