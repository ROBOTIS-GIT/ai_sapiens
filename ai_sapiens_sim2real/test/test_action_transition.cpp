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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <limits>

#include "ai_sapiens_sim2real/command_transition.hpp"

using ai_sapiens_sim2real::CommandTransition;
using ai_sapiens_sim2real::JointCommand;
using ai_sapiens_sim2real::PolicyUpdateResult;

namespace
{

void expect_same(const JointCommand & a, const JointCommand & b)
{
  ASSERT_EQ(a.position.size(), b.position.size());
  for (size_t i = 0; i < a.position.size(); ++i) {
    // Bitwise comparison, rather than a float tolerance, catches ordering drift.
    EXPECT_EQ(std::memcmp(&a.position[i], &b.position[i], sizeof(float)), 0) << i;
    EXPECT_EQ(std::memcmp(&a.stiffness[i], &b.stiffness[i], sizeof(float)), 0) << i;
    EXPECT_EQ(std::memcmp(&a.damping[i], &b.damping[i], sizeof(float)), 0) << i;
  }
}

JointCommand initial()
{
  return {{0.8f, -0.5f, 0.3f}, {40.0f, 50.0f, 60.0f}, {2.0f, 3.0f, 4.0f}};
}

}  // namespace

TEST(CommandTransition, FirstInferenceStartsAtZeroAndAllFieldsShareSmoothstep)
{
  auto output = initial();
  const auto source = output;
  JointCommand target{{1.8f, -1.5f, 0.9f}, {80.0f, 10.0f, 60.0f}, {6.0f, 1.0f, 4.0f}};
  CommandTransition transition(3);
  transition.begin(source, target, {0, 1, 2}, 0.2, output);
  transition.update(0.1, PolicyUpdateResult::TargetUnavailable, target, output);
  transition.update(0.1, PolicyUpdateResult::Skipped, target, output);
  expect_same(source, output);
  transition.update(0.1, PolicyUpdateResult::TargetReady, target, output);
  expect_same(source, output);
  transition.update(0.05, PolicyUpdateResult::Skipped, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.95625f);
  EXPECT_FLOAT_EQ(output.stiffness[0], 46.25f);
  EXPECT_FLOAT_EQ(output.damping[0], 2.625f);
  EXPECT_FLOAT_EQ(output.stiffness[2], 60.0f);
  transition.update(0.15, PolicyUpdateResult::Skipped, target, output);
  expect_same(target, output);
}

TEST(CommandTransition, DisabledEntryInstallsGainsButHoldsPositionUntilValidTarget)
{
  auto output = initial();
  JointCommand target{{-1.0f}, {99.0f}, {9.0f}};
  CommandTransition transition(3);
  transition.begin(output, target, {1}, 0.0, output);
  EXPECT_FLOAT_EQ(output.position[1], -0.5f);
  EXPECT_FLOAT_EQ(output.stiffness[1], 99.0f);
  transition.update(0.02, PolicyUpdateResult::TargetUnavailable, target, output);
  EXPECT_FLOAT_EQ(output.position[1], -0.5f);
  transition.update(0.02, PolicyUpdateResult::TargetReady, target, output);
  EXPECT_FLOAT_EQ(output.position[1], -1.0f);
  EXPECT_FLOAT_EQ(output.position[0], 0.8f);
  EXPECT_FLOAT_EQ(output.damping[2], 4.0f);
}

TEST(CommandTransition, FailureHoldsOutputButRetainsPreviousTickTimeAdvance)
{
  auto output = initial();
  JointCommand target{{1.8f}, {80.0f}, {6.0f}};
  CommandTransition transition(3);
  transition.begin(output, target, {0}, 0.2, output);
  transition.update(0.02, PolicyUpdateResult::TargetReady, target, output);
  transition.update(0.05, PolicyUpdateResult::TargetUnavailable, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.8f);
  transition.update(0.8, PolicyUpdateResult::Skipped, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.8f);
  transition.update(0.8, PolicyUpdateResult::TargetReady, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.95625f);
}

TEST(CommandTransition, RestartUsesClippedPublishedCommandAndDoesNotTouchOtherJoints)
{
  auto output = initial();
  JointCommand target{{2.0f}, {100.0f}, {10.0f}};
  CommandTransition transition(3);
  transition.begin(output, target, {0}, 0.2, output);
  transition.update(0.02, PolicyUpdateResult::TargetReady, target, output);
  transition.update(0.1, PolicyUpdateResult::Skipped, target, output);
  auto published = output;
  published.position[0] = std::min(published.position[0], 1.0f);
  transition.begin(published, target, {0}, 0.3, output);
  transition.update(0.02, PolicyUpdateResult::TargetReady, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 1.0f);
  EXPECT_FLOAT_EQ(output.stiffness[0], published.stiffness[0]);
  EXPECT_FLOAT_EQ(output.damping[0], published.damping[0]);
  EXPECT_FLOAT_EQ(output.position[1], -0.5f);
}

TEST(CommandTransition, HoldsPublishedPositionsUntilFirstTargetIsReady)
{
  for (const double duration : {0.0, 0.1}) {
    SCOPED_TRACE(duration);
    auto output = initial();
    output.position = {1.2f, -0.5f, -1.2f};
    auto published = output;
    published.position = {0.5f, -0.2f, -0.4f};
    const JointCommand target{{0.6f, -0.1f}, {90.0f, 80.0f}, {9.0f, 8.0f}};
    auto expected = output;
    expected.position[0] = published.position[0];
    expected.position[2] = published.position[2];
    if (duration == 0.0) {
      expected.stiffness[0] = 80.0f;
      expected.stiffness[2] = 90.0f;
      expected.damping[0] = 8.0f;
      expected.damping[2] = 9.0f;
    }

    CommandTransition transition(3);
    transition.begin(published, target, {2, 0}, duration, output);
    expect_same(expected, output);
    for (const auto result : {PolicyUpdateResult::Skipped,
        PolicyUpdateResult::TargetUnavailable, PolicyUpdateResult::Skipped})
    {
      transition.update(0.2, result, target, output);
      expect_same(expected, output);
      // Wider limits in the new policy must not expose the old unclipped target.
      EXPECT_FLOAT_EQ(std::clamp(output.position[0], -2.0f, 2.0f), 0.5f);
      EXPECT_FLOAT_EQ(std::clamp(output.position[2], -2.0f, 2.0f), -0.4f);
    }

    transition.update(0.2, PolicyUpdateResult::TargetReady, target, output);
    if (duration > 0.0) {
      expect_same(expected, output);
      transition.update(duration, PolicyUpdateResult::Skipped, target, output);
    }
    expected.position[0] = -0.1f;
    expected.position[2] = 0.6f;
    expected.stiffness[0] = 80.0f;
    expected.stiffness[2] = 90.0f;
    expected.damping[0] = 8.0f;
    expected.damping[2] = 9.0f;
    expect_same(expected, output);
  }
}

TEST(CommandTransition, ResetDoesNotOverwriteDampingOrPostureCommand)
{
  auto output = initial();
  JointCommand target{{1.8f}, {80.0f}, {6.0f}};
  CommandTransition transition(3);
  transition.begin(output, target, {0}, 0.2, output);
  transition.update(0.02, PolicyUpdateResult::TargetReady, target, output);
  transition.reset();
  output = {{0.1f, 0.2f, 0.3f}, {0.0f, 0.0f, 0.0f}, {3.0f, 3.0f, 3.0f}};
  const auto damping = output;
  transition.update(0.1, PolicyUpdateResult::Skipped, target, output);
  expect_same(damping, output);
}

TEST(CommandTransition, InvalidDurationAndJointCountAreRejected)
{
  auto output = initial();
  CommandTransition transition(3);
  EXPECT_THROW(transition.begin(output, output, {0, 1, 2}, -0.1, output), std::runtime_error);
  EXPECT_THROW(transition.begin(output, output, {0, 1, 2},
    std::numeric_limits<double>::quiet_NaN(), output), std::runtime_error);
  EXPECT_THROW(transition.begin(output, output, {0, 1, 2},
    std::numeric_limits<double>::infinity(), output), std::runtime_error);
  EXPECT_THROW(transition.begin(output, output, {0}, 0.2, output), std::runtime_error);
}
