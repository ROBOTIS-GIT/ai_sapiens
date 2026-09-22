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

#include <algorithm>
#include <cstring>
#include <limits>
#include <random>

#include "ai_sapiens_sim2real/command_transition.hpp"
#include "reference/action_transition_before_refactor.hpp"

using ai_sapiens_sim2real::CommandTransition;
using ai_sapiens_sim2real::JointCommand;
using ai_sapiens_sim2real::TargetUpdate;

namespace
{

// The old PolicyRuntime entry/update output path, with inference represented by
// its outcome. Preserve the advance-before-inference-failure ordering verbatim.
class PreviousOutputPath
{
public:
  void enter(
    const JointCommand & source, const JointCommand & target,
    const std::vector<size_t> & joints, double duration, JointCommand & output)
  {
    joints_ = joints;
    ready_ = false;
    transition_.resize(joints.size());
    transition_.reset();
    if (duration > 0.0) {
      transition_.begin(source.position, joints, duration);
      transition_.capture_gains(source.stiffness, source.damping, joints);
    }
    for (size_t j = 0; j < joints.size(); ++j) {
      output.stiffness[joints[j]] = transition_.stiffness(j, target.stiffness[j]);
      output.damping[joints[j]] = transition_.damping(j, target.damping[j]);
    }
  }

  void update(
    double period, TargetUpdate result, const JointCommand & target,
    JointCommand & output)
  {
    if (ready_) {
      transition_.advance(period);
    }
    if (result == TargetUpdate::HoldOutput) {
      ready_ = false;
    } else if (result == TargetUpdate::TargetReady) {
      ready_ = true;
    }
    if (ready_) {
      for (size_t j = 0; j < joints_.size(); ++j) {
        output.position[joints_[j]] = transition_.position(j, target.position[j]);
        output.stiffness[joints_[j]] = transition_.stiffness(j, target.stiffness[j]);
        output.damping[joints_[j]] = transition_.damping(j, target.damping[j]);
      }
    }
  }

private:
  before_refactor::ActionTransition transition_;
  std::vector<size_t> joints_;
  bool ready_{false};
};

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
  transition.update(0.1, TargetUpdate::HoldOutput, target, output);
  transition.update(0.1, TargetUpdate::NoUpdate, target, output);
  expect_same(source, output);
  transition.update(0.1, TargetUpdate::TargetReady, target, output);
  expect_same(source, output);
  transition.update(0.05, TargetUpdate::NoUpdate, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.95625f);
  EXPECT_FLOAT_EQ(output.stiffness[0], 46.25f);
  EXPECT_FLOAT_EQ(output.damping[0], 2.625f);
  EXPECT_FLOAT_EQ(output.stiffness[2], 60.0f);
  transition.update(0.15, TargetUpdate::NoUpdate, target, output);
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
  transition.update(0.02, TargetUpdate::HoldOutput, target, output);
  EXPECT_FLOAT_EQ(output.position[1], -0.5f);
  transition.update(0.02, TargetUpdate::TargetReady, target, output);
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
  transition.update(0.02, TargetUpdate::TargetReady, target, output);
  transition.update(0.05, TargetUpdate::HoldOutput, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.8f);
  transition.update(0.8, TargetUpdate::NoUpdate, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.8f);
  transition.update(0.8, TargetUpdate::TargetReady, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 0.95625f);
}

TEST(CommandTransition, RestartUsesClippedPublishedCommandAndDoesNotTouchOtherJoints)
{
  auto output = initial();
  JointCommand target{{2.0f}, {100.0f}, {10.0f}};
  CommandTransition transition(3);
  transition.begin(output, target, {0}, 0.2, output);
  transition.update(0.02, TargetUpdate::TargetReady, target, output);
  transition.update(0.1, TargetUpdate::NoUpdate, target, output);
  auto published = output;
  published.position[0] = std::min(published.position[0], 1.0f);
  transition.begin(published, target, {0}, 0.3, output);
  transition.update(0.02, TargetUpdate::TargetReady, target, output);
  EXPECT_FLOAT_EQ(output.position[0], 1.0f);
  EXPECT_FLOAT_EQ(output.stiffness[0], published.stiffness[0]);
  EXPECT_FLOAT_EQ(output.damping[0], published.damping[0]);
  EXPECT_FLOAT_EQ(output.position[1], -0.5f);
}

TEST(CommandTransition, ResetDoesNotOverwriteDampingOrPostureCommand)
{
  auto output = initial();
  JointCommand target{{1.8f}, {80.0f}, {6.0f}};
  CommandTransition transition(3);
  transition.begin(output, target, {0}, 0.2, output);
  transition.update(0.02, TargetUpdate::TargetReady, target, output);
  transition.reset();
  output = {{0.1f, 0.2f, 0.3f}, {0.0f, 0.0f, 0.0f}, {3.0f, 3.0f, 3.0f}};
  const auto damping = output;
  transition.update(0.1, TargetUpdate::NoUpdate, target, output);
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

TEST(CommandTransition, MatchesPreviousOutputPathBitForBitAcrossTransitionsAndFailures)
{
  std::mt19937 rng(21092026);
  std::uniform_real_distribution<float> angle(-2.0f, 2.0f);
  std::uniform_real_distribution<float> gain(0.0f, 100.0f);
  std::uniform_real_distribution<double> period(0.001, 0.03);
  for (const double duration : {0.0, 0.2, 0.3, 0.5}) {
    CommandTransition actual(3);
    PreviousOutputPath reference;
    auto actual_output = initial();
    auto expected_output = actual_output;
    auto published = actual_output;
    for (int episode = 0; episode < 60; ++episode) {
      SCOPED_TRACE(episode);
      const std::vector<size_t> joints = episode % 2 == 0 ?
        std::vector<size_t>{2, 0} : std::vector<size_t>{1, 0, 2};
      JointCommand target;
      target.resize(joints.size());
      for (size_t j = 0; j < joints.size(); ++j) {
        target.position[j] = angle(rng);
        target.stiffness[j] = gain(rng);
        target.damping[j] = gain(rng);
      }
      // Covers published baseline, unpublished fallback and repeated transitions.
      const auto source = episode % 3 == 0 ? actual_output : published;
      actual.begin(source, target, joints, duration, actual_output);
      reference.enter(source, target, joints, duration, expected_output);
      expect_same(actual_output, expected_output);
      for (int tick = 0; tick < 120; ++tick) {
        SCOPED_TRACE(tick);
        auto result = tick % 5 == 0 ? TargetUpdate::TargetReady : TargetUpdate::NoUpdate;
        if (rng() % 13 == 0 || tick < episode % 5) {
          result = TargetUpdate::HoldOutput;
        }
        if (result == TargetUpdate::TargetReady) {
          for (auto & position : target.position) {
            position = angle(rng);
          }
        }
        double dt = period(rng);
        if (tick == 33) {dt = -0.01;}
        if (tick == 44) {dt = std::numeric_limits<double>::quiet_NaN();}
        if (tick == 55) {dt = 0.0;}
        // Model teleop-unavailable ticks: neither pipeline is called.
        if (tick % 23 != 0) {
          actual.update(dt, result, target, actual_output);
          reference.update(dt, result, target, expected_output);
        }
        expect_same(actual_output, expected_output);
        published = actual_output;
        for (auto & position : published.position) {
          position = std::clamp(position, -1.0f, 1.0f);
        }
        // Some episodes end mid-blend to exercise re-entry.
        if (episode % 4 == 0 && tick == 4) {break;}
      }
    }
  }
}
