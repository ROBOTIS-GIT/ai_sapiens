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
#include "ai_sapiens_sim2real/mode_runtime/group_arbiter.hpp"

using Arbiter = ai_sapiens_sim2real::GroupArbiter;
using Command = Arbiter::Command;

class GroupTest : public ::testing::Test
{
protected:
  Arbiter arbiter;
  Arbiter::Input local{true, true, false, Command::Locomotion, 200, {}};
  Arbiter::Input common{true, true, false, Command::Locomotion, 201, {}};
  Arbiter::Output tick(bool locomotion = true) {return arbiter.update(local, common, locomotion);}
  void join() {tick(); local.join = true; ASSERT_TRUE(tick().participating);}
};

TEST_F(GroupTest, HeldJoinWaitsForLocomotionWithoutStartingHeldMimic)
{
  local.join = true;
  EXPECT_FALSE(tick(false).participating);
  EXPECT_TRUE(tick().participating);
  local.command = Command::Mimic;
  Arbiter fresh;
  EXPECT_EQ(fresh.update(local, common, true).command, Command::Locomotion);
  EXPECT_EQ(fresh.update(local, common, true).command, Command::Locomotion);
}
TEST_F(GroupTest, JoiningRequiresBothCommandsBothZerosAndActualLocomotion)
{
  tick(); local.join = true; common.velocity[0] = 0.4f; local.velocity[0] = -0.4f;
  EXPECT_FALSE(tick().participating);
  local.velocity[0] = 0; common.velocity[0] = 0;
  EXPECT_TRUE(tick().participating);  // held switch joins as soon as both commands are zero
  local.join = false; tick(); local.join = true;
  EXPECT_FALSE(tick(false).participating);
  local.join = false; tick(); common.command = Command::ReadyPose; local.join = true;
  EXPECT_FALSE(tick().participating);
  local.join = false; common.command = Command::Locomotion; tick();
  local.command = Command::Damping; local.join = true;
  EXPECT_FALSE(tick().participating);
}
TEST_F(GroupTest, StaleZeroCannotSatisfyJoin)
{
  tick(); local.join = true; common.velocity_fresh = false;
  EXPECT_FALSE(tick().participating);
}
TEST_F(GroupTest, AddsAndClampsEachNormalizedAxis)
{
  join(); local.velocity = {0.8f, -0.8f, 0.3f}; common.velocity = {0.6f, -0.6f, -0.2f};
  auto out = tick();
  EXPECT_FLOAT_EQ(out.velocity[0], 1); EXPECT_FLOAT_EQ(out.velocity[1], -1);
  EXPECT_NEAR(out.velocity[2], 0.1f, 1e-6f);
  local.join = false; out = tick();
  EXPECT_TRUE(out.exited); EXPECT_EQ(out.command, Command::Locomotion);
  EXPECT_FLOAT_EQ(out.velocity[0], 0.8f);
}
TEST_F(GroupTest, GroupLossExitsToLocomotionEvenWithLocalMimicHeld)
{
  join(); local.command = Command::Mimic; tick();
  common.valid = false; local.velocity[0] = 0.4f;
  auto out = tick(false);
  EXPECT_TRUE(out.exited); EXPECT_FALSE(out.participating);
  EXPECT_EQ(out.command, Command::Locomotion); EXPECT_FLOAT_EQ(out.velocity[0], 0.4f);
  common.valid = true;
  EXPECT_FALSE(tick().participating);
  EXPECT_EQ(tick().command, Command::Hold); // held local mimic cannot restart
}
TEST_F(GroupTest, LocalLossRevokesGroupAndSuppliesNoRequestOrVelocity)
{
  join(); local.valid = false;
  const auto out = tick();
  EXPECT_FALSE(out.participating); EXPECT_FALSE(out.request);
  EXPECT_EQ(out.velocity, (std::array<float, 3>{}));
}
TEST_F(GroupTest, IndividualIgnoresGroupLoss)
{
  tick(); common.valid = false; local.velocity[0] = 0.6f;
  const auto out = tick(); EXPECT_FALSE(out.exited); EXPECT_FLOAT_EQ(out.velocity[0], 0.6f);
}
TEST_F(GroupTest, LocalMimicWinsSimultaneousEdges)
{
  join(); local.command = common.command = Command::Mimic;
  auto out = tick(); EXPECT_EQ(out.command, Command::Mimic); EXPECT_EQ(out.selector, 200);
  EXPECT_TRUE(out.participating);
  EXPECT_EQ(tick(false).command, Command::Hold);
}
TEST_F(GroupTest, BothMimicSourcesAreAcceptedWhileGrouped)
{
  join(); common.command = Command::Mimic;
  EXPECT_EQ(tick().selector, 201);
  local.command = Command::Mimic;
  EXPECT_EQ(tick(false).command, Command::Hold); // no mimic -> mimic
  EXPECT_EQ(tick().command, Command::Hold); // no deferred request on completion
  local.command = Command::Locomotion; tick(); local.command = Command::Mimic;
  EXPECT_EQ(tick().selector, 200);
}
TEST_F(GroupTest, HeldGroupLocomotionDoesNotCancelLocalMimic)
{
  join(); local.command = Command::Mimic;
  ASSERT_EQ(tick().command, Command::Mimic);
  EXPECT_EQ(tick(false).command, Command::Hold);
  common.command = Command::Mimic; EXPECT_EQ(tick(false).command, Command::Hold);
  common.command = Command::Locomotion; EXPECT_EQ(tick(false).command, Command::Locomotion);
}
TEST_F(GroupTest, LeaveWinsBothMimicEdges)
{
  join(); local.join = false; local.command = common.command = Command::Mimic;
  EXPECT_EQ(tick().command, Command::Locomotion);
  EXPECT_EQ(tick().command, Command::Hold);
}
TEST_F(GroupTest, LocalPostureOverridesGroupAndExit)
{
  join(); local.command = Command::ReadyPose; common.command = Command::Mimic;
  EXPECT_EQ(tick().command, Command::ReadyPose);
  local.command = Command::Damping; local.join = false;
  EXPECT_EQ(tick(false).command, Command::Damping);
}
TEST_F(GroupTest, ReconnectionDoesNotSynthesizeMimicEdge)
{
  tick(); local.valid = false; tick(); local.command = Command::Mimic; local.valid = true;
  EXPECT_EQ(tick().command, Command::Hold);
}
TEST_F(GroupTest, MimicOutsideLocomotionIsConsumed)
{
  tick(false); local.command = Command::Mimic;
  EXPECT_EQ(tick(false).command, Command::Hold);
  EXPECT_EQ(tick(true).command, Command::Hold);
}
TEST_F(GroupTest, StaleVelocityZerosOnlyThatContribution)
{
  join(); local.velocity[0] = 0.3f; common.velocity[0] = 0.5f;
  common.velocity_fresh = false; EXPECT_FLOAT_EQ(tick().velocity[0], 0.3f);
  common.velocity_fresh = true; local.velocity_fresh = false;
  EXPECT_FLOAT_EQ(tick().velocity[0], 0.5f);
}

TEST_F(GroupTest, TransitionEventsAreEmittedOnce)
{
  using Event = Arbiter::Event;
  EXPECT_EQ(tick().event, Event::None);
  local.join = true;
  EXPECT_EQ(tick().event, Event::Joined);
  EXPECT_EQ(tick().event, Event::None);
  local.join = false;
  EXPECT_EQ(tick().event, Event::Left);
  EXPECT_EQ(tick().event, Event::None);
  local.join = true; tick(); common.valid = false;
  EXPECT_EQ(tick().event, Event::GroupLost);
  EXPECT_EQ(tick().event, Event::None);
  common.valid = true;
  EXPECT_EQ(tick().event, Event::Joined);
  EXPECT_EQ(tick().event, Event::None);
  local.join = false; tick(); local.join = true; tick();
  local.valid = false; common.valid = false;
  EXPECT_EQ(tick().event, Event::IndividualLost);
  EXPECT_EQ(tick().event, Event::None);
}

TEST_F(GroupTest, RejectedJoinReportsReasonOnlyOnFreshRequest)
{
  using Event = Arbiter::Event;
  const auto request = [this]() {
      local.join = false; tick(); local.join = true;
      return tick();
    };
  common.valid = false;
  EXPECT_EQ(request().event, Event::JoinGroupUnavailable);
  common.valid = true;
  local.join = false; tick(); local.join = true;
  EXPECT_EQ(tick(false).event, Event::JoinRobotNotLocomotion);
  local.command = Command::Damping;
  EXPECT_EQ(request().event, Event::JoinIndividualNotLocomotion);
  local.command = Command::Locomotion; common.command = Command::ReadyPose;
  EXPECT_EQ(request().event, Event::JoinGroupNotLocomotion);
  common.command = Command::Locomotion; local.velocity_fresh = false;
  EXPECT_EQ(request().event, Event::JoinIndividualVelocityStale);
  local.velocity_fresh = true; common.velocity_fresh = false;
  EXPECT_EQ(request().event, Event::JoinGroupVelocityStale);
  common.velocity_fresh = true; local.velocity[0] = 0.1f;
  EXPECT_EQ(request().event, Event::JoinIndividualMoving);
  local.velocity[0] = 0; common.velocity[2] = 0.1f;
  EXPECT_EQ(request().event, Event::JoinGroupMoving);
  EXPECT_EQ(tick().event, Event::None);
  common.velocity[2] = 0;
  EXPECT_EQ(tick().event, Event::Joined);
  EXPECT_TRUE(tick().participating);
  EXPECT_EQ(tick().event, Event::None);
}

TEST_F(GroupTest, JoinAcceptsHeldMimicAfterActualLocomotionReturn)
{
  tick(); local.command = Command::Mimic; tick();
  common.command = Command::Mimic; tick(false);
  local.join = true;
  auto out = tick();
  EXPECT_TRUE(out.participating);
  EXPECT_EQ(out.event, Arbiter::Event::Joined);
  EXPECT_EQ(out.command, Command::Locomotion);
  EXPECT_EQ(tick().command, Command::Hold);
  common.command = Command::Locomotion; tick();
  common.command = Command::Mimic;
  EXPECT_EQ(tick().command, Command::Mimic);
}

TEST_F(GroupTest, HeldJoinWaitsUntilActualMimicEnds)
{
  tick(); local.command = common.command = Command::Mimic; tick(false);
  local.join = true;
  EXPECT_EQ(tick(false).event, Arbiter::Event::JoinRobotNotLocomotion);
  EXPECT_FALSE(tick(false).participating);
  EXPECT_TRUE(tick().participating);
}

TEST_F(GroupTest, JoiningConsumesSimultaneousMimicEdges)
{
  tick(); local.join = true; local.command = common.command = Command::Mimic;
  const auto out = tick();
  EXPECT_TRUE(out.participating);
  EXPECT_EQ(out.command, Command::Locomotion);
  EXPECT_EQ(tick().command, Command::Hold);
}

TEST_F(GroupTest, ReadyPoseToLocomotionWithUnavailableGroup)
{
  common.valid = false;
  local.command = Command::ReadyPose;
  arbiter.update(local, common, false, true);
  local.command = Command::Locomotion;
  local.join = true;  // A rejected group request must not swallow local control.
  auto out = arbiter.update(local, common, false, true);
  EXPECT_EQ(out.event, Arbiter::Event::JoinGroupUnavailable);
  EXPECT_FALSE(out.participating);
  EXPECT_TRUE(out.request);
  EXPECT_EQ(out.command, Command::Locomotion);
  // Preserve the request while the posture transition is still pending.
  out = arbiter.update(local, common, false, true);
  EXPECT_TRUE(out.request);
  EXPECT_EQ(out.command, Command::Locomotion);
  local.velocity[0] = 0.4f;
  out = arbiter.update(local, common, true);
  EXPECT_FLOAT_EQ(out.velocity[0], 0.4f);
  local.valid = false;
  out = arbiter.update(local, common, true);
  EXPECT_FALSE(out.request);
  EXPECT_EQ(out.velocity, (std::array<float, 3>{}));
}

TEST_F(GroupTest, LocalMimicFromPostureStillRequiresFreshEdge)
{
  common.valid = false;
  local.command = Command::Mimic;
  EXPECT_EQ(arbiter.update(local, common, false, true).command, Command::Hold);
  local.valid = false;
  arbiter.update(local, common, false, true);
  local.valid = true;
  EXPECT_EQ(arbiter.update(local, common, false, true).command, Command::Hold);
  local.command = Command::ReadyPose;
  arbiter.update(local, common, false, true);
  local.command = Command::Mimic;
  const auto out = arbiter.update(local, common, false, true);
  EXPECT_EQ(out.command, Command::Mimic);
  EXPECT_EQ(out.selector, local.selector);
  EXPECT_EQ(arbiter.update(local, common, false, true).command, Command::Hold);
}

TEST_F(GroupTest, HeldJoinRequiresEveryAxisFreshAndIndividuallyZero)
{
  for (size_t axis = 0; axis < 3; ++axis) {
    Arbiter fresh;
    local.join = true;
    local.velocity[axis] = 0.2f; common.velocity[axis] = -0.2f;
    EXPECT_FALSE(fresh.update(local, common, true).participating);
    local.velocity[axis] = 0;
    EXPECT_FALSE(fresh.update(local, common, true).participating);
    common.velocity[axis] = 0; common.velocity_fresh = false;
    EXPECT_FALSE(fresh.update(local, common, true).participating);
    common.velocity_fresh = true; local.velocity_fresh = false;
    EXPECT_FALSE(fresh.update(local, common, true).participating);
    local.velocity_fresh = true;
    EXPECT_TRUE(fresh.update(local, common, true).participating);
  }
}
TEST_F(GroupTest, SwitchingOffCancelsPendingJoin)
{
  local.join = true; common.velocity[0] = 0.2f;
  EXPECT_FALSE(tick().participating);
  local.join = false; common.velocity[0] = 0;
  EXPECT_FALSE(tick().participating);
}
