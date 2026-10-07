#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "ai_sapiens_sim2real/policy/carry_mode.hpp"

TEST(CarryMode, RaiseLowerAndReset)
{
  k1_carry::ModeMachine machine;
  for (int i = 0; i < 100; ++i) {
    machine.step(true, false, false, {1.5F, 1.0F, 1.57F});
  }
  ASSERT_EQ(machine.state, k1_carry::CARRY);
  EXPECT_FLOAT_EQ(machine.mode()[0], 1.0F);
  EXPECT_FLOAT_EQ(machine.mode()[1], 1.0F);
  for (int j = 0; j < 3; ++j) {
    EXPECT_LE(std::abs(machine.command[j]), machine.params.carry_limits[j] + 1e-6F);
  }
  for (int i = 0; i < 100; ++i) {
    machine.step(false, false, true, {0.0F, 0.0F, 0.0F});
  }
  EXPECT_EQ(machine.state, k1_carry::WALK);
  EXPECT_FLOAT_EQ(machine.mode()[0], 0.0F);
  EXPECT_FLOAT_EQ(machine.mode()[1], 0.0F);
  machine.reset();
  EXPECT_EQ(machine.command, (std::array<float, 3>{}));
  EXPECT_EQ(machine.mode(), (std::array<float, 6>{}));
}

TEST(CarryMode, BowWaitsForQuietAndDoesNotRetriggerWhileHeld)
{
  k1_carry::ModeMachine machine;
  machine.step(false, true, false, {0.5F, 0.0F, 0.0F});
  EXPECT_TRUE(machine.bow_pending);
  for (int i = 0; i < 100; ++i) {
    machine.step(false, true, false, {0.5F, 0.0F, 0.0F});
    EXPECT_EQ(machine.state, k1_carry::WALK);
    EXPECT_FLOAT_EQ(machine.command[0], 0.0F);
  }
  for (int i = 0; i < 36; ++i) {
    machine.step(false, true, true, {0.0F, 0.0F, 0.0F});
  }
  EXPECT_GE(machine.state, k1_carry::BOW_IN);
  EXPECT_FLOAT_EQ(machine.mode()[2], 1.0F);
  for (int i = 0; i < 300; ++i) {
    machine.step(false, true, true, {0.0F, 0.0F, 0.0F});
  }
  EXPECT_EQ(machine.state, k1_carry::WALK);
  EXPECT_FALSE(machine.bow_pending);
}

TEST(CarryMode, SaCancelsPendingBow)
{
  k1_carry::ModeMachine machine;
  machine.step(false, true, false, {0.0F, 0.0F, 0.0F});
  ASSERT_TRUE(machine.bow_pending);
  machine.step(true, true, false, {0.0F, 0.0F, 0.0F});
  EXPECT_FALSE(machine.bow_pending);
  EXPECT_EQ(machine.state, k1_carry::ENTER);
}

TEST(CarryMode, LinearCommandsApplyImmediatelyWithinSpeedLimits)
{
  k1_carry::ModeMachine machine;
  machine.step(false, false, false, {1.2F, 0.7F, 0.0F});
  EXPECT_FLOAT_EQ(machine.command[0], 1.2F);
  EXPECT_FLOAT_EQ(machine.command[1], 0.7F);
  machine.step(false, false, false, {-1.2F, -0.7F, 0.0F});
  EXPECT_FLOAT_EQ(machine.command[0], -1.2F);
  EXPECT_FLOAT_EQ(machine.command[1], -0.7F);
  machine.step(false, false, false, {0.0F, 0.0F, 0.0F});
  EXPECT_FLOAT_EQ(machine.command[0], 0.0F);
  EXPECT_FLOAT_EQ(machine.command[1], 0.0F);
  machine.step(true, false, false, {5.0F, -5.0F, 0.0F});
  EXPECT_FLOAT_EQ(machine.command[0], machine.params.carry_limits[0]);
  EXPECT_FLOAT_EQ(machine.command[1], -machine.params.carry_limits[1]);
  machine.step(true, false, false, {-5.0F, 5.0F, 0.0F});
  EXPECT_FLOAT_EQ(machine.command[0], -machine.params.carry_limits[0]);
  EXPECT_FLOAT_EQ(machine.command[1], machine.params.carry_limits[1]);
  machine.step(true, false, false, {0.0F, 0.0F, 0.0F});
  EXPECT_FLOAT_EQ(machine.command[0], 0.0F);
  EXPECT_FLOAT_EQ(machine.command[1], 0.0F);
  for (int i = 0; i < 100; ++i) {
    machine.step(false, false, false, {5.0F, -5.0F, 0.0F});
  }
  ASSERT_EQ(machine.state, k1_carry::WALK);
  EXPECT_FLOAT_EQ(machine.command[0], machine.params.walk_limits[0]);
  EXPECT_FLOAT_EQ(machine.command[1], -machine.params.walk_limits[1]);
}

TEST(CarryMode, YawCommandsApplyImmediatelyWithinSpeedLimits)
{
  k1_carry::ModeMachine machine;
  for (float yaw : {1.2F, -1.2F, 0.0F}) {
    machine.step(false, false, false, {0.0F, 0.0F, yaw});
    EXPECT_FLOAT_EQ(machine.command[2], yaw);
  }
  machine.step(true, false, false, {0.0F, 0.0F, 5.0F});
  EXPECT_FLOAT_EQ(machine.command[2], machine.params.carry_limits[2]);
  machine.step(true, false, false, {0.0F, 0.0F, -5.0F});
  EXPECT_FLOAT_EQ(machine.command[2], -machine.params.carry_limits[2]);
  machine.step(true, false, false, {0.0F, 0.0F, 0.0F});
  EXPECT_FLOAT_EQ(machine.command[2], 0.0F);
  for (int i = 0; i < 100; ++i) {
    machine.step(false, false, false, {0.0F, 0.0F, 5.0F});
  }
  ASSERT_EQ(machine.state, k1_carry::WALK);
  EXPECT_FLOAT_EQ(machine.command[2], machine.params.walk_limits[2]);
  machine.step(false, true, false, {0.0F, 0.0F, 5.0F});
  EXPECT_TRUE(machine.bow_pending);
  EXPECT_FLOAT_EQ(machine.command[2], 0.0F);
}

TEST(CarryMode, RejectsNonFiniteCommand)
{
  k1_carry::ModeMachine machine;
  EXPECT_THROW(machine.step(false, false, false,
    {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}), std::invalid_argument);
}
