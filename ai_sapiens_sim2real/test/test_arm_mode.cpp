#include <gtest/gtest.h>
#include "ai_sapiens_sim2real/policy/arm_mode.hpp"
#include "ai_sapiens_sim2real/policy/action_pipeline.hpp"

using ai_sapiens_sim2real::ArmMode;

static YAML::Node config()
{
  return YAML::Load(R"(
observations:
  arm_mode:
    params: {transition_seconds: 1.0}
actions:
  joint_pos:
    command_name: arm_mode
    ready_arm_joint_pos: {arm: 1.0}
commands:
  base_velocity:
    ranges: {lin_vel_x: [-1, 1.5], lin_vel_y: [-0.6, 0.6], ang_vel_z: [-1.57, 1.57]}
    hold_ranges: {lin_vel_x: [-0.5, 1], lin_vel_y: [-0.5, 0.5], ang_vel_z: [-1, 1]}
)");
}

TEST(ArmMode, TogglesOncePerPressAndBlendsInOneSecond)
{
  ArmMode mode(config(), {"leg", "arm"}, {0.2F, -0.4F}, 0.02);
  EXPECT_FLOAT_EQ(mode.offsets()[1], 1.0F);
  for (int i = 0; i < 25; ++i) mode.step(true);
  EXPECT_NEAR(mode.ratio(), 0.5F, 1e-6F);
  EXPECT_NEAR(mode.offsets()[1], 0.3F, 1e-6F);
  EXPECT_FLOAT_EQ(mode.offsets()[0], 0.2F);
  for (int i = 0; i < 50; ++i) mode.step(true);
  EXPECT_FLOAT_EQ(mode.ratio(), 1.0F);
  EXPECT_DOUBLE_EQ(mode.ranges().angular_z.max, 1.0);
  mode.step(false);
  mode.step(true);
  EXPECT_FALSE(mode.target());
  for (int i = 0; i < 49; ++i) mode.step(false);
  EXPECT_FLOAT_EQ(mode.ratio(), 0.0F);
  EXPECT_DOUBLE_EQ(mode.ranges().angular_z.max, 1.57);
  EXPECT_FLOAT_EQ(mode.offsets()[1], 1.0F);
}

TEST(ArmMode, HeldButtonOnEntryRequiresReleaseAndPress)
{
  ArmMode mode(config(), {"arm"}, {-0.4F}, 0.02);
  mode.reset(true);
  mode.step(true);
  EXPECT_FLOAT_EQ(mode.ratio(), 0.0F);
  mode.step(false);
  mode.step(true);
  EXPECT_TRUE(mode.target());
  EXPECT_GT(mode.ratio(), 0.0F);
}

TEST(ArmMode, ReversesMidTransitionWithoutPositionJump)
{
  ArmMode mode(config(), {"arm"}, {-0.4F}, 0.02);
  for (int i = 0; i < 20; ++i) mode.step(true);
  mode.step(false);
  const float ratio = mode.ratio();
  mode.step(true);
  EXPECT_NEAR(mode.ratio(), ratio - 0.02F, 1e-6F);
}

TEST(ArmMode, RejectsInvalidDeploymentParameters)
{
  auto node = config();
  node["observations"]["arm_mode"]["params"]["transition_seconds"] = 0;
  EXPECT_THROW(ArmMode(node, {"arm"}, {0.0F}, 0.02), std::runtime_error);
  EXPECT_THROW(ArmMode(config(), {"other"}, {0.0F}, 0.02), std::runtime_error);
}

TEST(ArmMode, OffsetOverridePreservesActionScaleAndClip)
{
  ai_sapiens_sim2real::ActionProperties props;
  props.scale = {0.5F};
  props.offset = {-0.4F};
  props.clip = {std::make_pair(-2.0F, 2.0F)};
  ai_sapiens_sim2real::ActionPipeline pipeline(props);
  std::vector<float> ready{1.0F};
  EXPECT_FLOAT_EQ(pipeline.process({1.0F}, &ready)[0], 1.5F);
  EXPECT_FLOAT_EQ(pipeline.process({10.0F}, &ready)[0], 2.0F);
  EXPECT_NEAR(pipeline.process({1.0F})[0], 0.1F, 1e-6F);
}
