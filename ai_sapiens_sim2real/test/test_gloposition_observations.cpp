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

#include <filesystem>
#include <fstream>
#include <limits>

#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/observation/observations.hpp"
#include "ai_sapiens_sim2real/policy/action_pipeline.hpp"
#include "ai_sapiens_sim2real/policy/localization_pose.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/motion_steering_release.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

class GlobalPositionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    file_ = std::filesystem::temp_directory_path() / "gloposition_test_motion.csv";
    std::ofstream(file_) <<
      "2,3,0.8,0,0,0,1,0\n"
      "3,4,0.8,0,0,0,1,0.1\n"
      "6,5,0.8,0,0,0,1,0.4\n";
  }
  void TearDown() override {std::filesystem::remove(file_);}
  std::filesystem::path file_;
};

TEST_F(GlobalPositionTest, MjlabUsesCentralVelocityAndExactFrames)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  EXPECT_NEAR(motion.duration(), 0.04, 1e-6);
  motion.seek(0.02);
  EXPECT_TRUE(motion.root_position().isApprox(Eigen::Vector3f(3, 4, 0.8)));
  EXPECT_NEAR(motion.joint_pos()[0], 0.1, 1e-6);
  EXPECT_NEAR(motion.joint_vel()[0], 10.0, 1e-5);
  motion.seek(0.01);
  EXPECT_NEAR(motion.root_position().x(), 2.5, 1e-5);
  motion.seek(100.0);
  EXPECT_NEAR(motion.root_position().x(), 6.0, 1e-5);
  EXPECT_THROW(motion.seek(std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
}

TEST_F(GlobalPositionTest, LegacyVelocityRemainsForwardDifference)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"});
  motion.seek(0.02);
  EXPECT_NEAR(motion.joint_vel()[0], 15.0, 1e-5);
  EXPECT_NEAR(motion.duration(), 0.06, 1e-6);
}

TEST_F(GlobalPositionTest, RejectsNonFiniteMotion)
{
  std::ofstream(file_) << "nan,0,0,0,0,0,1,0\n";
  EXPECT_THROW(MotionReference(file_.string(), 50, {"waist_yaw_joint"}, true),
    std::runtime_error);
}

TEST(MotionFrameAlignment, RotatesDisplacementAndAlignsHeadingOnce)
{
  MotionFrameAlignment frame;
  Eigen::Quaternionf quarter_turn(Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitZ()));
  frame.align(Eigen::Vector2f(10, 20), quarter_turn,
    Eigen::Vector2f(2, 3), Eigen::Quaternionf::Identity());
  EXPECT_TRUE(frame.position(Eigen::Vector2f(10, 20)).isZero(1e-5));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(2, 3)).isZero(1e-5));
  EXPECT_TRUE(frame.position(Eigen::Vector2f(10, 21)).isApprox(Eigen::Vector2f(1, 0), 1e-5));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(3, 3)).isApprox(Eigen::Vector2f(1, 0)));
  EXPECT_TRUE(frame.orientation(quarter_turn).isApprox(Eigen::Quaternionf::Identity(), 1e-5));
  frame.reset();
  EXPECT_TRUE(frame.position(Eigen::Vector2f(10, 21)).isApprox(Eigen::Vector2f(10, 21)));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(3, 3)).isApprox(Eigen::Vector2f(3, 3)));
}

TEST(MotionFrameAlignment, ReentryAfterWalkingCapturesNewPositionAndYaw)
{
  MotionFrameAlignment frame;
  frame.align(Eigen::Vector2f(10, 20), Eigen::Quaternionf::Identity(),
    Eigen::Vector2f(2, 3), Eigen::Quaternionf::Identity());
  EXPECT_TRUE(frame.position(Eigen::Vector2f(12, 21)).isApprox(Eigen::Vector2f(2, 1)));

  // A later dance starts after walking and turning, with a different reference start.
  const Eigen::Quaternionf quarter_turn(
    Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitZ()));
  frame.align(Eigen::Vector2f(12, 21), quarter_turn,
    Eigen::Vector2f(6, -4), Eigen::Quaternionf::Identity());
  EXPECT_TRUE(frame.position(Eigen::Vector2f(12, 21)).isZero(1e-5));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(6, -4)).isZero(1e-5));
  EXPECT_TRUE(frame.position(Eigen::Vector2f(12, 22)).isApprox(Eigen::Vector2f(1, 0), 1e-5));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(7, -4)).isApprox(Eigen::Vector2f(1, 0)));
  EXPECT_TRUE(frame.orientation(quarter_turn).isApprox(Eigen::Quaternionf::Identity(), 1e-5));
}

TEST_F(GlobalPositionTest, RegistryUsesMotionCoordinatesAndEstimatorOrientation)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  SharedControlData shared;
  shared.resize(1, 1);
  shared.sensors.joint_pos.setZero();
  shared.policy.uses_global_position = true;
  shared.localization.position = Eigen::Vector2f(10, 20);
  shared.localization.orientation = Eigen::Quaternionf::Identity();
  // Deliberately disagree with raw IMU yaw: the global policy must use odometry.
  shared.sensors.orientation = Eigen::AngleAxisf(1.0f, Eigen::Vector3f::UnitZ());
  shared.policy.motion_frame.align(shared.localization.position, shared.localization.orientation,
    Eigen::Vector2f(2, 3), motion.root_quaternion());
  PolicyJointContext joints;
  joints.policy_joint_names = {"waist_yaw_joint"};
  joints.policy_to_controller = {0};
  ObservationContext context{shared, joints, &motion};
  auto & registry = ObservationRegistry::get_registry();
  auto robot_xy = registry.at("robot_root_position_xy_w")(context, YAML::Node{});
  auto reference_xy = registry.at("reference_root_position_xy_w")(context, YAML::Node{});
  EXPECT_EQ(robot_xy, reference_xy);
  EXPECT_EQ(robot_xy, (std::vector<float>{0.0f, 0.0f}));
  auto orientation = registry.at("motion_anchor_ori_b")(context, YAML::Node{});
  EXPECT_EQ(orientation, (std::vector<float>{1, 0, 0, 1, 0, 0}));
  shared.localization.position.x() += 0.25f;
  robot_xy = registry.at("robot_root_position_xy_w")(context, YAML::Node{});
  EXPECT_FLOAT_EQ(robot_xy[0], 0.25f);
  motion.seek(0.02);
  reference_xy = registry.at("reference_root_position_xy_w")(context, YAML::Node{});
  EXPECT_EQ(reference_xy, (std::vector<float>{1.0f, 1.0f}));
  EXPECT_TRUE(shared.localization.position.isApprox(Eigen::Vector2f(10.25f, 20)));
  // A configured time_start uses that frame as the reference origin, not CSV row zero.
  shared.policy.motion_frame.align(shared.localization.position, shared.localization.orientation,
    motion.root_position().head<2>(), motion.root_quaternion());
  EXPECT_EQ(registry.at("robot_root_position_xy_w")(context, YAML::Node{}),
    (std::vector<float>{0.0f, 0.0f}));
  EXPECT_EQ(registry.at("reference_root_position_xy_w")(context, YAML::Node{}),
      (std::vector<float>{0.0f, 0.0f}));
}

TEST_F(GlobalPositionTest, ReadsSteeringFromPolicyAssetAndValidatesValues)
{
  auto config =
    YAML::Load(
        R"(
policy_joints: [waist_yaw_joint]
step_dt: 0.02
joint_properties:
  waist_yaw_joint: {default_position: 0, stiffness: 20, damping: 2}
actions:
  joint_pos: {scale: 0.25}
observations: {}
commands:
  reference_trajectory:
    steering:
      lin_vel_x: [-0.2, 0.25]
      lin_vel_y: [-0.1, 0.2]
      yaw_rate: [-0.15, 0.3]
      smoothing_time_constant: 0.8
)");
  auto load = [&]() {
      std::ofstream(file_) << YAML::Dump(config);
      return Sim2RealConfig(file_);
    };
  const auto parsed = load();
  ASSERT_TRUE(parsed.steering());
  EXPECT_DOUBLE_EQ(parsed.steering()->ranges.linear_x.max, 0.25);
  EXPECT_DOUBLE_EQ(parsed.steering()->ranges.linear_y.min, -0.1);
  EXPECT_DOUBLE_EQ(parsed.steering()->ranges.angular_z.min, -0.15);
  EXPECT_FLOAT_EQ(parsed.steering()->smoothing_time_constant, 0.8f);
  EXPECT_FALSE(parsed.velocity_command_ranges());
  auto steering = config["commands"]["reference_trajectory"]["steering"];
  steering["tracking_mode"] = "trajectory";
  EXPECT_NO_THROW(load());
  steering["tracking_mode"] = "velocity";
  EXPECT_THROW(load(), std::runtime_error);
  steering["tracking_mode"] = "invalid";
  EXPECT_THROW(load(), std::runtime_error);
  steering.remove("tracking_mode");
  steering["velocity_estimator_time_constant"] = .1;
  EXPECT_THROW(load(), std::runtime_error);
  steering.remove("velocity_estimator_time_constant");
  for (const auto & limits : {"[0.1, 0.3]", "[.nan, 0.3]", "[-.inf, 0.3]", "[-0.3]"}) {
    steering["lin_vel_x"] = YAML::Load(limits);
    EXPECT_THROW(load(), std::runtime_error);
  }
  steering["lin_vel_x"] = YAML::Load("[-0.3, 0.3]");
  for (const auto & tau : {"-0.1", ".nan", ".inf"}) {
    steering["smoothing_time_constant"] = YAML::Load(tau);
    EXPECT_THROW(load(), std::runtime_error);
  }
  steering.remove("smoothing_time_constant");
  EXPECT_THROW(load(), std::runtime_error);
  config["commands"] = YAML::Load("{}");
  EXPECT_FALSE(load().steering());
  config["commands"] =
    YAML::Load(
        R"(
base_velocity:
  ranges:
    lin_vel_x: [-1, 1]
    lin_vel_y: [-0.5, 0.5]
    ang_vel_z: [-2, 2]
)");
  EXPECT_FALSE(load().steering());
  ASSERT_TRUE(load().velocity_command_ranges());
  EXPECT_DOUBLE_EQ(load().velocity_command_ranges()->angular_z.max, 2.0);
}

TEST(PlanarMotionSteering, ZeroInputPreservesOriginalMovingClip)
{
  PlanarMotionSteering steering;
  PlanarSteeringConfig config;
  for (int i = 0; i < 100; ++i) {
    steering.step(Eigen::Vector2f(i, -i), Eigen::Vector2f(i + 1, -i - 1),
      Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(), 0.02f, config);
  }
  EXPECT_TRUE(steering.offset.isZero());
  EXPECT_TRUE(steering.velocity.isZero());
  EXPECT_FLOAT_EQ(steering.yaw, 0.0f);
}

TEST(PlanarMotionSteering, TranslationFollowsMeasuredRobotHeadingAndClamps)
{
  PlanarMotionSteering steering;
  PlanarSteeringConfig config;
  config.smoothing_time_constant = 0;
  const Eigen::Quaternionf quarter_turn(Eigen::AngleAxisf(1.57079632679f,
      Eigen::Vector3f::UnitZ()));
  for (int i = 0; i < 50; ++i) {
    steering.step(Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), quarter_turn,
      Eigen::Vector3f(5, 0, 0), 0.02f, config);
  }
  EXPECT_TRUE(steering.offset.isApprox(Eigen::Vector2f(0, 0.3f), 1e-5));
  EXPECT_FLOAT_EQ(steering.velocity.x(), 0.3f);
  const Eigen::Vector2f offset = steering.offset;
  steering.step(Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), quarter_turn,
    Eigen::Vector3f::Zero(), 0.02f, config);
  EXPECT_TRUE(steering.offset.isApprox(offset));
}

TEST(PlanarMotionSteering, MidpointYawRedirectsClipDisplacement)
{
  PlanarMotionSteering steering;
  steering.yaw = 1.57079632679f;
  PlanarSteeringConfig config;
  config.smoothing_time_constant = 0;
  steering.step(Eigen::Vector2f(2, 3), Eigen::Vector2f(3, 3),
    Eigen::Quaternionf::Identity(), Eigen::Vector3f(0, 0, 0.2f), 0.02f, config);
  const float mid = 1.57079632679f + 0.002f;
  EXPECT_NEAR(steering.offset.x(), std::cos(mid) - 1.0f, 1e-6);
  EXPECT_NEAR(steering.offset.y(), std::sin(mid), 1e-6);
  EXPECT_NEAR(steering.yaw, 1.57079632679f + 0.004f, 1e-6);
  // A released yaw command holds accumulated heading and still redirects the clip.
  const Eigen::Vector2f old_offset = steering.offset;
  steering.step(Eigen::Vector2f(3, 3), Eigen::Vector2f(4, 3),
    Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(), 0.02f, config);
  EXPECT_NEAR((steering.offset - old_offset).x(), std::cos(steering.yaw) - 1.0f, 1e-6);
}

TEST(PlanarMotionSteering, SmoothsReleaseWithoutSnappingAndResetsEveryEntry)
{
  PlanarMotionSteering steering;
  PlanarSteeringConfig config;
  const Eigen::Vector3f command(0.1f, -0.2f, 0.3f);
  steering.step(Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(),
    Eigen::Quaternionf::Identity(), command, 0.02f, config);
  EXPECT_TRUE(steering.velocity.isApprox(command * (1.0f - std::exp(-0.04f)), 1e-5));
  const auto prior = steering.velocity.eval();
  steering.step(Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(),
    Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(), 0.02f, config);
  EXPECT_TRUE(steering.velocity.isApprox(prior * std::exp(-0.04f), 1e-5));
  for (int i = 0; i < 1000; ++i) {
    steering.step(Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(),
      Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(), 0.02f, config);
  }
  EXPECT_TRUE(steering.velocity.isZero(1e-6));
  EXPECT_GT(steering.offset.norm(), 0.0f);
  EXPECT_GT(steering.yaw, 0.0f);
  steering.reset();
  EXPECT_TRUE(steering.offset.isZero());
  EXPECT_TRUE(steering.velocity.isZero());
  EXPECT_FLOAT_EQ(steering.yaw, 0.0f);
}

TEST_F(GlobalPositionTest, SteeringChangesReferenceAndAppliedCommandOnly)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  SharedControlData shared;
  shared.resize(1, 1);
  shared.policy.uses_global_position = true;
  shared.policy.uses_motion_steering = true;
  shared.policy.motion_frame.align(Eigen::Vector2f::Zero(), Eigen::Quaternionf::Identity(),
    Eigen::Vector2f(2, 3), Eigen::Quaternionf::Identity());
  shared.policy.motion_steering.offset = Eigen::Vector2f(0.1f, 0.2f);
  shared.policy.motion_steering.yaw = 1.57079632679f;
  shared.policy.motion_steering.velocity = Eigen::Vector3f(0.01f, -0.02f, 0.03f);
  shared.mode.velocity_commands = Eigen::Vector3f(0.1f, -0.2f, 0.3f);
  PolicyJointContext joints{{"waist_yaw_joint"}, {0}};
  ObservationContext context{shared, joints, &motion};
  auto & registry = ObservationRegistry::get_registry();
  const std::vector<float> reference_xy{0.1f, 0.2f};
  const std::vector<float> robot_xy{0, 0};
  const std::vector<float> applied_velocity{0.01f, -0.02f, 0.03f};
  EXPECT_EQ(registry.at("reference_root_position_xy_w")(context, YAML::Node{}), reference_xy);
  EXPECT_EQ(registry.at("robot_root_position_xy_w")(context, YAML::Node{}), robot_xy);
  EXPECT_EQ(registry.at("velocity_commands")(context, YAML::Node{}), applied_velocity);
  const auto anchor = registry.at("motion_anchor_ori_b")(context, YAML::Node{});
  const std::vector<float> expected{0, -1, 1, 0, 0, 0};
  for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_NEAR(anchor[i], expected[i], 1e-6);
  }
  shared.policy.uses_motion_steering = false;
  shared.policy.motion_steering.reset();
  EXPECT_EQ(registry.at("velocity_commands")(context, YAML::Node{}),
    (std::vector<float>{0.1f, -0.2f, 0.3f}));
}

TEST(ActionPipeline, ClipsRawBeforeScaleAndKeepsAppliedHistory)
{
  ActionProperties properties;
  properties.scale = {0.5f};
  properties.offset = {-0.3f};
  properties.clip = {std::nullopt};
  properties.raw_clip = {std::make_pair(-10.0f, 10.0f)};
  ActionPipeline pipeline(properties);
  EXPECT_NEAR(pipeline.process({20.0f})[0], 4.7f, 1e-6);
  EXPECT_FLOAT_EQ(pipeline.applied_raw_action()[0], 10.0f);
  EXPECT_FALSE(std::isfinite(pipeline.process({std::numeric_limits<float>::infinity()})[0]));
}

TEST_F(GlobalPositionTest, BlockedRobotRetainsReferencePositionErrorAfterRelease)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  SharedControlData shared;
  shared.resize(1, 1);
  shared.policy.uses_global_position = true;
  shared.policy.uses_motion_steering = true;
  shared.policy.motion_frame.align(Eigen::Vector2f::Zero(), Eigen::Quaternionf::Identity(),
    motion.root_position().head<2>(), motion.root_quaternion());
  PolicyJointContext joints{{"waist_yaw_joint"}, {0}};
  ObservationContext context{shared, joints, &motion};
  auto & registry = ObservationRegistry::get_registry();
  auto & steering = shared.policy.motion_steering;
  PlanarSteeringConfig config;
  config.smoothing_time_constant = 0;
  const Eigen::Vector2f root = motion.root_position().head<2>();
  // The robot remains blocked while the command advances its reference for 10 seconds.
  for (int i = 0; i < 500; ++i) {
    steering.step(root, root, Eigen::Quaternionf::Identity(),
      Eigen::Vector3f(0.1f, -0.1f, 0.2f), 0.02f, config);
  }
  const auto robot = registry.at("robot_root_position_xy_w")(context, YAML::Node{});
  const auto reference = registry.at("reference_root_position_xy_w")(context, YAML::Node{});
  EXPECT_EQ(robot, (std::vector<float>{0, 0}));
  EXPECT_NEAR(reference[0], 1.0f, 1e-5);
  EXPECT_NEAR(reference[1], -1.0f, 1e-5);
  EXPECT_NEAR(steering.yaw, 2.0f, 2e-5);
  steering.step(root, root, Eigen::Quaternionf::Identity(),
    Eigen::Vector3f::Zero(), 0.02f, config);
  EXPECT_TRUE(steering.velocity.isZero());
  EXPECT_EQ(registry.at("reference_root_position_xy_w")(context, YAML::Node{}), reference);
  EXPECT_NEAR(steering.yaw, 2.0f, 2e-5);
  steering.reset();
  EXPECT_EQ(registry.at("reference_root_position_xy_w")(context, YAML::Node{}), robot);
}

TEST_F(GlobalPositionTest, RejectsObsoleteVelocityObservationSchema)
{
  auto & registry = ObservationRegistry::get_registry();
  EXPECT_THROW(registry.at("robot_root_velocity_xy_h"), std::out_of_range);
  EXPECT_THROW(registry.at("reference_root_velocity_xy_h"), std::out_of_range);
}

TEST_F(GlobalPositionTest, ReleaseUsesReachedEpisodePoseThenContinuesOriginalClip)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  SharedControlData shared;
  shared.resize(1, 1);
  shared.policy.uses_global_position = true;
  shared.policy.uses_motion_steering = true;
  const Eigen::Quaternionf entry_yaw(Eigen::AngleAxisf(0.7f, Eigen::Vector3f::UnitZ()));
  shared.policy.motion_frame.align(Eigen::Vector2f(10, 20), entry_yaw,
    motion.root_position().head<2>(), motion.root_quaternion());
  shared.localization.position = Eigen::Vector2f(10, 20) +
    Eigen::Rotation2Df(0.7f) * Eigen::Vector2f(0.7f, 0);
  shared.localization.orientation = Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitZ());
  const auto robot_xy = shared.policy.motion_frame.position(shared.localization.position);
  const auto robot_q = shared.policy.motion_frame.orientation(shared.localization.orientation);
  auto & steering = shared.policy.motion_steering;
  steering.offset = Eigen::Vector2f(1, 0);  // Commanded 1 m, only reached 0.7 m.
  steering.yaw = 0.4f;  // Also discard the missed turn on release.
  MotionSteeringRelease release;
  const auto apply = [&](const Eigen::Vector3f & requested) {
      release.apply(steering, requested, robot_xy, robot_q,
        shared.policy.motion_frame.reference_position(motion.root_position().head<2>()),
        motion.root_quaternion());
    };
  apply(Eigen::Vector3f(0.2f, 0, 0.2f));
  EXPECT_TRUE(steering.offset.isApprox(Eigen::Vector2f(1, 0)));
  apply(Eigen::Vector3f::Zero());
  EXPECT_TRUE(steering.offset.isApprox(Eigen::Vector2f(0.7f, 0), 2e-6));
  EXPECT_NEAR(steering.yaw, 0.1f, 1e-6);

  PolicyJointContext joints{{"waist_yaw_joint"}, {0}};
  ObservationContext context{shared, joints, &motion};
  auto & registry = ObservationRegistry::get_registry();
  EXPECT_EQ(registry.at("reference_root_position_xy_w")(context, YAML::Node{}),
    registry.at("robot_root_position_xy_w")(context, YAML::Node{}));
  const auto old_root = motion.root_position().head<2>().eval();
  motion.seek(0.02);
  steering.step(old_root, motion.root_position().head<2>(), robot_q,
    Eigen::Vector3f::Zero(), 0.02f, PlanarSteeringConfig{});
  apply(Eigen::Vector3f::Zero());
  const auto reference = registry.at("reference_root_position_xy_w")(context, YAML::Node{});
  const Eigen::Vector2f expected = robot_xy +
    Eigen::Rotation2Df(0.1f) * Eigen::Vector2f(1, 1);
  EXPECT_NEAR(reference[0], expected.x(), 2e-6);
  EXPECT_NEAR(reference[1], expected.y(), 2e-6);
  EXPECT_NEAR(motion.joint_pos()[0], 0.1f, 1e-6);  // Playback was not restarted.
}

TEST(MotionSteeringRelease, DecaysCommandWhileDiscardingDebtThenStopsReanchoring)
{
  PlanarMotionSteering steering;
  MotionSteeringRelease release;
  PlanarSteeringConfig config;
  const auto q = Eigen::Quaternionf::Identity();
  const auto root = Eigen::Vector2f::Zero().eval();
  const Eigen::Vector3f command(0.3f, 0, 0.3f);
  for (int i = 0; i < 50; ++i) {
    steering.step(root, root, q, command, 0.02f, config);
    release.apply(steering, command, root, q, root, q);
  }
  ASSERT_GT(steering.offset.norm(), 0.1f);
  Eigen::Vector2f reached(0.07f, -0.1f);
  const auto before = steering.velocity.eval();
  steering.step(root, root, q, Eigen::Vector3f::Zero(), 0.02f, config);
  release.apply(steering, Eigen::Vector3f::Zero(), reached, q, root, q);
  EXPECT_TRUE(steering.velocity.isApprox(before * std::exp(-0.04f), 1e-6));
  EXPECT_TRUE(steering.offset.isApprox(reached));
  EXPECT_FLOAT_EQ(steering.yaw, 0);
  for (int i = 0; i < 200 && !steering.velocity.isZero(); ++i) {
    reached.x() += 0.001f;  // Measured motion during deceleration.
    steering.step(root, root, q, Eigen::Vector3f::Zero(), 0.02f, config);
    release.apply(steering, Eigen::Vector3f::Zero(), reached, q, root, q);
    EXPECT_TRUE(steering.offset.isApprox(reached));
  }
  ASSERT_TRUE(steering.velocity.isZero());
  const auto settled = steering.offset.eval();
  reached.x() += 2.0f;
  release.apply(steering, Eigen::Vector3f::Zero(), reached, q, root, q);
  EXPECT_TRUE(steering.offset.isApprox(settled));
}

TEST(MotionSteeringRelease, InitialNeutralAndResetDoNotDiscardDanceTrackingError)
{
  PlanarMotionSteering steering;
  MotionSteeringRelease release;
  const auto q = Eigen::Quaternionf::Identity();
  const auto zero = Eigen::Vector2f::Zero().eval();
  const Eigen::Vector2f measured(1, 2);
  release.apply(steering, Eigen::Vector3f::Zero(), measured, q, zero, q);
  EXPECT_TRUE(steering.offset.isZero());
  // Even a command held on the first zero-velocity frame arms the release.
  release.apply(steering, Eigen::Vector3f(0.2f, 0, 0), measured, q, zero, q);
  release.apply(steering, Eigen::Vector3f::Zero(), measured, q, zero, q);
  EXPECT_TRUE(steering.offset.isApprox(measured));
  release.apply(steering, Eigen::Vector3f(0.2f, 0, 0), measured, q, zero, q);
  release.reset();
  steering.reset();
  release.apply(steering, Eigen::Vector3f::Zero(), measured, q, zero, q);
  EXPECT_TRUE(steering.offset.isZero());
}

TEST(MotionSteeringRelease, TranslationReleaseWorksWhileTurnRemainsActive)
{
  PlanarMotionSteering steering;
  MotionSteeringRelease release;
  PlanarSteeringConfig config;
  const auto q = Eigen::Quaternionf::Identity();
  const auto zero = Eigen::Vector2f::Zero().eval();
  const Eigen::Vector2f measured(0.7f, -0.2f);
  steering.velocity = Eigen::Vector3f(0.2f, 0, 0.2f);
  steering.offset = Eigen::Vector2f(1, 0);
  release.apply(steering, steering.velocity, measured, q, zero, q);
  release.apply(steering, Eigen::Vector3f(0, 0, 0.2f), measured, q, zero, q);
  EXPECT_TRUE(steering.offset.isApprox(measured));
  EXPECT_FLOAT_EQ(steering.velocity.z(), 0.2f);
  release.apply(steering, Eigen::Vector3f::Zero(), measured, q, zero, q);
  EXPECT_TRUE(steering.offset.isApprox(measured));
  const Eigen::Vector3f reverse(-0.2f, 0, -0.2f);
  steering.step(zero, zero, q, reverse, 0.02f, config);
  const auto integrated = steering.offset.eval();
  release.apply(steering, reverse, measured, q, zero, q);
  EXPECT_TRUE(steering.offset.isApprox(integrated));
  EXPECT_FALSE(steering.offset.isApprox(measured));
  release.apply(steering, Eigen::Vector3f::Zero(), measured, q, zero, q);
  EXPECT_LT((steering.offset - measured).norm(), (integrated - measured).norm());
}

TEST(MotionSteeringRelease, CommandsWithinDeadbandDoNotAccumulateOrCancelRelease)
{
  MotionSteeringRelease release;
  PlanarMotionSteering steering;
  PlanarSteeringConfig config;
  const Eigen::Vector2f zero = Eigen::Vector2f::Zero();
  const auto q = Eigen::Quaternionf::Identity();
  const auto tick = [&](Eigen::Vector3f raw) {
      const auto command = release.filter_command(raw);
      steering.step(zero, zero, q, command, 0.02f, config);
      release.apply(steering, command, zero, q, zero, q);
    };
  for (int i = 0; i < 10000; ++i) {
    tick(Eigen::Vector3f(0.1f, -0.1f, 0.09f));
  }
  EXPECT_TRUE(steering.offset.isZero());
  EXPECT_FLOAT_EQ(steering.yaw, 0);
  for (int i = 0; i < 200; ++i) {
    tick(Eigen::Vector3f(0.3f, 0, 0));
  }
  ASSERT_GT(steering.offset.norm(), 1.0f);
  for (int i = 0; i < 500; ++i) {
    tick(Eigen::Vector3f(0.1f, -0.09f, -0.1f));
  }
  EXPECT_LT(steering.offset.norm(), 1e-6);
  EXPECT_TRUE(steering.velocity.isZero());
  EXPECT_FLOAT_EQ(steering.yaw, 0);
}

TEST(MotionSteeringRelease, DeadbandIncludesBothBoundariesAndPreservesOtherAxes)
{
  MotionSteeringRelease release;
  const float above = std::nextafter(0.1f, 1.0f);
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3f command(0.2f, -0.2f, 0.3f);
    for (const float value : {-0.1f, -0.09f, 0.0f, 0.09f, 0.1f}) {
      command[axis] = value;
      auto expected = command.eval();
      expected[axis] = 0.0f;
      EXPECT_TRUE(release.filter_command(command).isApprox(expected));
    }
  }
  const Eigen::Vector3f outside(above, -above, above);
  EXPECT_TRUE(release.filter_command(outside).isApprox(outside));
  // Returning from an active command must use the same threshold.
  EXPECT_TRUE(release.filter_command(Eigen::Vector3f::Constant(0.1f)).isZero());
}

TEST(MotionSteeringRelease, TranslationOnlyReleasePreservesHeading)
{
  MotionSteeringRelease release;
  PlanarMotionSteering steering;
  steering.yaw = 0.6f;
  const auto q = Eigen::Quaternionf::Identity();
  const Eigen::Vector2f zero = Eigen::Vector2f::Zero();
  release.apply(steering, Eigen::Vector3f(0.2f, 0, 0), zero, q, zero, q);
  for (int i = 0; i < 200; ++i) {
    release.apply(steering, Eigen::Vector3f::Zero(), zero, q, zero, q);
  }
  EXPECT_FLOAT_EQ(steering.yaw, 0.6f);
}

TEST(MotionSteeringRelease, TurnReleaseWrapsHeadingWithoutChangingActiveTranslation)
{
  MotionSteeringRelease release;
  PlanarMotionSteering steering;
  steering.yaw = 3.13f;
  steering.offset = Eigen::Vector2f(0.7f, 0.2f);
  const Eigen::Quaternionf q(Eigen::AngleAxisf(-3.13f, Eigen::Vector3f::UnitZ()));
  const Eigen::Vector2f zero = Eigen::Vector2f::Zero();
  const auto ref_q = Eigen::Quaternionf::Identity();
  release.apply(steering, Eigen::Vector3f(0.2f, 0, 0.2f), zero, q, zero, ref_q);
  release.apply(steering, Eigen::Vector3f(0.2f, 0, 0), zero, q, zero, ref_q);
  EXPECT_NEAR(steering.yaw, -3.13f, 1e-6);
  EXPECT_TRUE(steering.offset.isApprox(Eigen::Vector2f(0.7f, 0.2f)));
}

}  // namespace
}  // namespace ai_sapiens_sim2real
