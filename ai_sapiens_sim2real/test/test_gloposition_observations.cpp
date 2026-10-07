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
    directory_ = std::filesystem::temp_directory_path() / "gloposition_test";
    std::filesystem::create_directories(directory_);
    file_ = directory_ / "motion.csv";
    std::ofstream(file_) <<
      "2,3,0.8,0,0,0,1,0\n"
      "3,4,0.8,0,0,0,1,0.1\n"
      "6,5,0.8,0,0,0,1,0.4\n";
  }
  void TearDown() override {std::filesystem::remove_all(directory_);}
  std::filesystem::path directory_;
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

TEST_F(GlobalPositionTest, FrameInspectionPreservesPlaybackAndOriginalData)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  motion.seek(.02);
  EXPECT_NEAR(motion.frame(2).joint_pos[0], .4f, 1e-6);
  EXPECT_EQ(motion.frame_index(), 1);
  EXPECT_NEAR(motion.joint_pos()[0], .1f, 1e-6);
  motion.set_joint_targets(Eigen::VectorXf::Constant(1, .6f),
    Eigen::VectorXf::Constant(1, -.7f), Eigen::Vector3f(.01f, -.02f, 0));
  EXPECT_NEAR(motion.frame(1).joint_pos[0], .1f, 1e-6);
  EXPECT_NEAR(motion.joint_pos()[0], .6f, 1e-6);
  EXPECT_TRUE(motion.root_position().isApprox(Eigen::Vector3f(3, 4, .8f)));
  EXPECT_THROW(motion.frame(-1), std::out_of_range);
  EXPECT_THROW(motion.frame(3), std::out_of_range);
  EXPECT_THROW(motion.set_joint_targets(Eigen::VectorXf::Zero(2),
      Eigen::VectorXf::Zero(1), Eigen::Vector3f::Zero()), std::runtime_error);
  EXPECT_THROW(motion.set_joint_targets(Eigen::VectorXf::Constant(1,
      std::numeric_limits<float>::quiet_NaN()), Eigen::VectorXf::Zero(1),
      Eigen::Vector3f::Zero()), std::runtime_error);
  motion.seek(.02);
  EXPECT_NEAR(motion.joint_pos()[0], .1f, 1e-6);
  EXPECT_TRUE(motion.root_shift().isZero());
}

TEST_F(GlobalPositionTest, RetargetedMotionUsesTheExistingPolicyJointMapping)
{
  std::ofstream(file_) << "0,0,0.8,0,0,0,1,0.1,0.2\n"
    "0,0,0.8,0,0,0,1,0.2,0.4\n";
  MotionReference motion(file_.string(), 50, {"right", "left"}, true);
  motion.set_joint_targets(Eigen::Vector2f(.6f, .7f),
    Eigen::Vector2f(-.2f, .3f), Eigen::Vector3f::Zero());
  SharedControlData shared;
  PolicyJointContext joints{{"left", "right"}, {1, 0}};
  ObservationContext context{shared, joints, &motion};
  const auto & registry = ObservationRegistry::get_registry();
  EXPECT_EQ(registry.at("motion_command")(context, YAML::Node{}),
    (std::vector<float>{.7f, .6f, .3f, -.2f}));
  MotionReference other(file_.string(), 50, {"right", "left"}, true);
  ObservationContext other_context{shared, joints, &other};
  const auto command = registry.at("motion_command")(other_context, YAML::Node{});
  EXPECT_FLOAT_EQ(command[0], .2f);
  EXPECT_FLOAT_EQ(command[1], .1f);
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

TEST(MotionFrameAlignment, MotionOriginPreservesCoordinatesAcrossReentryAndPolicyChanges)
{
  MotionFrameAlignment frame;
  const Eigen::Quaternionf quarter_turn(
    Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitZ()));
  frame.align(Eigen::Vector2f(10, 20), quarter_turn,
    Eigen::Vector2f(2, 3), Eigen::Quaternionf::Identity(), MotionObservationOrigin::Motion);
  EXPECT_TRUE(frame.position(Eigen::Vector2f(10, 20)).isApprox(Eigen::Vector2f(2, 3)));
  EXPECT_TRUE(frame.position(Eigen::Vector2f(10, 21)).isApprox(Eigen::Vector2f(3, 3)));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(3, 4)).isApprox(Eigen::Vector2f(3, 4)));
  EXPECT_TRUE(frame.orientation(quarter_turn).isApprox(Eigen::Quaternionf::Identity(), 1e-5));

  // Walking before re-entry or starting at another motion frame must not zero
  // that frame's original coordinates or reuse the previous odometry origin.
  frame.align(Eigen::Vector2f(12, 21), Eigen::Quaternionf::Identity(),
    Eigen::Vector2f(6, -4), quarter_turn, MotionObservationOrigin::Motion);
  EXPECT_TRUE(frame.position(Eigen::Vector2f(12, 21)).isApprox(Eigen::Vector2f(6, -4)));
  EXPECT_TRUE(frame.position(Eigen::Vector2f(13, 21)).isApprox(Eigen::Vector2f(6, -3)));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(7, -4)).isApprox(Eigen::Vector2f(7, -4)));

  // Switching back to an unspecified/episode policy clears the motion offset.
  frame.align(Eigen::Vector2f(12, 21), quarter_turn,
    Eigen::Vector2f(6, -4), Eigen::Quaternionf::Identity());
  EXPECT_TRUE(frame.position(Eigen::Vector2f(12, 21)).isZero(1e-5));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(6, -4)).isZero(1e-5));
  frame.align(Eigen::Vector2f(12, 21), quarter_turn,
    Eigen::Vector2f(6, -4), Eigen::Quaternionf::Identity(), MotionObservationOrigin::Motion);
  frame.reset();
  EXPECT_TRUE(frame.position(Eigen::Vector2f(1, 2)).isApprox(Eigen::Vector2f(1, 2)));
  EXPECT_TRUE(frame.reference_position(Eigen::Vector2f(3, 4)).isApprox(Eigen::Vector2f(3, 4)));
}

TEST_F(GlobalPositionTest, ObservationOriginIsPerPolicyAndDoesNotRequireSteering)
{
  auto config = YAML::Load(R"(
policy_joints: [waist_yaw_joint]
step_dt: 0.02
joint_properties:
  waist_yaw_joint: {default_position: 0, stiffness: 20, damping: 2}
actions:
  joint_pos: {scale: 0.25}
observations: {}
commands: {}
)");
  const auto load = [&]() {
      std::ofstream(file_) << YAML::Dump(config);
      return Sim2RealConfig(file_);
    };
  EXPECT_EQ(load().observation_origin(), MotionObservationOrigin::Episode);
  auto reference = config["commands"]["reference_trajectory"];
  EXPECT_FALSE(load().use_imu_orientation());
  reference["orientation_source"] = "imu";
  const auto imu_config = load();
  EXPECT_TRUE(imu_config.use_imu_orientation());
  reference["orientation_source"] = "localization";
  EXPECT_FALSE(load().use_imu_orientation());
  EXPECT_TRUE(imu_config.use_imu_orientation());
  reference["orientation_source"] = "unknown";
  EXPECT_THROW(load(), std::runtime_error);
  reference.remove("orientation_source");
  reference["observation_origin"] = "motion";
  const auto motion_config = load();
  EXPECT_EQ(motion_config.observation_origin(), MotionObservationOrigin::Motion);
  EXPECT_FALSE(motion_config.steering());
  EXPECT_FALSE(motion_config.dance_reference());
  reference["observation_origin"] = "episode";
  EXPECT_EQ(load().observation_origin(), MotionObservationOrigin::Episode);
  EXPECT_EQ(motion_config.observation_origin(), MotionObservationOrigin::Motion);
  reference["observation_origin"] = "invalid";
  EXPECT_THROW(load(), std::runtime_error);
  reference["observation_origin"] = YAML::Load("[motion]");
  EXPECT_THROW(load(), std::runtime_error);
  reference.remove("observation_origin");
  EXPECT_EQ(load().observation_origin(), MotionObservationOrigin::Episode);
  config.remove("commands");
  EXPECT_EQ(load().observation_origin(), MotionObservationOrigin::Episode);
}

TEST_F(GlobalPositionTest, ImuAttitudeIgnoresEstimatorTiltAcrossRepeatedEntries)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  SharedControlData shared;
  shared.resize(1, 1);
  shared.policy.uses_global_position = true;
  PolicyJointContext joints{{"waist_yaw_joint"}, {0}};
  ObservationContext context{shared, joints, &motion};
  const auto & registry = ObservationRegistry::get_registry();
  const auto imu_tilt = Eigen::Quaternionf(Eigen::AngleAxisf(.12f, Eigen::Vector3f::UnitX()));
  std::vector<float> first;
  for (int entry = 0; entry < 5; ++entry) {
    shared.policy.motion_frame.reset();
    shared.localization.position = Eigen::Vector2f(10 + entry, -2 * entry);
    shared.localization.orientation = Eigen::AngleAxisf(.3f * entry, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(.2f * entry, Eigen::Vector3f::UnitY());
    shared.sensors.orientation = Eigen::AngleAxisf(-.4f * entry, Eigen::Vector3f::UnitZ()) * imu_tilt;
    shared.policy.motion_frame.align(shared.localization.position, shared.localization.orientation,
      motion.root_position().head<2>(), motion.root_quaternion(), MotionObservationOrigin::Motion);
    shared.policy.motion_frame.use_imu_orientation(shared.sensors.orientation,
      motion.root_quaternion(), true);
    auto obs = registry.at("motion_anchor_ori_b")(context, YAML::Node{});
    if (entry == 0) {first = obs;}
    for (size_t i = 0; i < obs.size(); ++i) {EXPECT_NEAR(obs[i], first[i], 1e-6);}
    EXPECT_TRUE(shared.policy.motion_frame.orientation(shared.localization.orientation,
        shared.sensors.orientation).isApprox(imu_tilt, 1e-6));
    EXPECT_TRUE(shared.policy.motion_frame.position(shared.localization.position).isApprox(
        motion.root_position().head<2>(), 1e-6));
  }
  // Switching back to a policy without the option preserves its estimator attitude.
  shared.policy.motion_frame.reset();
  EXPECT_TRUE(shared.policy.motion_frame.orientation(shared.localization.orientation,
      shared.sensors.orientation).isApprox(shared.localization.orientation, 1e-6));
}

TEST(MotionFrameAlignment, ImuAttitudeTracksActualTiltAndHonorsDisabledAlignment)
{
  MotionFrameAlignment frame;
  const Eigen::Quaternionf imu(Eigen::AngleAxisf(.4f, Eigen::Vector3f::UnitZ()));
  const Eigen::Quaternionf ref(Eigen::AngleAxisf(1.2f, Eigen::Vector3f::UnitZ()));
  frame.use_imu_orientation(imu, ref, true);
  const Eigen::Quaternionf moved = imu * Eigen::AngleAxisf(.25f, Eigen::Vector3f::UnitY());
  EXPECT_TRUE(frame.orientation(Eigen::Quaternionf::Identity(), moved).isApprox(
      ref * Eigen::AngleAxisf(.25f, Eigen::Vector3f::UnitY()), 1e-6));
  frame.use_imu_orientation(imu, ref, false);
  EXPECT_TRUE(frame.orientation(ref, moved).isApprox(moved, 1e-6));
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

  // Motion-origin policies preserve absolute clip coordinates in both terms.
  shared.policy.motion_frame.align(shared.localization.position, shared.localization.orientation,
    motion.root_position().head<2>(), motion.root_quaternion(), MotionObservationOrigin::Motion);
  EXPECT_EQ(registry.at("robot_root_position_xy_w")(context, YAML::Node{}),
    (std::vector<float>{3.0f, 4.0f}));
  EXPECT_EQ(registry.at("reference_root_position_xy_w")(context, YAML::Node{}),
    (std::vector<float>{3.0f, 4.0f}));
  shared.localization.position.x() += .25f;
  motion.seek(.04);
  EXPECT_EQ(registry.at("robot_root_position_xy_w")(context, YAML::Node{}),
    (std::vector<float>{3.25f, 4.0f}));
  EXPECT_EQ(registry.at("reference_root_position_xy_w")(context, YAML::Node{}),
    (std::vector<float>{6.0f, 5.0f}));
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
  steering["command_deadband"] = 0.04;
  steering["release_on_zero"] = false;
  steering["release_velocity_threshold"] = 0.002;
  EXPECT_FLOAT_EQ(load().steering()->command_deadband, .04f);
  EXPECT_FALSE(load().steering()->release_on_zero);
  EXPECT_FLOAT_EQ(load().steering()->release_velocity_threshold, .002f);
  steering["command_deadband"] = -1;
  EXPECT_THROW(load(), std::runtime_error);
  steering["command_deadband"] = 0.04;
  steering["release_velocity_threshold"] = 0;
  EXPECT_THROW(load(), std::runtime_error);
  steering["release_velocity_threshold"] = 0.002;
  auto reference = config["commands"]["reference_trajectory"];
  reference["required_runtime_features"] = YAML::Load("[future_reference]");
  EXPECT_THROW(load(), std::runtime_error);
  reference.remove("required_runtime_features");
  reference["motion_command_source"] = "retargeted_joint_position_velocity";
  EXPECT_THROW(load(), std::runtime_error);
  reference.remove("motion_command_source");
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

TEST_F(GlobalPositionTest, DanceReferenceUsesRetargetedJointsAndShiftedTargetOnly)
{
  MotionReference motion(file_.string(), 50, {"waist_yaw_joint"}, true);
  SharedControlData shared;
  shared.resize(23, 23);
  motion.set_joint_targets(Eigen::VectorXf::Constant(1, .4f),
    Eigen::VectorXf::Constant(1, -.7f), Eigen::Vector3f(.01f, -.025f, -.02f));
  shared.policy.motion_steering.offset = Eigen::Vector2f(.2f, .3f);
  shared.policy.motion_frame.align(Eigen::Vector2f(10, 20), Eigen::Quaternionf::Identity(),
    motion.root_position().head<2>(), motion.root_quaternion());
  shared.localization.position = Eigen::Vector2f(10, 20);
  PolicyJointContext joints{{"waist_yaw_joint"}, {0}};
  ObservationContext context{shared, joints, &motion};
  auto & registry = ObservationRegistry::get_registry();
  const auto command = registry.at("motion_command")(context, YAML::Node{});
  ASSERT_EQ(command.size(), 2U);
  EXPECT_FLOAT_EQ(command[0], .4f);
  EXPECT_FLOAT_EQ(command[1], -.7f);
  auto xy = registry.at("reference_root_position_xy_w")(context, YAML::Node{});
  EXPECT_NEAR(xy[0], .21f, 1e-6);
  EXPECT_NEAR(xy[1], .275f, 1e-6);
  EXPECT_EQ(registry.at("robot_root_position_xy_w")(context, YAML::Node{}),
    (std::vector<float>{0, 0}));
  // A following CSV policy must get neither adapted joints nor the previous shift.
  motion.seek(0.0);
  EXPECT_EQ(registry.at("motion_command")(context, YAML::Node{}).size(), 2U);
  xy = registry.at("reference_root_position_xy_w")(context, YAML::Node{});
  EXPECT_NEAR(xy[0], .2f, 1e-6);
  EXPECT_NEAR(xy[1], .3f, 1e-6);
}

YAML::Node dance_reference_config()
{
  return YAML::Load(
        R"(
observation_origin: episode
motion_command_source: retargeted_joint_position_velocity
dance_steps:
  min_stance_duration: 0.18
  support_transfer_duration: 0.06
  support_reach: 0.12
  max_step_lift: 0.06
  weight_shift: 0.025
  moving_crouch: 0.02
  max_step_correction: 0.3
  max_step_yaw: 0.5
  ik_iterations: 8
  ik_joint_correction_limit: 2.0
  ik_max_correction_rate: 8.0
  ik_position_tolerance: 0.008
  ik_orientation_tolerance: 0.08
  gesture_source: reference_trajectory
  scheduling: reach_and_source_motion
  phase_labels_sha256: aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  phase_labels: rhythm_reference.npz
)");
}

TEST(DanceReferenceConfig, ValidatesPlannerLimitsAndInputSemantics)
{
  auto reference = dance_reference_config();
  EXPECT_NO_THROW(DanceReferenceConfig::read(reference, .02));
  reference["dance_steps"]["ik_iterations"] = 0;
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  reference["dance_steps"]["ik_iterations"] = 8;
  reference["dance_steps"]["ik_max_correction_rate"] = -1;
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  reference["dance_steps"]["ik_max_correction_rate"] = 8;
  reference["motion_command_source"] = "csv";
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
}

TEST(DanceReferenceConfig, V4RequiresExactDataAndMonotonicSolver)
{
  auto reference = dance_reference_config();
  reference["required_runtime_features"] = YAML::Load("[dance_motion_reference_v4]");
  auto steps = reference["dance_steps"];
  steps["release_behavior"] = "finish_active_gesture_no_new_steps";
  steps["step_intent_source"] = "deadbanded_request_before_smoothing";
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  steps["training_reference_file"] = "dance_reference_v1.bin";
  steps["training_reference_sha256"] = std::string(64, 'a');
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  steps["ik_monotonic"] = true;
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  steps["ik_damping"] = .0025;
  const auto config = DanceReferenceConfig::read(reference, .02);
  EXPECT_TRUE(config.ik_monotonic);
  EXPECT_TRUE(config.stop_new_steps_on_release);
  reference["required_runtime_features"] = YAML::Load("[dance_motion_reference_v3]");
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
}

TEST(DanceReferenceConfig, V3RequiresExplicitReleaseSemantics)
{
  auto reference = dance_reference_config();
  EXPECT_FALSE(DanceReferenceConfig::read(reference, .02).stop_new_steps_on_release);
  reference["required_runtime_features"] = YAML::Load("[dance_motion_reference_v3]");
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  auto steps = reference["dance_steps"];
  steps["release_behavior"] = "finish_active_gesture_no_new_steps";
  steps["step_intent_source"] = "deadbanded_request_before_smoothing";
  EXPECT_TRUE(DanceReferenceConfig::read(reference, .02).stop_new_steps_on_release);
  steps["step_intent_source"] = "smoothed_velocity";
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  steps["step_intent_source"] = "deadbanded_request_before_smoothing";
  steps["release_behavior"] = "cancel_active_gesture";
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  steps["release_behavior"] = "finish_active_gesture_no_new_steps";
  reference["required_runtime_features"] = YAML::Load("[dance_motion_reference_v2]");
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
}

TEST(DanceReferenceConfig, DeclaredTrainingReferenceCannotSilentlyUseCsvFallback)
{
  auto reference = dance_reference_config();
  reference["dance_steps"]["training_reference_file"] = "";
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  reference["dance_steps"]["training_reference_file"] = "dance_reference_v1.bin";
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  reference["dance_steps"]["training_reference_sha256"] = std::string(64, 'a');
  EXPECT_EQ(DanceReferenceConfig::read(reference, .02).training_reference_file,
    "dance_reference_v1.bin");
  EXPECT_EQ(DanceReferenceConfig::read(reference, .02).training_reference_sha256,
    std::string(64, 'a'));
  for (const auto & invalid : {std::string(63, 'a'), std::string(64, 'g')}) {
    reference["dance_steps"]["training_reference_sha256"] = invalid;
    EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
  }
  reference["dance_steps"]["training_reference_sha256"] = std::string(64, 'a');
  reference["dance_steps"].remove("training_reference_file");
  EXPECT_THROW(DanceReferenceConfig::read(reference, .02), std::runtime_error);
}

TEST_F(GlobalPositionTest, TrainingExportDoesNotRequirePhaseLabelFiles)
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
)");
  auto reference = dance_reference_config();
  reference["required_runtime_features"] = YAML::Load("[dance_motion_reference_v2]");
  reference["steering"] =
    YAML::Load(
        R"(
lin_vel_x: [-0.3, 0.3]
lin_vel_y: [-0.3, 0.3]
yaw_rate: [-0.3, 0.3]
smoothing_time_constant: 0.5
command_deadband: 0.1
release_on_zero: true
release_velocity_threshold: 0.01
)");
  config["commands"]["reference_trajectory"] = reference;
  const auto config_path = directory_ / "sim2real.yaml";
  const auto load = [&]() {
      std::ofstream(config_path) << YAML::Dump(config);
      return Sim2RealConfig(config_path);
    };
  EXPECT_TRUE(load().dance_reference().has_value());
  reference["dance_steps"].remove("phase_labels");
  reference["dance_steps"].remove("phase_labels_sha256");
  EXPECT_TRUE(load().dance_reference().has_value());

  reference["required_runtime_features"] = YAML::Load("[dance_motion_reference_v3]");
  reference["dance_steps"]["release_behavior"] = "finish_active_gesture_no_new_steps";
  reference["dance_steps"]["step_intent_source"] = "deadbanded_request_before_smoothing";
  EXPECT_TRUE(load().dance_reference()->stop_new_steps_on_release);
  reference["dance_steps"]["training_reference_file"] = "dance_reference_v1.bin";
  reference["dance_steps"]["training_reference_sha256"] = std::string(64, 'a');
  EXPECT_EQ(load().dance_reference()->training_reference_file,
    (directory_ / "dance_reference_v1.bin").string());
  reference["dance_steps"]["training_reference_file"] = "/tmp/explicit_dance_reference.bin";
  EXPECT_EQ(load().dance_reference()->training_reference_file, "/tmp/explicit_dance_reference.bin");
  reference["required_runtime_features"] = YAML::Load("[dance_motion_reference_v99]");
  EXPECT_THROW(load(), std::runtime_error);
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
