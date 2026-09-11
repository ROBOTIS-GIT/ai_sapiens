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

}  // namespace
}  // namespace ai_sapiens_sim2real
