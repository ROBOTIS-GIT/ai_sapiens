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

#include <cmath>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ai_sapiens_mujoco/mujoco_simulation.hpp"

using ai_sapiens_mujoco::MujocoSimulation;
using ai_sapiens_mujoco::JointCommand;

namespace
{
const std::vector<std::string> kJoints = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_yaw_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"};

std::string scene(const std::string & name)
{
  return ament_index_cpp::get_package_share_directory("ai_sapiens_description") +
         "/mujoco/k1/" + name;
}
}  // namespace

TEST(MujocoSimulation, LoadsSceneAndIndexesJoints)
{
  MujocoSimulation sim;
  ASSERT_NO_THROW(sim.load(scene("scene.xml"), kJoints));
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    EXPECT_TRUE(std::isfinite(sim.joint_state(i).position));
  }
}

TEST(MujocoSimulation, ThrowsOnUnknownJoint)
{
  MujocoSimulation sim;
  EXPECT_THROW(sim.load(scene("scene.xml"), {"no_such_joint"}), std::runtime_error);
}

TEST(MujocoSimulation, ThrowsOnMissingScene)
{
  MujocoSimulation sim;
  EXPECT_THROW(sim.load("/nonexistent/scene.xml", kJoints), std::runtime_error);
}

TEST(MujocoSimulation, AdvanceAccumulatesTime)
{
  MujocoSimulation sim;
  sim.load(scene("scene.xml"), kJoints);
  // CM period 1 ms < model timestep 2 ms: two advances -> one step.
  sim.advance(0.001);
  EXPECT_DOUBLE_EQ(sim.sim_time(), 0.0);
  sim.advance(0.001);
  EXPECT_NEAR(sim.sim_time(), 0.002, 1e-9);
}

TEST(MujocoSimulation, MitImpedanceHoldsJointAtTarget)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);  // hang so legs swing freely
  const std::size_t knee = 3;  // left_knee_joint
  JointCommand cmd;
  cmd.position = 0.8;
  cmd.kp = 120.0;
  cmd.kd = 3.0;
  sim.set_command(knee, cmd);
  for (int i = 0; i < 2000; ++i) {sim.advance(0.001);}  // 2 s
  EXPECT_NEAR(sim.joint_state(knee).position, 0.8, 0.05);
  EXPECT_NE(sim.joint_state(knee).effort, 0.0);
}

TEST(MujocoSimulation, ImuStateIsSane)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);
  for (int i = 0; i < 500; ++i) {sim.advance(0.001);}
  const auto imu = sim.imu_state();
  const double norm = std::sqrt(
    imu.quat[0] * imu.quat[0] + imu.quat[1] * imu.quat[1] +
    imu.quat[2] * imu.quat[2] + imu.quat[3] * imu.quat[3]);
  EXPECT_NEAR(norm, 1.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
