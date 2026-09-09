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

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
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
  ASSERT_NE(sim.model(), nullptr);
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    EXPECT_TRUE(std::isfinite(sim.joint_state(i).position));
  }
}

TEST(MujocoSimulation, UsesConfiguredActuatorProperties)
{
  MujocoSimulation sim;
  ASSERT_NO_THROW(sim.load(scene("scene.xml"), kJoints));
  const mjModel * model = sim.model();

  for (const auto & joint_name : kJoints) {
    const bool is_qc060_200_r020_re =
      joint_name.find("shoulder") != std::string::npos ||
      joint_name.find("elbow") != std::string::npos ||
      joint_name.find("wrist") != std::string::npos ||
      joint_name.find("ankle_roll") != std::string::npos;
    const double armature = is_qc060_200_r020_re ? 0.00564892 : 0.01936542;
    const double max_effort = is_qc060_200_r020_re ? 47.277 : 96.864;

    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    const int actuator_id =
      mj_name2id(model, mjOBJ_ACTUATOR, (joint_name + "_motor").c_str());
    ASSERT_GE(joint_id, 0);
    ASSERT_GE(actuator_id, 0);

    const int dof_id = model->jnt_dofadr[joint_id];
    EXPECT_DOUBLE_EQ(model->dof_armature[dof_id], armature);
    EXPECT_DOUBLE_EQ(model->dof_damping[dof_id], 0.0);
    EXPECT_DOUBLE_EQ(model->dof_frictionloss[dof_id], 0.0);
    EXPECT_TRUE(model->jnt_limited[joint_id]);
    EXPECT_TRUE(model->jnt_actfrclimited[joint_id]);
    EXPECT_DOUBLE_EQ(model->jnt_actfrcrange[2 * joint_id], -max_effort);
    EXPECT_DOUBLE_EQ(model->jnt_actfrcrange[2 * joint_id + 1], max_effort);
    EXPECT_TRUE(model->actuator_forcelimited[actuator_id]);
    EXPECT_DOUBLE_EQ(model->actuator_forcerange[2 * actuator_id], -max_effort);
    EXPECT_DOUBLE_EQ(model->actuator_forcerange[2 * actuator_id + 1], max_effort);
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

TEST(MujocoSimulation, FailedLoadLeavesSimulationReusable)
{
  MujocoSimulation sim;
  EXPECT_THROW(sim.load(scene("scene.xml"), {"no_such_joint"}), std::runtime_error);
  EXPECT_NO_THROW(sim.load(scene("scene.xml"), kJoints));
  EXPECT_NE(sim.model(), nullptr);
}

TEST(MujocoSimulation, RejectsLoadingASecondScene)
{
  MujocoSimulation sim;
  sim.load(scene("scene.xml"), kJoints);
  const mjModel * loaded_model = sim.model();

  EXPECT_THROW(sim.load(scene("scene_gantry.xml"), kJoints), std::logic_error);
  EXPECT_EQ(sim.model(), loaded_model);
  EXPECT_TRUE(std::isfinite(sim.joint_state(0).position));
}

TEST(MujocoSimulation, RejectsNonFiniteHangHeight)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);

  EXPECT_THROW(
    sim.set_hang_height(std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    sim.set_hang_height(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
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
  for (int i = 0; i < 2000; ++i) {
    sim.advance(0.001);
  }  // 2 s
  EXPECT_NEAR(sim.joint_state(knee).position, 0.8, 0.05);
  EXPECT_NE(sim.joint_state(knee).effort, 0.0);
}

TEST(MujocoSimulation, ReadyPoseArmGainsRemainStable)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);

  const std::size_t left_wrist = 17;
  const std::size_t right_wrist = 22;
  JointCommand left_command;
  left_command.position = 0.15;
  left_command.kp = 40.0;
  left_command.kd = 10.0;
  JointCommand right_command = left_command;
  right_command.position = -0.15;
  sim.set_command(left_wrist, left_command);
  sim.set_command(right_wrist, right_command);

  double max_abs_wrist_velocity = 0.0;
  for (int step = 0; step < 3000; ++step) {
    sim.advance(0.001);
    max_abs_wrist_velocity = std::max(
      max_abs_wrist_velocity,
      std::max(
        std::abs(sim.joint_state(left_wrist).velocity),
        std::abs(sim.joint_state(right_wrist).velocity)));
  }

  EXPECT_LT(max_abs_wrist_velocity, 5.0);
  EXPECT_NEAR(sim.joint_state(left_wrist).position, left_command.position, 0.02);
  EXPECT_NEAR(sim.joint_state(right_wrist).position, right_command.position, 0.02);
}

TEST(MujocoSimulation, AffineImpedanceRespectsActuatorForceLimit)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);

  const std::size_t left_wrist = 17;
  JointCommand command;
  command.position = 3.0;
  command.feedforward = 100.0;
  command.kp = 1000.0;
  command.kd = 10.0;
  sim.set_command(left_wrist, command);

  sim.advance(0.002);

  EXPECT_LE(std::abs(sim.joint_state(left_wrist).effort), 47.277 + 1e-9);
}

TEST(MujocoSimulation, DampingCommandDoesNotExciteLowInertiaJoints)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);

  JointCommand cmd;
  cmd.kd = 3.0;
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    sim.set_command(i, cmd);
  }

  double max_abs_velocity = 0.0;
  for (int step = 0; step < 2000; ++step) {
    sim.advance(0.001);
    for (std::size_t i = 0; i < kJoints.size(); ++i) {
      max_abs_velocity = std::max(max_abs_velocity, std::abs(sim.joint_state(i).velocity));
    }
  }

  EXPECT_LT(max_abs_velocity, 5.0);
}

TEST(MujocoSimulation, ImuStateIsSane)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);
  for (int i = 0; i < 500; ++i) {
    sim.advance(0.001);
  }
  const auto imu = sim.imu_state();
  const double norm = std::sqrt(
    imu.quat[0] * imu.quat[0] + imu.quat[1] * imu.quat[1] +
    imu.quat[2] * imu.quat[2] + imu.quat[3] * imu.quat[3]);
  EXPECT_NEAR(norm, 1.0, 1e-6);
}

TEST(MujocoSimulationGantry, LowerBringsRobotDown)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);
  ASSERT_TRUE(sim.gantry_present());
  ASSERT_TRUE(sim.gantry_attached());
  const double start_z = sim.data()->qpos[2];
  ASSERT_TRUE(sim.gantry_set_target(sim.gantry_height() - 0.10, 0.2));
  for (int i = 0; i < 1500; ++i) {
    sim.advance(0.001);
  }
  EXPECT_LT(sim.data()->qpos[2], start_z - 0.05);
}

TEST(MujocoSimulationGantry, RejectsNonFiniteTarget)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);
  const double initial_height = sim.gantry_height();

  EXPECT_FALSE(
    sim.gantry_set_target(
      std::numeric_limits<double>::quiet_NaN(), 0.2));
  EXPECT_FALSE(
    sim.gantry_set_target(
      initial_height, std::numeric_limits<double>::infinity()));
  EXPECT_DOUBLE_EQ(sim.gantry_height(), initial_height);
}

TEST(MujocoSimulationGantry, ReleaseAndAttachToggleWeld)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);
  EXPECT_FALSE(sim.gantry_attach());
  const double hanging_z = sim.data()->qpos[2];
  const double hanging_gantry_z = sim.gantry_height();
  ASSERT_TRUE(sim.gantry_release());
  EXPECT_FALSE(sim.gantry_attached());
  // Motion commands fail while released, and the robot falls freely.
  EXPECT_FALSE(sim.gantry_set_target(1.5, 0.2));
  EXPECT_FALSE(sim.gantry_release());
  const double z_before = sim.data()->qpos[2];
  for (int i = 0; i < 500; ++i) {
    sim.advance(0.001);
  }
  EXPECT_LT(sim.data()->qpos[2], z_before);  // free fall: robot drops

  // Reattach restores the robot under the last active gantry pose.
  ASSERT_TRUE(sim.gantry_attach());
  EXPECT_TRUE(sim.gantry_attached());
  EXPECT_NEAR(sim.data()->qpos[2], hanging_z, 1e-12);
  EXPECT_NEAR(sim.gantry_height(), hanging_gantry_z, 1e-12);
  for (int i = 0; i < sim.model()->nv; ++i) {
    EXPECT_NEAR(sim.data()->qvel[i], 0.0, 1e-12);
  }
  EXPECT_FALSE(sim.gantry_attach());
  EXPECT_TRUE(sim.gantry_set_target(sim.gantry_height() + 0.02, 0.2));
  for (int i = 0; i < 20; ++i) {
    sim.advance(0.001);
  }
  EXPECT_LT(std::abs(sim.data()->qpos[2] - hanging_z), 0.05);
  EXPECT_TRUE(sim.gantry_release());
}

TEST(MujocoSimulationGantry, ReattachRestoresUprightHangingPoseForFallenRobot)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(0.90);

  const mjModel * model = sim.model();
  mjData * data = sim.data();
  const int gantry_body = mj_name2id(model, mjOBJ_BODY, "gantry");
  ASSERT_GE(gantry_body, 0);
  const int gantry_mocap = model->body_mocapid[gantry_body];
  ASSERT_GE(gantry_mocap, 0);

  std::array<mjtNum, 4> upright_gantry_quat;
  std::array<mjtNum, 4> upright_robot_quat;
  mju_copy4(upright_gantry_quat.data(), data->mocap_quat + 4 * gantry_mocap);
  mju_copy4(upright_robot_quat.data(), data->qpos + 3);
  const double hanging_z = data->qpos[2];

  ASSERT_TRUE(sim.gantry_release());
  const mjtNum roll_axis[3] = {1.0, 0.0, 0.0};
  mjtNum fallen_robot_quat[4];
  mju_axisAngle2Quat(fallen_robot_quat, roll_axis, 0.5 * M_PI);
  mju_copy4(data->qpos + 3, fallen_robot_quat);
  data->qpos[2] = 0.3;
  mju_fill(data->qvel, 1.0, model->nv);
  mj_forward(model, data);

  ASSERT_TRUE(sim.gantry_attach());
  EXPECT_NEAR(data->qpos[2], hanging_z, 1e-12);
  EXPECT_GT(std::abs(mju_dot(data->qpos + 3, upright_robot_quat.data(), 4)), 0.999999);
  for (int i = 0; i < model->nv; ++i) {
    EXPECT_NEAR(data->qvel[i], 0.0, 1e-12);
  }
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(data->mocap_quat[4 * gantry_mocap + i], upright_gantry_quat[i], 1e-12);
  }

  sim.advance(0.002);
  EXPECT_GT(std::abs(mju_dot(data->qpos + 3, upright_robot_quat.data(), 4)), 0.999);
}

TEST(MujocoSimulationGantry, PlainSceneHasNoGantry)
{
  MujocoSimulation sim;
  sim.load(scene("scene.xml"), kJoints);
  EXPECT_FALSE(sim.gantry_present());
  EXPECT_FALSE(sim.gantry_set_target(1.0, 0.1));
  EXPECT_FALSE(sim.gantry_attach());
  EXPECT_FALSE(sim.gantry_release());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(MujocoSimulation, ResetRestoresHangPoseWeldAndClearsForces)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"), kJoints);
  sim.set_hang_height(1.25);
  const std::vector<mjtNum> initial(sim.data()->qpos, sim.data()->qpos + sim.model()->nq);
  const std::vector<mjtNum> weld(sim.model()->eq_data,
    sim.model()->eq_data + sim.model()->neq * mjNEQDATA);
  ASSERT_TRUE(sim.gantry_release());
  sim.advance(0.02);
  ASSERT_TRUE(sim.gantry_attach());
  sim.data()->xfrc_applied[6] = 100;
  sim.set_command(0, JointCommand{1.0, 100.0, 200.0, 3.0});
  sim.reset();
  EXPECT_DOUBLE_EQ(sim.sim_time(), 0.0);
  EXPECT_TRUE(sim.gantry_attached());
  for (int i=0; i<sim.model()->nq; ++i) EXPECT_DOUBLE_EQ(sim.data()->qpos[i], initial[i]);
  for (int i=0; i<sim.model()->neq * mjNEQDATA; ++i) EXPECT_DOUBLE_EQ(sim.model()->eq_data[i], weld[i]);
  EXPECT_DOUBLE_EQ(sim.data()->xfrc_applied[6], 0);
  EXPECT_DOUBLE_EQ(sim.data()->ctrl[0], 0);
}

TEST(MujocoSimulation, PauseDoesNotAccumulateWallTime)
{
  MujocoSimulation sim;
  sim.load(scene("scene.xml"), kJoints);
  sim.advance(0.01);
  const double before = sim.sim_time();
  sim.set_paused(true);
  sim.advance(20.0);
  EXPECT_DOUBLE_EQ(sim.sim_time(), before);
  sim.set_paused(false);
  sim.advance(0.01);
  EXPECT_NEAR(sim.sim_time(), before + 0.01, 1e-9);
}

TEST(MujocoSimulation, FrictionChangesEffectiveFootContactsAndRestoresDefaults)
{
  MujocoSimulation sim;
  sim.load(scene("scene.xml"), kJoints);
  ASSERT_TRUE(sim.physics_settings().floor_available);
  auto * model = sim.model();
  std::vector<mjtNum> original(model->geom_friction,model->geom_friction+3*model->ngeom);
  sim.set_floor_friction(0.13);
  sim.data()->qpos[2] -= 0.04;
  mj_forward(model,sim.data());
  int floor = mj_name2id(model,mjOBJ_GEOM,"floor");
  int feet_contacts = 0;
  for (int i=0; i<sim.data()->ncon; ++i) {
    const auto & contact = sim.data()->contact[i];
    int other = contact.geom[0] == floor ? contact.geom[1] :
      (contact.geom[1] == floor ? contact.geom[0] : -1);
    if (other < 0) continue;
    const char * name = mj_id2name(model,mjOBJ_BODY,model->geom_bodyid[other]);
    if (!name || std::string(name).find("ankle_roll") == std::string::npos) continue;
    ++feet_contacts;
    EXPECT_NEAR(contact.friction[0],0.13,1e-12);
    EXPECT_NEAR(contact.friction[1],0.13,1e-12);
  }
  EXPECT_GT(feet_contacts,0);
  sim.reset(); EXPECT_DOUBLE_EQ(sim.physics_settings().friction,0.13);
  sim.restore_floor_friction();
  for (int i=0; i<3*model->ngeom; ++i) EXPECT_DOUBLE_EQ(model->geom_friction[i],original[i]);
  EXPECT_THROW(sim.set_floor_friction(std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
}

TEST(MujocoSimulation, PayloadUpdatesMassConstantsWithoutResettingMotion)
{
  MujocoSimulation sim;
  sim.load(scene("scene_gantry.xml"),kJoints);
  sim.set_hang_height(1.2); sim.advance(0.01);
  auto * model = sim.model();
  const int body = mj_name2id(model,mjOBJ_BODY,"pelvis");
  const double mass = model->body_mass[body], total = model->body_subtreemass[0];
  std::array<mjtNum,3> inertia;
  std::copy_n(model->body_inertia+3*body,3,inertia.begin());
  std::vector<mjtNum> qpos(sim.data()->qpos,sim.data()->qpos+model->nq);
  std::vector<mjtNum> qvel(sim.data()->qvel,sim.data()->qvel+model->nv);
  const double time = sim.sim_time();
  sim.set_payload(7.5);
  EXPECT_DOUBLE_EQ(model->body_mass[body],mass+7.5);
  EXPECT_NEAR(model->body_subtreemass[0],total+7.5,1e-9);
  EXPECT_DOUBLE_EQ(sim.sim_time(),time);
  for (int i=0;i<model->nq;++i) EXPECT_DOUBLE_EQ(sim.data()->qpos[i],qpos[i]);
  for (int i=0;i<model->nv;++i) EXPECT_DOUBLE_EQ(sim.data()->qvel[i],qvel[i]);
  for (int i=0;i<3;++i) EXPECT_DOUBLE_EQ(model->body_inertia[3*body+i],inertia[i]);
  sim.reset(); EXPECT_DOUBLE_EQ(model->body_mass[body],mass+7.5);
  sim.restore_payload(); EXPECT_DOUBLE_EQ(model->body_mass[body],mass);
  EXPECT_NEAR(model->body_subtreemass[0],total,1e-9);
  EXPECT_THROW(sim.set_payload(std::numeric_limits<double>::infinity()),std::invalid_argument);
}
