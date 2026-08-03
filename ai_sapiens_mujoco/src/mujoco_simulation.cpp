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

#include "ai_sapiens_mujoco/mujoco_simulation.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace ai_sapiens_mujoco
{

MujocoSimulation::~MujocoSimulation()
{
  if (data_) {
    mj_deleteData(data_);
  }
  if (model_) {
    mj_deleteModel(model_);
  }
}

void MujocoSimulation::load(
  const std::string & scene_path, const std::vector<std::string> & joint_names)
{
  char error[1024] = {0};
  model_ = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    throw std::runtime_error("MuJoCo failed to load '" + scene_path + "': " + error);
  }
  data_ = mj_makeData(model_);
  for (const auto & name : joint_names) {
    const int jid = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    if (jid < 0) {
      throw std::runtime_error("Joint '" + name + "' not found in MuJoCo model");
    }
    const int aid = mj_name2id(model_, mjOBJ_ACTUATOR, (name + "_motor").c_str());
    if (aid < 0) {
      throw std::runtime_error("Actuator '" + name + "_motor' not found in MuJoCo model");
    }

    // Represent impedance feedback as an affine actuator instead of baking it
    // into ctrl as an explicit torque. MuJoCo can then integrate the velocity
    // bias implicitly while still applying the actuator force limit to the
    // complete feedforward + PD effort.
    model_->actuator_biastype[aid] = mjBIAS_AFFINE;
    model_->actuator_ctrllimited[aid] = 0;
    mju_zero(model_->actuator_biasprm + aid * mjNBIAS, mjNBIAS);

    qpos_adr_.push_back(model_->jnt_qposadr[jid]);
    qvel_adr_.push_back(model_->jnt_dofadr[jid]);
    act_id_.push_back(aid);
  }
  commands_.resize(joint_names.size());
  const int quat_id = mj_name2id(model_, mjOBJ_SENSOR, "imu_quat");
  const int gyro_id = mj_name2id(model_, mjOBJ_SENSOR, "imu_gyro");
  const int acc_id = mj_name2id(model_, mjOBJ_SENSOR, "imu_acc");
  if (quat_id < 0 || gyro_id < 0 || acc_id < 0) {
    throw std::runtime_error("IMU sensors (imu_quat/imu_gyro/imu_acc) missing from model");
  }
  imu_quat_adr_ = model_->sensor_adr[quat_id];
  imu_gyro_adr_ = model_->sensor_adr[gyro_id];
  imu_acc_adr_ = model_->sensor_adr[acc_id];
  // Gantry lookups are optional (plain scene has none).
  gantry_body_ = mj_name2id(model_, mjOBJ_BODY, "gantry");
  if (gantry_body_ >= 0) {
    gantry_mocap_ = model_->body_mocapid[gantry_body_];
    gantry_eq_ = mj_name2id(model_, mjOBJ_EQUALITY, "gantry_weld");
    if (
      gantry_mocap_ >= 0 && gantry_eq_ >= 0 &&
      model_->eq_type[gantry_eq_] == mjEQ_WELD &&
      model_->eq_objtype[gantry_eq_] == mjOBJ_BODY)
    {
      const int body1 = model_->eq_obj1id[gantry_eq_];
      const int body2 = model_->eq_obj2id[gantry_eq_];
      if (body1 == gantry_body_) {
        gantry_robot_body_ = body2;
      } else if (body2 == gantry_body_) {
        gantry_robot_body_ = body1;
      }
      gantry_target_z_ = data_->mocap_pos[3 * gantry_mocap_ + 2];
      gantry_released_ = data_->eq_active[gantry_eq_] == 0;
    }
  }
  mj_forward(model_, data_);

  if (gantry_robot_body_ >= 0) {
    const mjtNum * gantry_pos = data_->xpos + 3 * gantry_body_;
    const mjtNum * gantry_quat = data_->xquat + 4 * gantry_body_;
    const mjtNum * robot_pos = data_->xpos + 3 * gantry_robot_body_;
    const mjtNum * robot_quat = data_->xquat + 4 * gantry_robot_body_;
    mjtNum inverse_gantry_quat[4];
    mjtNum world_offset[3];
    mju_negQuat(inverse_gantry_quat, gantry_quat);
    mju_sub3(world_offset, robot_pos, gantry_pos);
    mju_rotVecQuat(gantry_to_robot_pos_.data(), world_offset, inverse_gantry_quat);
    mju_mulQuat(gantry_to_robot_quat_.data(), inverse_gantry_quat, robot_quat);
  }
}

void MujocoSimulation::set_hang_height(double pelvis_z)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double dz = pelvis_z - data_->qpos[2];  // freejoint z is qpos[2]
  data_->qpos[2] += dz;
  if (gantry_mocap_ >= 0) {
    data_->mocap_pos[3 * gantry_mocap_ + 2] += dz;
    gantry_target_z_ = data_->mocap_pos[3 * gantry_mocap_ + 2];
  }
  mj_forward(model_, data_);
}

void MujocoSimulation::set_command(std::size_t joint_index, const JointCommand & cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  commands_[joint_index] = cmd;
}

JointState MujocoSimulation::joint_state(std::size_t joint_index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return JointState{
    data_->qpos[qpos_adr_[joint_index]],
    data_->qvel[qvel_adr_[joint_index]],
    data_->actuator_force[act_id_[joint_index]]};
}

ImuState MujocoSimulation::imu_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ImuState imu;
  for (int i = 0; i < 4; ++i) {
    imu.quat[i] = data_->sensordata[imu_quat_adr_ + i];  // w, x, y, z
  }
  for (int i = 0; i < 3; ++i) {
    imu.gyro[i] = data_->sensordata[imu_gyro_adr_ + i];
    imu.accel[i] = data_->sensordata[imu_acc_adr_ + i];
  }
  return imu;
}

void MujocoSimulation::apply_control()
{
  for (std::size_t i = 0; i < commands_.size(); ++i) {
    const auto & c = commands_[i];
    const int actuator_id = act_id_[i];
    mjtNum * bias = model_->actuator_biasprm + actuator_id * mjNBIAS;
    bias[1] = -c.kp;
    bias[2] = -c.kd;

    // With fixed gain 1 and affine bias, actuator force is:
    // ctrl - kp*q - kd*qd = feedforward + kp*(position-q) - kd*qd.
    data_->ctrl[actuator_id] = c.feedforward + c.kp * c.position;
  }
}

void MujocoSimulation::advance(double dt_seconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  accumulator_ += dt_seconds;
  const double h = model_->opt.timestep;
  while (accumulator_ >= h) {
    apply_control();
    update_gantry();
    mj_step(model_, data_);
    accumulator_ -= h;
  }
}

double MujocoSimulation::sim_time() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return data_ ? data_->time : 0.0;
}

std::mutex & MujocoSimulation::mutex()
{
  return mutex_;
}

const mjModel * MujocoSimulation::model() const
{
  return model_;
}

mjData * MujocoSimulation::data()
{
  return data_;
}

bool MujocoSimulation::gantry_present() const
{
  return
    gantry_body_ >= 0 && gantry_mocap_ >= 0 && gantry_eq_ >= 0 &&
    gantry_robot_body_ >= 0;
}

bool MujocoSimulation::gantry_attached() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return
    gantry_eq_ >= 0 && data_->eq_active[gantry_eq_] != 0 &&
    !gantry_released_;
}

bool MujocoSimulation::gantry_set_target(double height_m, double speed_mps)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    gantry_mocap_ < 0 || gantry_eq_ < 0 ||
    data_->eq_active[gantry_eq_] == 0 || gantry_released_)
  {
    return false;
  }
  gantry_target_z_ = height_m;
  gantry_speed_ = speed_mps > 0.0 ? speed_mps : 0.05;
  return true;
}

bool MujocoSimulation::gantry_attach()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    gantry_mocap_ < 0 || gantry_eq_ < 0 || gantry_robot_body_ < 0 ||
    data_->eq_active[gantry_eq_] != 0 || !gantry_released_)
  {
    return false;
  }

  const mjtNum * robot_pos = data_->xpos + 3 * gantry_robot_body_;
  const mjtNum * robot_quat = data_->xquat + 4 * gantry_robot_body_;
  mjtNum inverse_relative_quat[4];
  mjtNum gantry_quat[4];
  mjtNum world_offset[3];
  mju_negQuat(inverse_relative_quat, gantry_to_robot_quat_.data());
  mju_mulQuat(gantry_quat, robot_quat, inverse_relative_quat);
  mju_rotVecQuat(world_offset, gantry_to_robot_pos_.data(), gantry_quat);

  mjtNum * mocap_pos = data_->mocap_pos + 3 * gantry_mocap_;
  mjtNum * mocap_quat = data_->mocap_quat + 4 * gantry_mocap_;
  mju_sub3(mocap_pos, robot_pos, world_offset);
  mju_copy4(mocap_quat, gantry_quat);
  gantry_target_z_ = mocap_pos[2];
  gantry_speed_ = 0.0;
  data_->eq_active[gantry_eq_] = 1;
  gantry_released_ = false;
  mj_forward(model_, data_);
  return true;
}

bool MujocoSimulation::gantry_release()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    gantry_eq_ < 0 || gantry_mocap_ < 0 ||
    data_->eq_active[gantry_eq_] == 0 || gantry_released_)
  {
    return false;
  }
  data_->eq_active[gantry_eq_] = 0;
  gantry_released_ = true;
  gantry_speed_ = 0.0;
  data_->mocap_pos[3 * gantry_mocap_ + 2] += 1.0;  // park the visual out of the way
  gantry_target_z_ = data_->mocap_pos[3 * gantry_mocap_ + 2];
  mj_forward(model_, data_);
  return true;
}

double MujocoSimulation::gantry_height() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return gantry_mocap_ >= 0 ? data_->mocap_pos[3 * gantry_mocap_ + 2] : 0.0;
}

void MujocoSimulation::update_gantry()  // caller holds mutex_
{
  if (
    gantry_mocap_ < 0 || gantry_eq_ < 0 ||
    data_->eq_active[gantry_eq_] == 0 || gantry_released_ || gantry_speed_ <= 0.0)
  {
    return;
  }
  double & z = data_->mocap_pos[3 * gantry_mocap_ + 2];
  const double max_dz = gantry_speed_ * model_->opt.timestep;
  const double err = gantry_target_z_ - z;
  z += std::clamp(err, -max_dz, max_dz);
}

}  // namespace ai_sapiens_mujoco
