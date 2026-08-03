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
    gantry_target_z_ = data_->mocap_pos[3 * gantry_mocap_ + 2];
  }
  mj_forward(model_, data_);
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
    const double q = data_->qpos[qpos_adr_[i]];
    const double qd = data_->qvel[qvel_adr_[i]];
    data_->ctrl[act_id_[i]] = c.feedforward + c.kp * (c.position - q) - c.kd * qd;
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
  return gantry_body_ >= 0;
}

bool MujocoSimulation::gantry_attached() const
{
  return false;  // Task 5
}

bool MujocoSimulation::gantry_set_target(double /*height_m*/, double /*speed_mps*/)
{
  return false;  // Task 5
}

bool MujocoSimulation::gantry_release()
{
  return false;  // Task 5
}

double MujocoSimulation::gantry_height() const
{
  return 0.0;  // Task 5
}

void MujocoSimulation::update_gantry()  // caller holds mutex_
{
  // no-op until Task 5
}

}  // namespace ai_sapiens_mujoco
