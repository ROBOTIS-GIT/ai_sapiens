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
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
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
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ || data_) {
    throw std::logic_error("MuJoCo scene is already loaded");
  }

  char error[1024] = {0};
  using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
  using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
  ModelPtr loaded_model(
    mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!loaded_model) {
    throw std::runtime_error("MuJoCo failed to load '" + scene_path + "': " + error);
  }

  DataPtr loaded_data(mj_makeData(loaded_model.get()), &mj_deleteData);
  if (!loaded_data) {
    throw std::runtime_error("MuJoCo failed to allocate simulation data");
  }

  std::vector<int> qpos_addresses;
  std::vector<int> qvel_addresses;
  std::vector<int> actuator_ids;
  qpos_addresses.reserve(joint_names.size());
  qvel_addresses.reserve(joint_names.size());
  actuator_ids.reserve(joint_names.size());

  for (const auto & name : joint_names) {
    const int jid = mj_name2id(loaded_model.get(), mjOBJ_JOINT, name.c_str());
    if (jid < 0) {
      throw std::runtime_error("Joint '" + name + "' not found in MuJoCo model");
    }
    const int aid =
      mj_name2id(loaded_model.get(), mjOBJ_ACTUATOR, (name + "_motor").c_str());
    if (aid < 0) {
      throw std::runtime_error("Actuator '" + name + "_motor' not found in MuJoCo model");
    }

    // Represent impedance feedback as an affine actuator instead of baking it
    // into ctrl as an explicit torque. MuJoCo can then integrate the velocity
    // bias implicitly while still applying the actuator force limit to the
    // complete feedforward + PD effort.
    loaded_model->actuator_biastype[aid] = mjBIAS_AFFINE;
    loaded_model->actuator_ctrllimited[aid] = 0;
    mju_zero(loaded_model->actuator_biasprm + aid * mjNBIAS, mjNBIAS);

    qpos_addresses.push_back(loaded_model->jnt_qposadr[jid]);
    qvel_addresses.push_back(loaded_model->jnt_dofadr[jid]);
    actuator_ids.push_back(aid);
  }

  const int quat_id = mj_name2id(loaded_model.get(), mjOBJ_SENSOR, "imu_quat");
  const int gyro_id = mj_name2id(loaded_model.get(), mjOBJ_SENSOR, "imu_gyro");
  const int acc_id = mj_name2id(loaded_model.get(), mjOBJ_SENSOR, "imu_acc");
  if (quat_id < 0 || gyro_id < 0 || acc_id < 0) {
    throw std::runtime_error("IMU sensors (imu_quat/imu_gyro/imu_acc) missing from model");
  }

  model_ = loaded_model.release();
  data_ = loaded_data.release();
  qpos_adr_ = std::move(qpos_addresses);
  qvel_adr_ = std::move(qvel_addresses);
  act_id_ = std::move(actuator_ids);
  commands_.assign(joint_names.size(), JointCommand{});
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
      for (int body = gantry_robot_body_; body > 0; body = model_->body_parentid[body]) {
        const int joint_begin = model_->body_jntadr[body];
        const int joint_end = joint_begin + model_->body_jntnum[body];
        for (int joint = joint_begin; joint < joint_end; ++joint) {
          if (model_->jnt_type[joint] == mjJNT_FREE) {
            robot_free_qpos_adr_ = model_->jnt_qposadr[joint];
            break;
          }
        }
        if (robot_free_qpos_adr_ >= 0) {
          break;
        }
      }
      gantry_target_z_ = data_->mocap_pos[3 * gantry_mocap_ + 2];
      gantry_released_ = data_->eq_active[gantry_eq_] == 0;
      mju_copy4(
        gantry_upright_quat_.data(),
        data_->mocap_quat + 4 * gantry_mocap_);
    }
  }
  mj_forward(model_, data_);

  if (gantry_robot_body_ >= 0) {
    const mjtNum * gantry_pos = data_->xpos + 3 * gantry_body_;
    const mjtNum * gantry_quat = data_->xquat + 4 * gantry_body_;
    mjtNum inverse_gantry_quat[4];
    mjtNum world_offset[3];
    mju_negQuat(inverse_gantry_quat, gantry_quat);

    if (robot_free_qpos_adr_ >= 0) {
      const mjtNum * base_pos = data_->qpos + robot_free_qpos_adr_;
      const mjtNum * base_quat = base_pos + 3;
      mju_sub3(world_offset, base_pos, gantry_pos);
      mju_rotVecQuat(gantry_to_base_pos_.data(), world_offset, inverse_gantry_quat);
      mju_mulQuat(gantry_to_base_quat_.data(), inverse_gantry_quat, base_quat);
      mju_copy3(
        gantry_attach_pos_.data(),
        data_->mocap_pos + 3 * gantry_mocap_);
    }
  }
}

void MujocoSimulation::set_hang_height(double pelvis_z)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!model_ || !data_) {
    throw std::logic_error("MuJoCo scene is not loaded");
  }
  if (!std::isfinite(pelvis_z)) {
    throw std::invalid_argument("Hang height must be finite");
  }
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
    !std::isfinite(height_m) || !std::isfinite(speed_mps) ||
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
    robot_free_qpos_adr_ < 0 ||
    data_->eq_active[gantry_eq_] != 0 || !gantry_released_)
  {
    return false;
  }

  mjtNum world_offset[3];

  mjtNum * mocap_pos = data_->mocap_pos + 3 * gantry_mocap_;
  mjtNum * mocap_quat = data_->mocap_quat + 4 * gantry_mocap_;
  mju_copy3(mocap_pos, gantry_attach_pos_.data());
  mju_copy4(mocap_quat, gantry_upright_quat_.data());

  // Return the floating base to its canonical upright hanging transform under
  // the last active gantry position. Joint positions are intentionally kept.
  mjtNum * base_pos = data_->qpos + robot_free_qpos_adr_;
  mjtNum * base_quat = base_pos + 3;
  mju_rotVecQuat(world_offset, gantry_to_base_pos_.data(), mocap_quat);
  mju_add3(base_pos, mocap_pos, world_offset);
  mju_mulQuat(base_quat, mocap_quat, gantry_to_base_quat_.data());
  mju_normalize4(base_quat);

  // A reattached robot should hang at rest instead of carrying fall/contact
  // momentum into the weld.
  mju_zero(data_->qvel, model_->nv);
  gantry_target_z_ = mocap_pos[2];
  gantry_speed_ = 0.0;

  // Refresh body poses while the weld is inactive, then choose the restored
  // robot origin as a common world anchor for both sides of the weld.
  mj_forward(model_, data_);

  const int body1 = model_->eq_obj1id[gantry_eq_];
  const int body2 = model_->eq_obj2id[gantry_eq_];
  const mjtNum * body1_pos = data_->xpos + 3 * body1;
  const mjtNum * body1_quat = data_->xquat + 4 * body1;
  const mjtNum * body2_pos = data_->xpos + 3 * body2;
  const mjtNum * body2_quat = data_->xquat + 4 * body2;
  const mjtNum * anchor_world = data_->xpos + 3 * gantry_robot_body_;

  mjtNum * weld_data = model_->eq_data + gantry_eq_ * mjNEQDATA;
  mjtNum inverse_body_quat[4];
  mjtNum relative_quat[4];

  // Weld layout: body2 anchor [0:3], body1 anchor [3:6],
  // body2 pose relative to body1 [6:10], torquescale [10].
  mju_sub3(world_offset, anchor_world, body2_pos);
  mju_negQuat(inverse_body_quat, body2_quat);
  mju_rotVecQuat(weld_data, world_offset, inverse_body_quat);
  mju_sub3(world_offset, anchor_world, body1_pos);
  mju_negQuat(inverse_body_quat, body1_quat);
  mju_rotVecQuat(weld_data + 3, world_offset, inverse_body_quat);
  mju_mulQuat(relative_quat, inverse_body_quat, body2_quat);
  mju_copy4(weld_data + 6, relative_quat);

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
  mju_copy3(
    gantry_attach_pos_.data(),
    data_->mocap_pos + 3 * gantry_mocap_);
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
