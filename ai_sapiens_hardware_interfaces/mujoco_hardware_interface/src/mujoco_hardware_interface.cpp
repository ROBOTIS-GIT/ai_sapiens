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

#include "mujoco_hardware_interface/mujoco_hardware_interface.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace mujoco_hardware_interface
{

using ai_sapiens_mujoco::ImuState;
using ai_sapiens_mujoco::JointCommand;
using ai_sapiens_mujoco::JointState;

namespace
{

std::string get_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & default_value)
{
  const auto it = params.find(key);
  return it != params.end() ? it->second : default_value;
}

bool parse_bool(const std::string & value)
{
  return value == "true" || value == "True" || value == "1";
}

double finite_or_zero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

}  // namespace

MujocoSystem::~MujocoSystem()
{
  // on_deactivate may never run (e.g. hard shutdown); stop the threads here too.
  stop_viewer();
  stop_gantry_services();
}

hardware_interface::CallbackReturn MujocoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hw_params = info_.hardware_parameters;

  const std::string scene_file = get_param(hw_params, "scene_file", "");
  if (scene_file.empty()) {
    RCLCPP_FATAL(get_logger(), "Missing required hardware parameter 'scene_file'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  viewer_enabled_ = parse_bool(get_param(hw_params, "viewer", "false"));
  gantry_enabled_ = parse_bool(get_param(hw_params, "gantry", "true"));

  try {
    hang_height_ = std::stod(get_param(hw_params, "hang_height", "0.90"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Invalid 'hang_height' parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }


  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }
  pos_state_.clear();
  vel_state_.clear();
  eff_state_.clear();
  pos_cmd_.clear();
  ff_cmd_.clear();
  kp_cmd_.clear();
  kd_cmd_.clear();
  for (const auto & name : joint_names_) {
    pos_state_.push_back(name + "/position");
    vel_state_.push_back(name + "/velocity");
    eff_state_.push_back(name + "/effort");
    pos_cmd_.push_back(name + "/position");
    ff_cmd_.push_back(name + "/feedforward");
    kp_cmd_.push_back(name + "/proportional");
    kd_cmd_.push_back(name + "/derivative");
  }

  sim_ = std::make_shared<MujocoSimulation>();
  try {
    sim_->load(scene_file, joint_names_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to load MuJoCo scene: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (gantry_enabled_ && sim_->gantry_present()) {
    sim_->set_hang_height(hang_height_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // State interface handles exist only after export, so initial values are set
  // here rather than in on_init.
  publish_states();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (viewer_enabled_) {
    viewer_ = std::make_unique<MujocoViewer>(sim_);
    viewer_->start();
  }
  if (sim_->gantry_present() && rclcpp::ok()) {
    gantry_node_ = std::make_shared<GantryServiceNode>(sim_);
    gantry_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    gantry_executor_->add_node(gantry_node_);
    gantry_thread_ = std::thread([executor = gantry_executor_] {executor->spin();});
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_viewer();
  stop_gantry_services();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  sim_->advance(period.seconds());
  publish_states();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    JointCommand cmd;
    cmd.position = finite_or_zero(get_command(pos_cmd_[i]));
    cmd.feedforward = finite_or_zero(get_command(ff_cmd_[i]));
    cmd.kp = finite_or_zero(get_command(kp_cmd_[i]));
    cmd.kd = finite_or_zero(get_command(kd_cmd_[i]));
    sim_->set_command(i, cmd);
  }
  return hardware_interface::return_type::OK;
}

void MujocoSystem::publish_states()
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const JointState state = sim_->joint_state(i);
    set_state(pos_state_[i], state.position);
    set_state(vel_state_[i], state.velocity);
    set_state(eff_state_[i], state.effort);
  }

  const ImuState imu = sim_->imu_state();
  set_state("imu/orientation.w", imu.quat[0]);
  set_state("imu/orientation.x", imu.quat[1]);
  set_state("imu/orientation.y", imu.quat[2]);
  set_state("imu/orientation.z", imu.quat[3]);
  set_state("imu/angular_velocity.x", imu.gyro[0]);
  set_state("imu/angular_velocity.y", imu.gyro[1]);
  set_state("imu/angular_velocity.z", imu.gyro[2]);
  set_state("imu/linear_acceleration.x", imu.accel[0]);
  set_state("imu/linear_acceleration.y", imu.accel[1]);
  set_state("imu/linear_acceleration.z", imu.accel[2]);
}

void MujocoSystem::stop_viewer()
{
  if (viewer_) {
    viewer_->stop();
    viewer_.reset();
  }
}

void MujocoSystem::stop_gantry_services()
{
  if (gantry_executor_) {
    gantry_executor_->cancel();
  }
  if (gantry_thread_.joinable()) {
    gantry_thread_.join();
  }
  gantry_executor_.reset();
  gantry_node_.reset();
}

}  // namespace mujoco_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mujoco_hardware_interface::MujocoSystem, hardware_interface::SystemInterface)
