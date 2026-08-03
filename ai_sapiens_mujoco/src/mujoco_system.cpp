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

#include "ai_sapiens_mujoco/mujoco_system.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ai_sapiens_mujoco
{

namespace
{

constexpr const char * kDefaultRcChannels =
  "1500,1500,1500,1500,1000,1500,2000,2000,1000,1000,1000,1000,1500,1500,1500,1500";

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

  const std::string rc_defaults_str =
    get_param(hw_params, "rc_channel_defaults", kDefaultRcChannels);
  rc_defaults_.clear();
  std::stringstream rc_stream(rc_defaults_str);
  std::string token;
  while (std::getline(rc_stream, token, ',')) {
    try {
      rc_defaults_.push_back(std::stod(token));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(), "Invalid 'rc_channel_defaults' entry '%s': %s", token.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (rc_defaults_.size() != 16) {
    RCLCPP_FATAL(
      get_logger(), "'rc_channel_defaults' must contain 16 values, got %zu",
      rc_defaults_.size());
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
  for (std::size_t i = 0; i < rc_defaults_.size(); ++i) {
    set_state("hat/RC Channel " + std::to_string(i + 1), rc_defaults_[i]);
  }
  set_state("hat/Hardware Error Status", 0.0);
  set_state("hat/E-stop Active", 0.0);
  set_state("hat/BMS SOC", 100.0);
  set_state("hat/BMS Voltage", 54.0);
  set_state("hat/BMS Current", 2.0);
  set_state("hat/BMS Temperature", 35.0);
  set_state("hat/BMS Service Error Flags", 0.0);
  set_state("hat/BMS Remaining Energy", 500.0);
  set_state("hat/CRSF Failsafe", 0.0);
  set_state("hat/CRSF Link Quality", 100.0);
  set_state("hat/CRSF RSSI 1", 80.0);
  set_state("hat/CRSF Last Frame Age", 1.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Task 7 starts the gantry service node here; Task 8 starts the viewer.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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

  // rc_broadcaster expects the watchdog tick to keep changing and stay in
  // [0, 32767].
  set_state(
    "hat/Realtime Tick", std::fmod(std::floor(sim_->sim_time() * 1000.0), 32768.0));
}

}  // namespace ai_sapiens_mujoco

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ai_sapiens_mujoco::MujocoSystem, hardware_interface::SystemInterface)
