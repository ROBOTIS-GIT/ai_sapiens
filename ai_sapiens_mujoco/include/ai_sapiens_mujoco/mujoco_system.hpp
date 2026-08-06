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

#ifndef AI_SAPIENS_MUJOCO__MUJOCO_SYSTEM_HPP_
#define AI_SAPIENS_MUJOCO__MUJOCO_SYSTEM_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ai_sapiens_mujoco/gantry_service_node.hpp"
#include "ai_sapiens_mujoco/mujoco_simulation.hpp"
#include "ai_sapiens_mujoco/mujoco_viewer.hpp"

namespace ai_sapiens_mujoco
{

class MujocoSystem : public hardware_interface::SystemInterface
{
public:
  ~MujocoSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Publish joint, IMU, and Realtime Tick states from the simulation.
  void publish_states();

  /// Stop the gantry service executor thread and drop the node.
  void stop_gantry_services();

  /// Stop the viewer render thread and drop the viewer.
  void stop_viewer();

  std::shared_ptr<MujocoSimulation> sim_;
  std::vector<std::string> joint_names_;
  std::vector<double> rc_defaults_;   // 16 values
  bool has_hat_state_interfaces_{true};
  bool viewer_enabled_{false};
  bool gantry_enabled_{true};
  double hang_height_{0.90};
  // Precomputed full interface names (index-aligned with joint_names_).
  std::vector<std::string> pos_state_, vel_state_, eff_state_;
  std::vector<std::string> pos_cmd_, ff_cmd_, kp_cmd_, kd_cmd_;
  // Gantry ROS services (spun in a dedicated thread while active).
  std::shared_ptr<GantryServiceNode> gantry_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> gantry_executor_;
  std::thread gantry_thread_;
  // Interactive viewer (started in on_activate when viewer_enabled_).
  std::unique_ptr<MujocoViewer> viewer_;
};

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__MUJOCO_SYSTEM_HPP_
