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
// Author: Kiwoong Park, Woojin Wie

#ifndef AI_SAPIENS_SIM2REAL__SIM2REAL_NODE_HPP_
#define AI_SAPIENS_SIM2REAL__SIM2REAL_NODE_HPP_

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/control_loop.hpp"
#include "ai_sapiens_sim2real/controllers/mode_controller.hpp"
#include "ai_sapiens_sim2real/controllers/mode_ros_interface.hpp"
#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/mode_runtime/operator_command_inputs.hpp"
#include "ai_sapiens_sim2real/mode_runtime/startup_gate.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

struct NodeOptions
{
  std::string config_path;
  std::string imu_topic;
  std::string joint_states_topic;
  double imu_timeout{0.5};
  double joint_states_timeout{0.5};
  int thread_priority{50};
  bool lock_memory{true};
  double wait_timeout{30.0};
  double control_rate{1000.0};
  bool status_log_enabled{false};
  bool detailed_status_log{false};
  bool debug_publish_enabled{false};
  double debug_publish_rate{50.0};
  std::string joint_command_topic;
  bool command_publisher_enabled{true};
  bool enforce_position_limits{false};
  std::string api_heartbeat_topic;
  double api_heartbeat_timeout{0.2};
  std::string cmd_vel_topic;
  std::string set_mode_by_name_service;
  std::string list_modes_service;
  std::string mode_status_topic;
  double mode_status_publish_period{0.1};
};

struct RuntimeConfig
{
  OperatorCommandInputOptions operator_command_input_options;
  std::vector<std::string> controller_joints;
};

class Sim2RealNode
{
public:
  Sim2RealNode();

  int run();

private:
  void declare_node_parameters();
  NodeOptions read_node_options() const;
  void log_startup_options() const;
  void configure();

  RuntimeConfig load_runtime_config() const;

  void create_control_loop();
  void initialize_shared_control_data();
  void add_robot_feedback_inputs();
  void add_operator_command_inputs();
  void wait_for_teleop_input();
  void add_mode_controller();
  void add_mode_ros_interface();
  void add_policy_controller();
  void add_joint_command_output();
  void run_control_loop_until_shutdown();

  rclcpp::Node::SharedPtr node_;
  NodeOptions options_;
  std::unique_ptr<RootConfig> root_config_;
  RuntimeConfig runtime_config_;
  std::shared_ptr<ControlLoop> control_loop_;
  SharedControlData * shared_data_{nullptr};
  OperatorCommandInputs operator_command_inputs_;
  std::shared_ptr<ModeController> mode_controller_;
  std::unique_ptr<ModeRosInterface> mode_ros_interface_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SIM2REAL_NODE_HPP_
