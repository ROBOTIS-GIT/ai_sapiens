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

#include "ai_sapiens_sim2real/sim2real_node.hpp"

#include <stdexcept>

#include "ai_sapiens_sim2real/command_publishers/joint_command_publisher.hpp"
#include "ai_sapiens_sim2real/controllers/mode_controller.hpp"
#include "ai_sapiens_sim2real/controllers/policy_controller.hpp"
#include "ai_sapiens_sim2real/sensor_handles/imu_sensor_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/joint_states_sensor_handle.hpp"
#include "ai_sapiens_sim2real/config/root_config.hpp"

namespace ai_sapiens_sim2real
{

// sim2real runtime wiring:
//
// ROS feedback topics   -> robot feedback inputs   -> SharedControlData::sensors
// Teleop/API topics     -> operator command inputs -> SharedControlData::teleop/api
// ModeController        -> selects mode, authority, and velocity source
// PolicyController      -> runs the active policy behavior
// JointCommandPublisher -> publishes the final joint command

Sim2RealNode::Sim2RealNode()
: node_(std::make_shared<rclcpp::Node>("ai_sapiens_sim2real_node"))
{
  declare_node_parameters();
}

int Sim2RealNode::run()
{
  try {
    options_ = read_node_options();
    log_startup_options();
    configure();
    run_control_loop_until_shutdown();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error: %s", e.what());
    return 1;
  }

  return 0;
}

void Sim2RealNode::declare_node_parameters()
{
  node_->declare_parameter<std::string>("config_path", "");
  node_->declare_parameter<std::string>("imu_topic", "/imu_sensor_broadcaster/imu");
  node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");
  node_->declare_parameter<double>("imu_timeout", 0.5);
  node_->declare_parameter<double>("joint_states_timeout", 0.5);
  node_->declare_parameter<int>("thread_priority", 50);
  node_->declare_parameter<bool>("lock_memory", true);
  node_->declare_parameter<double>("wait_for_ready_timeout", 30.0);
  node_->declare_parameter<double>("control_rate", 1000.0);
  node_->declare_parameter<bool>("status_log_enabled", false);
  node_->declare_parameter<bool>("detailed_status_log", false);
  node_->declare_parameter<bool>("debug_publish_enabled", false);
  node_->declare_parameter<double>("debug_publish_rate", 50.0);
  node_->declare_parameter<std::string>(
    "joint_command_topic", "/joint_group_impedance_controller/commands");
  node_->declare_parameter<bool>("command_publisher_enabled", true);
  node_->declare_parameter<bool>("enforce_position_limits", false);
  node_->declare_parameter<std::string>(
    "api_heartbeat_topic", "/ai_sapiens/api_heartbeat");
  node_->declare_parameter<double>("api_heartbeat_timeout", 0.2);
  node_->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  node_->declare_parameter<std::string>(
    "set_mode_by_name_service", "/ai_sapiens/set_mode_by_name");
  node_->declare_parameter<std::string>("list_modes_service", "/ai_sapiens/list_modes");
  node_->declare_parameter<std::string>("mode_status_topic", "/ai_sapiens/mode_status");
  node_->declare_parameter<double>("mode_status_publish_period", 0.1);
}

NodeOptions Sim2RealNode::read_node_options() const
{
  NodeOptions options;
  options.config_path = node_->get_parameter("config_path").as_string();
  if (options.config_path.empty()) {
    throw std::runtime_error("config_path parameter is required");
  }

  options.imu_topic = node_->get_parameter("imu_topic").as_string();
  options.joint_states_topic = node_->get_parameter("joint_states_topic").as_string();
  options.imu_timeout = node_->get_parameter("imu_timeout").as_double();
  options.joint_states_timeout = node_->get_parameter("joint_states_timeout").as_double();
  options.thread_priority = node_->get_parameter("thread_priority").as_int();
  options.lock_memory = node_->get_parameter("lock_memory").as_bool();
  options.wait_timeout = node_->get_parameter("wait_for_ready_timeout").as_double();
  options.control_rate = node_->get_parameter("control_rate").as_double();
  options.status_log_enabled = node_->get_parameter("status_log_enabled").as_bool();
  options.detailed_status_log = node_->get_parameter("detailed_status_log").as_bool();
  options.debug_publish_enabled = node_->get_parameter("debug_publish_enabled").as_bool();
  options.debug_publish_rate = node_->get_parameter("debug_publish_rate").as_double();
  options.joint_command_topic = node_->get_parameter("joint_command_topic").as_string();
  options.command_publisher_enabled =
    node_->get_parameter("command_publisher_enabled").as_bool();
  options.enforce_position_limits = node_->get_parameter("enforce_position_limits").as_bool();
  options.api_heartbeat_topic =
    node_->get_parameter("api_heartbeat_topic").as_string();
  options.api_heartbeat_timeout =
    node_->get_parameter("api_heartbeat_timeout").as_double();
  options.cmd_vel_topic = node_->get_parameter("cmd_vel_topic").as_string();
  options.set_mode_by_name_service =
    node_->get_parameter("set_mode_by_name_service").as_string();
  options.list_modes_service = node_->get_parameter("list_modes_service").as_string();
  options.mode_status_topic = node_->get_parameter("mode_status_topic").as_string();
  options.mode_status_publish_period =
    node_->get_parameter("mode_status_publish_period").as_double();
  return options;
}

void Sim2RealNode::log_startup_options() const
{
  RCLCPP_INFO(node_->get_logger(), "=== ai_sapiens_sim2real node ===");
  RCLCPP_INFO(node_->get_logger(), "Config: %s", options_.config_path.c_str());
  RCLCPP_INFO(node_->get_logger(), "IMU topic: %s", options_.imu_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "Joint states topic: %s", options_.joint_states_topic.c_str());
}

void Sim2RealNode::configure()
{
  root_config_ = std::make_unique<RootConfig>(options_.config_path);
  runtime_config_ = load_runtime_config();
  RCLCPP_INFO(node_->get_logger(), "Control publish rate: %.1f Hz", options_.control_rate);

  create_control_loop();
  shared_data_ = control_loop_->get_shared_data();

  initialize_shared_control_data();
  add_robot_feedback_inputs();
  add_operator_command_inputs();
  add_mode_controller();
  add_policy_controller();
  add_joint_command_output();

  RCLCPP_INFO(
    node_->get_logger(),
    "[startup_preflight] configuration, policy assets, and ONNX models are ready");

  wait_for_teleop_input();
  add_mode_ros_interface();
}

RuntimeConfig Sim2RealNode::load_runtime_config() const
{
  RuntimeConfig runtime_config;
  runtime_config.operator_command_input_options = root_config_->operator_command_input_options();
  runtime_config.controller_joints = root_config_->controller_joints();
  return runtime_config;
}

void Sim2RealNode::create_control_loop()
{
  control_loop_ = std::make_shared<ControlLoop>(node_, static_cast<float>(options_.control_rate));
  control_loop_->set_thread_priority(options_.thread_priority);
  control_loop_->set_lock_memory(options_.lock_memory);
  control_loop_->set_wait_for_ready_timeout(options_.wait_timeout);
  control_loop_->set_status_log_enabled(options_.status_log_enabled);
  control_loop_->set_detailed_status_log(options_.detailed_status_log);
  control_loop_->set_debug_publish_rate(options_.debug_publish_rate);
  control_loop_->set_debug_publish_enabled(options_.debug_publish_enabled);
}

void Sim2RealNode::initialize_shared_control_data()
{
  shared_data_->joint_map.controller_joint_names = runtime_config_.controller_joints;
  const size_t num_joints = runtime_config_.controller_joints.size();
  shared_data_->resize(num_joints, num_joints);
  RCLCPP_INFO(node_->get_logger(), "Configured %zu controller joints", num_joints);
}

void Sim2RealNode::add_robot_feedback_inputs()
{
  control_loop_->add_sensor_handle(
    std::make_shared<ImuSensorHandle>(
      node_,
      &shared_data_->sensors,
      &shared_data_->requests,
      options_.imu_timeout,
      options_.imu_topic),
    true);
  control_loop_->add_sensor_handle(
    std::make_shared<JointStatesSensorHandle>(
      node_,
      &shared_data_->sensors,
      &shared_data_->requests,
      options_.joint_states_timeout,
      runtime_config_.controller_joints,
      options_.joint_states_topic),
    true);
}

void Sim2RealNode::add_operator_command_inputs()
{
  auto input_options = runtime_config_.operator_command_input_options;

  input_options.api_heartbeat_topic = options_.api_heartbeat_topic;
  input_options.api_heartbeat_timeout = options_.api_heartbeat_timeout;
  input_options.cmd_vel_topic = options_.cmd_vel_topic;

  operator_command_inputs_ =
    add_operator_command_input_handles(control_loop_, node_, shared_data_, input_options);
}

void Sim2RealNode::wait_for_teleop_input()
{
  if (!wait_for_teleop_input_sample(
      node_, operator_command_inputs_.teleop_input_plugin, options_.wait_timeout))
  {
    throw std::runtime_error("timed out waiting for teleop input plugin sample");
  }
}

void Sim2RealNode::add_mode_controller()
{
  mode_controller_ = std::make_shared<ModeController>(node_, *root_config_, shared_data_);
  control_loop_->set_mode_controller(mode_controller_);
}

void Sim2RealNode::add_mode_ros_interface()
{
  mode_ros_interface_ = std::make_unique<ModeRosInterface>(
    node_,
    *mode_controller_,
    options_.set_mode_by_name_service,
    options_.list_modes_service,
    options_.mode_status_topic,
    options_.mode_status_publish_period);
}

void Sim2RealNode::add_policy_controller()
{
  control_loop_->set_policy_controller(
    std::make_shared<PolicyController>(node_, *root_config_, shared_data_));
}

void Sim2RealNode::add_joint_command_output()
{
  control_loop_->add_command_publisher(
    std::make_shared<JointCommandPublisher>(
      node_,
      &shared_data_->output,
      options_.joint_command_topic,
      runtime_config_.controller_joints,
      options_.enforce_position_limits,
      options_.command_publisher_enabled));
}

void Sim2RealNode::run_control_loop_until_shutdown()
{
  control_loop_->start();

  RCLCPP_INFO(node_->get_logger(), "Control loop started. Spinning...");
  rclcpp::spin(node_);

  control_loop_->stop();

  // Propagate RT-thread aborts as process failures.
  if (control_loop_->faulted()) {
    throw std::runtime_error("control loop aborted; see RT fatal log above");
  }
}

}  // namespace ai_sapiens_sim2real
