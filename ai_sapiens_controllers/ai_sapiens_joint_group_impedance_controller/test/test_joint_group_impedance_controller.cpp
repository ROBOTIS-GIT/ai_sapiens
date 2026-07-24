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
// Author: Woojin Wie

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ai_sapiens_interfaces/msg/joint_impedance_command.hpp"
#include \
  "ai_sapiens_joint_group_impedance_controller/ai_sapiens_joint_group_impedance_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace
{

using Controller =
  ai_sapiens_joint_group_impedance_controller::AiSapiensJointGroupImpedanceController;
using CommandMsg = ai_sapiens_interfaces::msg::JointImpedanceCommand;
using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::LoanedCommandInterface;

constexpr char kFeedforwardInterface[] = "feedforward";

class JointGroupImpedanceControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    controller_ = std::make_unique<Controller>();
  }

  void TearDown() override
  {
    if (node_added_) {
      executor_.remove_node(controller_->get_node()->get_node_base_interface());
    }
    publisher_.reset();
    controller_.reset();
  }

  void initialize(const std::vector<std::string> & joints = {})
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_joint_group_impedance_controller";
    params.robot_description = "";
    params.controller_manager_update_rate = 1000;
    params.node_namespace = "";
    params.node_options = controller_->define_custom_node_options();
    params.node_options.parameter_overrides({rclcpp::Parameter("joints", joints)});
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);

    executor_.add_node(controller_->get_node()->get_node_base_interface());
    node_added_ = true;
  }

  void assign_interfaces(bool wrong_last_interface = false)
  {
    command_values_.assign(8, 10.0);
    command_handles_.clear();
    command_handles_.reserve(8);

    command_handles_.emplace_back(
      joint_names_[0], hardware_interface::HW_IF_POSITION, &command_values_[0]);
    command_handles_.emplace_back(
      joint_names_[1], hardware_interface::HW_IF_POSITION, &command_values_[1]);
    command_handles_.emplace_back(joint_names_[0], kFeedforwardInterface, &command_values_[2]);
    command_handles_.emplace_back(joint_names_[1], kFeedforwardInterface, &command_values_[3]);
    command_handles_.emplace_back(
      joint_names_[0], hardware_interface::HW_IF_PROPORTIONAL_GAIN, &command_values_[4]);
    command_handles_.emplace_back(
      joint_names_[0], hardware_interface::HW_IF_DERIVATIVE_GAIN, &command_values_[5]);
    command_handles_.emplace_back(
      joint_names_[1], hardware_interface::HW_IF_PROPORTIONAL_GAIN, &command_values_[6]);
    command_handles_.emplace_back(
      joint_names_[1],
      wrong_last_interface ? "wrong_interface" : hardware_interface::HW_IF_DERIVATIVE_GAIN,
      &command_values_[7]);

    std::vector<LoanedCommandInterface> loaned_interfaces;
    loaned_interfaces.reserve(command_handles_.size());
    for (auto & handle : command_handles_) {
      loaned_interfaces.emplace_back(handle);
    }
    controller_->assign_interfaces(std::move(loaned_interfaces), {});
  }

  void configure_and_activate()
  {
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

    publisher_ = test_node_.create_publisher<CommandMsg>(
      controller_->get_node()->get_fully_qualified_name() + std::string("/commands"),
      rclcpp::SystemDefaultsQoS());
  }

  void publish(const CommandMsg & command)
  {
    publisher_->publish(command);
    const auto timeout = std::chrono::milliseconds(100);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::unique_ptr<Controller> controller_;
  const std::vector<std::string> joint_names_{"joint1", "joint2"};
  std::vector<double> command_values_;
  std::vector<CommandInterface> command_handles_;
  rclcpp::Node test_node_{"impedance_controller_test_publisher"};
  rclcpp::Publisher<CommandMsg>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  bool node_added_{false};
};

TEST_F(JointGroupImpedanceControllerTest, EmptyJointParameterFailsConfiguration)
{
  ASSERT_NO_FATAL_FAILURE(initialize());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(JointGroupImpedanceControllerTest, InterfaceConfigurationHasExpectedOrder)
{
  ASSERT_NO_FATAL_FAILURE(initialize(joint_names_));
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  const auto configuration = controller_->command_interface_configuration();
  EXPECT_EQ(configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_THAT(
    configuration.names,
    ::testing::ElementsAre(
      "joint1/position", "joint2/position", "joint1/feedforward", "joint2/feedforward",
      "joint1/proportional", "joint1/derivative", "joint2/proportional", "joint2/derivative"));
}

TEST_F(JointGroupImpedanceControllerTest, ActivationRejectsWrongInterface)
{
  ASSERT_NO_FATAL_FAILURE(initialize(joint_names_));
  assign_interfaces(true);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(JointGroupImpedanceControllerTest, NoCommandDoesNotModifyHardware)
{
  ASSERT_NO_FATAL_FAILURE(initialize(joint_names_));
  assign_interfaces();
  ASSERT_NO_FATAL_FAILURE(configure_and_activate());

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)),
    controller_interface::return_type::OK);
  EXPECT_THAT(command_values_, ::testing::Each(10.0));
}

TEST_F(JointGroupImpedanceControllerTest, ValidCommandUpdatesEveryInterface)
{
  ASSERT_NO_FATAL_FAILURE(initialize(joint_names_));
  assign_interfaces();
  ASSERT_NO_FATAL_FAILURE(configure_and_activate());

  CommandMsg command;
  command.positions = {1.0, 2.0};
  command.feedforward = {3.0, 4.0};
  command.kp = {5.0, 6.0};
  command.kd = {7.0, 8.0};
  publish(command);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)),
    controller_interface::return_type::OK);
  EXPECT_THAT(command_values_, ::testing::ElementsAre(1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 6.0, 8.0));
}

TEST_F(JointGroupImpedanceControllerTest, InvalidCommandIsDropped)
{
  ASSERT_NO_FATAL_FAILURE(initialize(joint_names_));
  assign_interfaces();
  ASSERT_NO_FATAL_FAILURE(configure_and_activate());

  CommandMsg command;
  command.positions = {1.0, 2.0};
  command.feedforward = {3.0};
  command.kp = {5.0, 6.0};
  command.kd = {7.0, 8.0};
  publish(command);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)),
    controller_interface::return_type::OK);
  EXPECT_THAT(command_values_, ::testing::Each(10.0));
}

TEST_F(JointGroupImpedanceControllerTest, ReactivationRequiresNewCommand)
{
  ASSERT_NO_FATAL_FAILURE(initialize(joint_names_));
  assign_interfaces();
  ASSERT_NO_FATAL_FAILURE(configure_and_activate());

  CommandMsg command;
  command.positions = {1.0, 2.0};
  command.feedforward = {3.0, 4.0};
  command.kp = {5.0, 6.0};
  command.kd = {7.0, 8.0};
  publish(command);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  std::fill(command_values_.begin(), command_values_.end(), 20.0);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)),
    controller_interface::return_type::OK);

  EXPECT_THAT(command_values_, ::testing::Each(20.0));
}

}  // namespace
