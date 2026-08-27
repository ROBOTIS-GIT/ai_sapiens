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

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ai_sapiens_interfaces/srv/set_gantry_height.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
std::string run_xacro()
{
  const std::string cmd =
    "xacro " + ament_index_cpp::get_package_share_directory("ai_sapiens_description") +
    "/urdf/k1_rev1/k1.urdf.xacro sim_mujoco:=true mujoco_viewer:=false mujoco_gantry:=true";
  std::string out;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    throw std::runtime_error("Failed to start xacro command");
  }
  char buf[4096];
  while (fgets(buf, sizeof(buf), pipe)) {out += buf;}
  EXPECT_EQ(pclose(pipe), 0);
  return out;
}
}  // namespace

class MujocoSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(MujocoSystemTest, FullCycleThroughResourceManager)
{
  rclcpp::Node node("test_mujoco_system");
  hardware_interface::ResourceManager rm(
    run_xacro(), node.get_node_clock_interface(), node.get_node_logging_interface(),
    true, 1000);
  ASSERT_TRUE(rm.are_components_initialized());

  // All expected interfaces exist.
  EXPECT_TRUE(rm.state_interface_exists("left_knee_joint/position"));
  EXPECT_TRUE(rm.state_interface_exists("imu/orientation.w"));
  EXPECT_TRUE(rm.state_interface_exists("hat/RC Channel 7"));
  EXPECT_TRUE(rm.command_interface_exists("left_knee_joint/proportional"));

  auto pos_cmd = rm.claim_command_interface("left_knee_joint/position");
  auto kp_cmd = rm.claim_command_interface("left_knee_joint/proportional");
  auto kd_cmd = rm.claim_command_interface("left_knee_joint/derivative");
  auto pos_state = rm.claim_state_interface("left_knee_joint/position");
  auto tick_state = rm.claim_state_interface("hat/Realtime Tick");
  auto ch7_state = rm.claim_state_interface("hat/RC Channel 7");

  ASSERT_TRUE(pos_cmd.set_value(0.8));
  ASSERT_TRUE(kp_cmd.set_value(120.0));
  ASSERT_TRUE(kd_cmd.set_value(3.0));

  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);
  const double tick_before = tick_state.get_optional().value();
  rclcpp::Time t(0, 0);
  for (int i = 0; i < 2000; ++i) {
    t += period;
    rm.read(t, period);
    rm.write(t, period);
  }
  EXPECT_NEAR(pos_state.get_optional().value(), 0.8, 0.05);
  EXPECT_NE(tick_state.get_optional().value(), tick_before);   // watchdog tick advances
  EXPECT_DOUBLE_EQ(ch7_state.get_optional().value(), 2000.0);  // default -> input_code 0
}

TEST_F(MujocoSystemTest, GantryServices)
{
  rclcpp::Node node("test_gantry_services");
  hardware_interface::ResourceManager rm(
    run_xacro(), node.get_node_clock_interface(), node.get_node_logging_interface(),
    true, 1000);  // activates components -> service node spins

  auto client_node = std::make_shared<rclcpp::Node>("gantry_client");
  auto set_h = client_node->create_client<ai_sapiens_interfaces::srv::SetGantryHeight>(
    "/mujoco_sim/gantry/set_height");
  auto attach = client_node->create_client<std_srvs::srv::Trigger>(
    "/mujoco_sim/gantry/attach");
  auto release = client_node->create_client<std_srvs::srv::Trigger>(
    "/mujoco_sim/gantry/release");
  ASSERT_TRUE(set_h->wait_for_service(std::chrono::seconds(5)));
  ASSERT_TRUE(attach->wait_for_service(std::chrono::seconds(5)));

  auto req = std::make_shared<ai_sapiens_interfaces::srv::SetGantryHeight::Request>();
  req->height = 1.45;
  req->speed = 0.2;
  auto fut = set_h->async_send_request(req);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, fut, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(fut.get()->success);

  auto rel_fut = release->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, rel_fut, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(rel_fut.get()->success);

  // After release, set_height must report failure.
  auto fut2 = set_h->async_send_request(req);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, fut2, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(fut2.get()->success);

  auto attach_fut =
    attach->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, attach_fut, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(attach_fut.get()->success);

  auto fut3 = set_h->async_send_request(req);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, fut3, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(fut3.get()->success);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
