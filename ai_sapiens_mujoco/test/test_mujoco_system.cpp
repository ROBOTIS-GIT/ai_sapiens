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

#include <cstdio>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
std::string run_xacro()
{
  const std::string cmd =
    "xacro " + ament_index_cpp::get_package_share_directory("ai_sapiens_description") +
    "/urdf/k1_rev1/k1.urdf.xacro sim_mujoco:=true mujoco_viewer:=false mujoco_gantry:=true";
  std::string out;
  FILE * pipe = popen(cmd.c_str(), "r");
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
