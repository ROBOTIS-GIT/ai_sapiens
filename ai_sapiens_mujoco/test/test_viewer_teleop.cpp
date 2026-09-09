// Copyright 2026 ROBOTIS CO., LTD. Licensed under Apache-2.0.
#include <gtest/gtest.h>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ai_sapiens_mujoco/viewer_teleop.hpp"
using namespace ai_sapiens_mujoco;
using namespace std::chrono_literals;

TEST(ViewerTeleop, CommandsRearmMimicAndResetWaitsForDamping)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  auto sim = std::make_shared<MujocoSimulation>();
  sim->load(ament_index_cpp::get_package_share_directory("ai_sapiens_description") +
    "/mujoco/k1/scene_gantry.xml", {});
  sim->set_hang_height(1.3);
  sim->advance(0.01);
  ViewerTeleop bridge(sim);
  auto node = std::make_shared<rclcpp::Node>("viewer_test_controller");
  auto status = node->create_publisher<ai_sapiens_interfaces::msg::ModeStatus>("/ai_sapiens/mode_status",10);
  std::vector<ai_sapiens_interfaces::msg::KeyboardInput> received;
  auto input = node->create_subscription<ai_sapiens_interfaces::msg::KeyboardInput>(
    "/mujoco_teleop/input",rclcpp::SensorDataQoS(),
    [&](const ai_sapiens_interfaces::msg::KeyboardInput & msg){ received.push_back(msg); });
  auto pump = [&](const char * mode, int ms) {
    auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < until) {
      ai_sapiens_interfaces::msg::ModeStatus msg;
      msg.active_mode=mode; msg.authority="MANUAL";
      status->publish(msg); bridge.tick(); rclcpp::spin_some(node);
      std::this_thread::sleep_for(10ms);
    }
  };
  pump("Damping",500);
  ASSERT_TRUE(bridge.state.connected);
  ASSERT_FALSE(received.empty()); EXPECT_EQ(received.back().input_code,1);
  bridge.request(ViewerUiAction::kReadyPose);pump("ReadyPose",150);
  EXPECT_EQ(received.back().input_code,2);
  bridge.request(ViewerUiAction::kVelocity);pump("Velocity",150);
  bridge.state.velocity[0]=2.0;bridge.state.velocity[1]=-0.4;
  pump("Velocity",100);
  EXPECT_FLOAT_EQ(received.back().linear_x,1.0f);
  EXPECT_FLOAT_EQ(received.back().linear_y,-0.4f);
  for (int attempt=0;attempt<2;++attempt) {
    received.clear(); bridge.request(ViewerUiAction::kMimic);pump("Velocity",450);
    ASSERT_GE(received.size(),6u);
    EXPECT_EQ(received.front().input_code,0);
    bool saw_mimic=false;
    for(const auto & msg:received) if(msg.input_code==4) saw_mimic=true;
    EXPECT_TRUE(saw_mimic); EXPECT_EQ(received.back().input_code,0);
    EXPECT_FLOAT_EQ(received.back().linear_x,0);
  }
  bridge.request(ViewerUiAction::kReset);pump("Velocity",200);
  EXPECT_TRUE(bridge.state.busy); EXPECT_GT(sim->sim_time(),0);
  pump("Damping",200);
  EXPECT_FALSE(bridge.state.busy); EXPECT_DOUBLE_EQ(sim->sim_time(),0);
  EXPECT_DOUBLE_EQ(sim->data()->qpos[2],1.3);
  bridge.request(ViewerUiAction::kPause);pump("Damping",250); EXPECT_TRUE(sim->paused());
  bridge.request(ViewerUiAction::kPause);pump("Damping",100); EXPECT_FALSE(sim->paused());
  std::this_thread::sleep_for(550ms);bridge.tick();
  EXPECT_FALSE(bridge.state.connected);
}
