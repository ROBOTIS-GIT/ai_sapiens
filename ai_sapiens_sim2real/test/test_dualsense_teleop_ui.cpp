// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Kiwoong Park

#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/dualsense_teleop/dualsense_teleop_ui.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

constexpr char kConfig[] =
  R"(
input_code:
  buttons:
    - {button: 1, code: 1, label: Damping}
    - {button: 0, code: 2, label: ReadyPose}
    - {button: 3, code: 3, label: Velocity}
    - {button: 2, code: 4, label: Mimic}
selector_navigation:
  options:
    - {code: 200, label: Squat}
    - {code: 201, label: Dance1}
    - {code: 202, label: Dance2}
ui:
  update_rate: 12.0
  stale_timeout: 0.4
  mode_status_topic: /test/mode_status
)";

TEST(DualSenseTeleopUiConfig, ParsesDashboardConfiguration)
{
  const auto config = DualSenseTeleopUiConfig::from_yaml(YAML::Load(kConfig));

  EXPECT_DOUBLE_EQ(config.update_rate, 12.0);
  EXPECT_DOUBLE_EQ(config.stale_timeout, 0.4);
  EXPECT_EQ(config.mode_status_topic, "/test/mode_status");
  EXPECT_EQ(config.input_labels.at(2), "ReadyPose");
  ASSERT_EQ(config.selector_options.size(), 3U);
  EXPECT_EQ(config.selector_options[1].label, "Dance1");
}

TEST(DualSenseTeleopUi, RendersLiveStateAndCorrectAxisDirections)
{
  DualSenseTeleopUi ui(
    DualSenseTeleopUiConfig::from_yaml(YAML::Load(kConfig)));
  DualSenseTeleopUiState state;
  state.joy_received = true;
  state.joy_fresh = true;
  state.mode_status_received = true;
  state.mode_status_fresh = true;
  state.teleop_input_valid = true;
  state.active_mode = "Velocity";
  state.authority = "Manual";
  state.transition_reason = "teleop request accepted";
  state.command.input_code = 3;
  state.command.selector_code = 201;
  state.command.velocity = Eigen::Vector3f(0.25F, 0.25F, -0.25F);

  const auto dashboard = ui.dashboard(state, false);
  EXPECT_NE(dashboard.find("AI SAPIENS  /  DUALSENSE TELEOP"), std::string::npos);
  EXPECT_NE(dashboard.find("DualSense    CONNECTED"), std::string::npos);
  EXPECT_NE(dashboard.find("Controller   READY"), std::string::npos);
  EXPECT_NE(dashboard.find("Active mode  Velocity"), std::string::npos);
  EXPECT_NE(dashboard.find("Request      Velocity (3)"), std::string::npos);
  EXPECT_NE(dashboard.find("[2/3]  Dance1 (201)"), std::string::npos);
  EXPECT_NE(
    dashboard.find("X    back  [----------|--o-------]  +0.25  forward"),
    std::string::npos);
  EXPECT_NE(
    dashboard.find("Y    left  [--------o-|----------]  +0.25  right"),
    std::string::npos);
  EXPECT_NE(
    dashboard.find("Yaw  left  [----------|--o-------]  -0.25  right"),
    std::string::npos);
  EXPECT_NE(dashboard.find("MODE"), std::string::npos);
  EXPECT_NE(dashboard.find("MIMIC"), std::string::npos);
  EXPECT_NE(dashboard.find("○ Damping"), std::string::npos);
  EXPECT_NE(dashboard.find("× Ready pose"), std::string::npos);
  EXPECT_NE(dashboard.find("△ Velocity"), std::string::npos);
  EXPECT_NE(dashboard.find("□ Mimic"), std::string::npos);
  EXPECT_NE(dashboard.find("◀ ▶ Select motion"), std::string::npos);
  EXPECT_NE(dashboard.find("Left stick Linear X/Y"), std::string::npos);
  EXPECT_NE(dashboard.find("Right stick Yaw"), std::string::npos);
  EXPECT_NE(dashboard.find("SYSTEM  PS API authority"), std::string::npos);
  EXPECT_EQ(dashboard.find("\033["), std::string::npos);

  const auto colored = ui.dashboard(state, true);
  EXPECT_NE(colored.find("\033["), std::string::npos);
}

TEST(DualSenseTeleopUi, DistinguishesWaitingAndStaleInput)
{
  DualSenseTeleopUi ui(
    DualSenseTeleopUiConfig::from_yaml(YAML::Load(kConfig)));
  DualSenseTeleopUiState state;

  auto dashboard = ui.dashboard(state, false);
  EXPECT_NE(dashboard.find("DualSense    WAITING"), std::string::npos);
  EXPECT_NE(dashboard.find("Controller   WAITING"), std::string::npos);

  state.joy_received = true;
  dashboard = ui.dashboard(state, false);
  EXPECT_NE(dashboard.find("DualSense    STALE"), std::string::npos);

  state.mode_status_received = true;
  dashboard = ui.dashboard(state, false);
  EXPECT_NE(dashboard.find("Controller   STALE"), std::string::npos);
}

TEST(DualSenseTeleopUiConfig, RejectsInvalidUiConfiguration)
{
  auto config = YAML::Load(kConfig);
  config["ui"]["update_rate"] = 0.0;
  EXPECT_THROW(
    DualSenseTeleopUiConfig::from_yaml(config), std::runtime_error);

  config = YAML::Load(kConfig);
  config["selector_navigation"]["options"][1]["code"] = 200;
  EXPECT_THROW(
    DualSenseTeleopUiConfig::from_yaml(config), std::runtime_error);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
