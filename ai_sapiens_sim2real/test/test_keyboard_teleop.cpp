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

#include "ai_sapiens_sim2real/teleop_devtools/keyboard/keyboard_teleop.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

constexpr char kConfig[] =
  R"(
topic: /test/keyboard
publish_rate: 20.0
velocity_step: 0.25
input_code:
  damping: 1
  ready_pose: 2
  velocity: 3
  mimic: 4
selector_navigation:
  initial_code: 200
  options:
    - {code: 200, label: Squat}
    - {code: 201, label: Dance1}
    - {code: 202, label: Dance2}
)";

KeyboardTeleopConfig config()
{
  return KeyboardTeleopConfig::from_yaml(YAML::Load(kConfig));
}

TEST(KeyboardInputDecoder, DecodesCharactersAndFragmentedArrowKeys)
{
  KeyboardInputDecoder decoder;

  const auto characters = decoder.feed("1w SDqepH");
  ASSERT_EQ(characters.size(), 9U);
  EXPECT_EQ(characters[0], KeyboardAction::kDamping);
  EXPECT_EQ(characters[1], KeyboardAction::kForward);
  EXPECT_EQ(characters[2], KeyboardAction::kStop);
  EXPECT_EQ(characters[3], KeyboardAction::kBackward);
  EXPECT_EQ(characters[4], KeyboardAction::kRight);
  EXPECT_EQ(characters[5], KeyboardAction::kYawLeft);
  EXPECT_EQ(characters[6], KeyboardAction::kYawRight);
  EXPECT_EQ(characters[7], KeyboardAction::kToggleApi);
  EXPECT_EQ(characters[8], KeyboardAction::kHelp);

  EXPECT_TRUE(decoder.feed("\x1b[").empty());
  const auto right = decoder.feed("C");
  ASSERT_EQ(right.size(), 1U);
  EXPECT_EQ(right.front(), KeyboardAction::kNextSelector);

  const auto left = decoder.feed("\x1bOD");
  ASSERT_EQ(left.size(), 1U);
  EXPECT_EQ(left.front(), KeyboardAction::kPreviousSelector);
}

TEST(KeyboardTeleopState, StartsSafeAndMapsModeVelocityAndAuthority)
{
  KeyboardTeleopState state(config());

  auto message = state.take_message(1);
  EXPECT_EQ(message.input_code, 1);
  EXPECT_EQ(message.selector_code, 200);
  EXPECT_FALSE(message.api_mode);
  EXPECT_FLOAT_EQ(message.linear_x, 0.0F);

  for (int i = 0; i < 6; ++i) {
    EXPECT_TRUE(state.apply(KeyboardAction::kForward));
  }
  EXPECT_TRUE(state.apply(KeyboardAction::kLeft));
  EXPECT_TRUE(state.apply(KeyboardAction::kYawRight));
  message = state.take_message(2);
  EXPECT_FLOAT_EQ(message.linear_x, 1.0F);
  EXPECT_FLOAT_EQ(message.linear_y, 0.25F);
  EXPECT_FLOAT_EQ(message.angular_z, -0.25F);

  EXPECT_TRUE(state.apply(KeyboardAction::kVelocity));
  message = state.take_message(3);
  EXPECT_EQ(message.input_code, 3);
  EXPECT_FLOAT_EQ(message.linear_x, 0.0F);
  EXPECT_FLOAT_EQ(message.linear_y, 0.0F);
  EXPECT_FLOAT_EQ(message.angular_z, 0.0F);

  EXPECT_TRUE(state.apply(KeyboardAction::kForward));
  EXPECT_TRUE(state.apply(KeyboardAction::kToggleApi));
  message = state.take_message(4);
  EXPECT_TRUE(message.api_mode);
  EXPECT_FLOAT_EQ(message.linear_x, 0.0F);

  EXPECT_TRUE(state.apply(KeyboardAction::kReadyPose));
  message = state.take_message(5);
  EXPECT_FALSE(message.api_mode);
  EXPECT_EQ(message.input_code, 2);
}

TEST(KeyboardTeleopState, WrapsAndLatchesSelector)
{
  KeyboardTeleopState state(config());

  EXPECT_TRUE(state.apply(KeyboardAction::kPreviousSelector));
  EXPECT_EQ(state.take_message(1).selector_code, 202);
  EXPECT_TRUE(state.apply(KeyboardAction::kNextSelector));
  EXPECT_EQ(state.take_message(2).selector_code, 200);
  EXPECT_TRUE(state.apply(KeyboardAction::kNextSelector));
  EXPECT_EQ(state.take_message(3).selector_code, 201);

  EXPECT_TRUE(state.apply(KeyboardAction::kMimic));
  EXPECT_TRUE(state.mimic_request_pending());
  auto message = state.take_message(4);
  EXPECT_EQ(message.selector_code, 201);
  EXPECT_EQ(message.input_code, 4);
  EXPECT_EQ(state.take_message(5).input_code, 4);
  EXPECT_FALSE(state.mimic_request_pending());
  EXPECT_EQ(state.take_message(6).input_code, 0);

  EXPECT_TRUE(state.apply(KeyboardAction::kMimic));
  EXPECT_EQ(state.take_message(7).input_code, 4);
}

TEST(KeyboardTeleopState, RendersReadableDashboard)
{
  KeyboardTeleopState state(config());
  EXPECT_TRUE(state.apply(KeyboardAction::kNextSelector));
  EXPECT_TRUE(state.apply(KeyboardAction::kForward));
  EXPECT_TRUE(state.apply(KeyboardAction::kLeft));
  EXPECT_TRUE(state.apply(KeyboardAction::kYawRight));

  const auto dashboard = state.dashboard("Increase right yaw", false);
  EXPECT_NE(dashboard.find("AI SAPIENS  /  KEYBOARD TELEOP"), std::string::npos);
  EXPECT_NE(dashboard.find("CONTROL STATE"), std::string::npos);
  EXPECT_NE(dashboard.find("MANUAL"), std::string::npos);
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
  EXPECT_NE(dashboard.find("MODE      [1] Damping"), std::string::npos);
  EXPECT_NE(dashboard.find("Last input   Increase right yaw"), std::string::npos);
  EXPECT_EQ(dashboard.find("\033["), std::string::npos);

  const auto colored = state.dashboard("colored", true);
  EXPECT_NE(colored.find("\033["), std::string::npos);
}

TEST(KeyboardTeleopState, DescribesEveryAcceptedAction)
{
  const std::vector<KeyboardAction> actions{
    KeyboardAction::kDamping,
    KeyboardAction::kReadyPose,
    KeyboardAction::kVelocity,
    KeyboardAction::kMimic,
    KeyboardAction::kForward,
    KeyboardAction::kBackward,
    KeyboardAction::kLeft,
    KeyboardAction::kRight,
    KeyboardAction::kYawLeft,
    KeyboardAction::kYawRight,
    KeyboardAction::kStop,
    KeyboardAction::kPreviousSelector,
    KeyboardAction::kNextSelector,
    KeyboardAction::kToggleApi,
    KeyboardAction::kHelp};

  for (const auto action : actions) {
    EXPECT_FALSE(KeyboardTeleopState::action_description(action).empty());
    EXPECT_NE(
      KeyboardTeleopState::action_description(action),
      "Unknown key ignored");
  }
}

TEST(KeyboardTeleopConfig, RejectsUnsafeConfiguration)
{
  auto node = YAML::Load(kConfig);
  node["velocity_step"] = 0.0;
  EXPECT_THROW(KeyboardTeleopConfig::from_yaml(node), std::runtime_error);

  node = YAML::Load(kConfig);
  node["input_code"]["mimic"] = 3;
  EXPECT_THROW(KeyboardTeleopConfig::from_yaml(node), std::runtime_error);

  node = YAML::Load(kConfig);
  node["selector_navigation"]["initial_code"] = 999;
  EXPECT_THROW(KeyboardTeleopConfig::from_yaml(node), std::runtime_error);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
