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

#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/plugins/teleop_input/dualsense_teleop_input_plugin.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

constexpr char kConfig[] =
  R"(
topic: /test/joy
deadzone: 0.08
velocity_command:
  axes:
    linear_x: {index: 1, invert: false}
    linear_y: {index: 0, invert: false}
    angular_z: {index: 2, invert: false}
api_mode:
  when:
    button: 5
    pressed: true
input_code:
  buttons:
    - {button: 1, code: 1}
    - {button: 0, code: 2}
    - {button: 3, code: 3}
    - {button: 2, code: 4}
selector_code:
  buttons:
    - {button: 9, code: 200}
    - {button: 10, code: 201}
  axes:
    - {axis: 4, minimum: 0.5, code: 202}
    - {axis: 5, minimum: 0.5, code: 203}
)";

constexpr char kSelectorNavigation[] =
  R"(
previous_button: 13
next_button: 14
initial_code: 200
options:
  - {code: 200, label: MimicSquat}
  - {code: 201, label: MimicDance1}
  - {code: 202, label: MimicDance2}
)";

YAML::Node navigation_config()
{
  auto config = YAML::Load(kConfig);
  config.remove("selector_code");
  config["selector_navigation"] = YAML::Load(kSelectorNavigation);
  config["input_guide"] = YAML::Load(
    R"(
- "D-pad Left/Right: select mimic"
- "Square: run selected mimic"
)");
  return config;
}

class TestableDualSensePlugin : public DualSenseTeleopInputPlugin
{
public:
  void inject(const sensor_msgs::msg::Joy & msg)
  {
    handle_raw_message(msg);
  }
};

class DualSenseTeleopInputPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("dualsense_teleop_input_plugin_test");
    plugin_ = std::make_shared<TestableDualSensePlugin>();
    plugin_->configure(node_, YAML::Load(kConfig));
  }

  sensor_msgs::msg::Joy valid_message() const
  {
    sensor_msgs::msg::Joy msg;
    msg.axes = {0.25f, 0.75f, -0.5f, 0.0f, 0.0f, 0.0f};
    msg.buttons.resize(17, 0);
    return msg;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TestableDualSensePlugin> plugin_;
};

TEST_F(DualSenseTeleopInputPluginTest, MapsAxesButtonsAndAuthority)
{
  auto msg = valid_message();
  msg.buttons[5] = 1;
  msg.buttons[0] = 1;
  msg.buttons[10] = 1;
  plugin_->inject(msg);

  TeleopInputCommand command;
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_TRUE(command.api_mode);
  EXPECT_EQ(command.input_code, 2);
  EXPECT_EQ(command.selector_code, 201);
  EXPECT_NEAR(command.velocity.x(), 0.72826087f, 1e-6f);
  EXPECT_NEAR(command.velocity.y(), 0.18478261f, 1e-6f);
  EXPECT_NEAR(command.velocity.z(), -0.45652174f, 1e-6f);
  EXPECT_EQ(plugin_->topic_name(), "/test/joy");
}

TEST_F(DualSenseTeleopInputPluginTest, MapsTriggerAxesToSelectors)
{
  auto msg = valid_message();
  msg.axes[4] = 0.75f;
  plugin_->inject(msg);

  TeleopInputCommand command;
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_EQ(command.selector_code, 202);
}

TEST_F(DualSenseTeleopInputPluginTest, CyclesAndLatchesSelectorWithDpadEdges)
{
  auto plugin = std::make_shared<TestableDualSensePlugin>();
  plugin->configure(node_, navigation_config());

  auto msg = valid_message();
  plugin->inject(msg);

  TeleopInputCommand command;
  ASSERT_TRUE(plugin->read_latest_accepted_command(command));
  EXPECT_EQ(command.selector_code, 200);

  msg.buttons[14] = 1;  // D-pad Right
  plugin->inject(msg);
  ASSERT_TRUE(plugin->read_latest_accepted_command(command));
  EXPECT_EQ(command.selector_code, 201);

  // A held button does not repeat.
  plugin->inject(msg);
  ASSERT_TRUE(plugin->read_latest_accepted_command(command));
  EXPECT_EQ(command.selector_code, 201);

  msg.buttons[14] = 0;
  plugin->inject(msg);
  msg.buttons[14] = 1;
  plugin->inject(msg);
  ASSERT_TRUE(plugin->read_latest_accepted_command(command));
  EXPECT_EQ(command.selector_code, 202);

  msg.buttons[14] = 0;
  plugin->inject(msg);
  msg.buttons[14] = 1;
  plugin->inject(msg);
  ASSERT_TRUE(plugin->read_latest_accepted_command(command));
  EXPECT_EQ(command.selector_code, 200);

  msg.buttons[14] = 0;
  plugin->inject(msg);
  msg.buttons[13] = 1;  // D-pad Left
  plugin->inject(msg);
  ASSERT_TRUE(plugin->read_latest_accepted_command(command));
  EXPECT_EQ(command.selector_code, 202);
}

TEST_F(DualSenseTeleopInputPluginTest, RejectsAmbiguousSelectorNavigation)
{
  auto config = navigation_config();
  config["selector_navigation"]["next_button"] = 13;
  DualSenseTeleopInputPlugin plugin;
  EXPECT_THROW(plugin.configure(node_, config), std::runtime_error);

  config = navigation_config();
  config["selector_code"] = YAML::Load(
    R"(
buttons:
  - {button: 9, code: 200}
)");
  EXPECT_THROW(plugin.configure(node_, config), std::runtime_error);
}

TEST_F(DualSenseTeleopInputPluginTest, AppliesDeadzoneAndDefaultsButtonsToInactive)
{
  auto msg = valid_message();
  msg.axes = {0.02f, -0.08f, 0.5f, 0.0f, 0.0f, 0.0f};
  plugin_->inject(msg);

  TeleopInputCommand command;
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_FALSE(command.api_mode);
  EXPECT_EQ(command.input_code, 0);
  EXPECT_EQ(command.selector_code, 0);
  EXPECT_FLOAT_EQ(command.velocity.x(), 0.0f);
  EXPECT_FLOAT_EQ(command.velocity.y(), 0.0f);
  EXPECT_NEAR(command.velocity.z(), 0.45652174f, 1e-6f);
}

TEST_F(DualSenseTeleopInputPluginTest, RenormalizesDeadzoneAndClampsToUnitRange)
{
  auto msg = valid_message();
  msg.axes = {-2.0f, 2.0f, 0.54f, 0.0f, 0.0f, 0.0f};
  plugin_->inject(msg);

  TeleopInputCommand command;
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_FLOAT_EQ(command.velocity.x(), 1.0f);
  EXPECT_FLOAT_EQ(command.velocity.y(), -1.0f);
  EXPECT_NEAR(command.velocity.z(), 0.5f, 1e-6f);
}

TEST_F(DualSenseTeleopInputPluginTest, RejectsIncompleteAndNonFiniteMessages)
{
  auto msg = valid_message();
  msg.axes.resize(2);
  plugin_->inject(msg);
  EXPECT_FALSE(plugin_->is_ready());

  msg = valid_message();
  msg.buttons.resize(10);
  plugin_->inject(msg);
  EXPECT_FALSE(plugin_->is_ready());

  msg = valid_message();
  msg.axes[1] = std::numeric_limits<float>::quiet_NaN();
  plugin_->inject(msg);
  EXPECT_FALSE(plugin_->is_ready());
}

TEST_F(DualSenseTeleopInputPluginTest, RejectsInvalidDeadzone)
{
  DualSenseTeleopInputPlugin plugin;
  auto config = YAML::Load(kConfig);
  config["deadzone"] = 1.0;
  EXPECT_THROW(plugin.configure(node_, config), std::runtime_error);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
