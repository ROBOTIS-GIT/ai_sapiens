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
    linear_x: {index: 1, invert: true}
    linear_y: {index: 0, invert: true}
    angular_z: {index: 2, invert: true}
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
  EXPECT_FLOAT_EQ(command.velocity.x(), -0.75f);
  EXPECT_FLOAT_EQ(command.velocity.y(), -0.25f);
  EXPECT_FLOAT_EQ(command.velocity.z(), 0.5f);
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
  EXPECT_FLOAT_EQ(command.velocity.z(), -0.5f);
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
