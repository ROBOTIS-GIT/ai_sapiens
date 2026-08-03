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

#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/plugins/teleop_input/keyboard_teleop_input_plugin.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

constexpr char kConfig[] =
  R"(
topic: /test/keyboard
publish_rate: 20.0
velocity_step: 0.2
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
)";

class TestableKeyboardPlugin : public KeyboardTeleopInputPlugin
{
public:
  void inject(const ai_sapiens_interfaces::msg::KeyboardInput & message)
  {
    handle_raw_message(message);
  }
};

class KeyboardTeleopInputPluginTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("keyboard_teleop_input_plugin_test");
    plugin_ = std::make_shared<TestableKeyboardPlugin>();
    plugin_->configure(node_, YAML::Load(kConfig));
  }

  ai_sapiens_interfaces::msg::KeyboardInput valid_message(uint32_t sequence) const
  {
    ai_sapiens_interfaces::msg::KeyboardInput message;
    message.sequence = sequence;
    message.input_code = 3;
    message.selector_code = 201;
    message.linear_x = 0.5F;
    message.linear_y = -0.25F;
    message.angular_z = 0.75F;
    return message;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TestableKeyboardPlugin> plugin_;
};

TEST_F(KeyboardTeleopInputPluginTest, MapsValidKeyboardInput)
{
  auto message = valid_message(1);
  message.api_mode = true;
  plugin_->inject(message);

  TeleopInputCommand command;
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_TRUE(command.api_mode);
  EXPECT_EQ(command.input_code, 3);
  EXPECT_EQ(command.selector_code, 201);
  EXPECT_FLOAT_EQ(command.velocity.x(), 0.5F);
  EXPECT_FLOAT_EQ(command.velocity.y(), -0.25F);
  EXPECT_FLOAT_EQ(command.velocity.z(), 0.75F);
  EXPECT_EQ(plugin_->topic_name(), "/test/keyboard");
}

TEST_F(KeyboardTeleopInputPluginTest, AcceptsNeutralInputToRearmEdgeTrigger)
{
  auto message = valid_message(1);
  message.input_code = 0;
  plugin_->inject(message);

  TeleopInputCommand command;
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_EQ(command.input_code, 0);
  EXPECT_EQ(command.selector_code, 201);
}

TEST_F(KeyboardTeleopInputPluginTest, RejectsStaleSequenceAndAcceptsRollover)
{
  plugin_->inject(valid_message(std::numeric_limits<uint32_t>::max()));
  plugin_->inject(valid_message(0));

  TeleopInputCommand command;
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_EQ(command.input_code, 3);

  auto stale = valid_message(0);
  stale.input_code = 1;
  plugin_->inject(stale);
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_EQ(command.input_code, 3);

  auto older = valid_message(std::numeric_limits<uint32_t>::max());
  older.input_code = 1;
  plugin_->inject(older);
  ASSERT_TRUE(plugin_->read_latest_accepted_command(command));
  EXPECT_EQ(command.input_code, 3);
}

TEST_F(KeyboardTeleopInputPluginTest, RejectsUnsafeCommandValues)
{
  auto invalid = valid_message(1);
  invalid.linear_x = 1.1F;
  plugin_->inject(invalid);
  EXPECT_FALSE(plugin_->is_ready());

  invalid = valid_message(2);
  invalid.angular_z = std::numeric_limits<float>::quiet_NaN();
  plugin_->inject(invalid);
  EXPECT_FALSE(plugin_->is_ready());

  invalid = valid_message(3);
  invalid.input_code = 99;
  plugin_->inject(invalid);
  EXPECT_FALSE(plugin_->is_ready());

  invalid = valid_message(4);
  invalid.selector_code = 999;
  plugin_->inject(invalid);
  EXPECT_FALSE(plugin_->is_ready());
}

TEST_F(KeyboardTeleopInputPluginTest, RejectsInvalidConfiguration)
{
  auto config = YAML::Load(kConfig);
  config["topic"] = "";
  KeyboardTeleopInputPlugin plugin;
  EXPECT_THROW(plugin.configure(node_, config), std::runtime_error);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
