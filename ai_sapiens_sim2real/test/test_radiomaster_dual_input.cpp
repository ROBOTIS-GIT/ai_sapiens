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
#include <rclcpp/rclcpp.hpp>
#include "ai_sapiens_sim2real/mode_runtime/operator_command_inputs.hpp"
#include "ai_sapiens_sim2real/plugins/teleop_input/radiomaster_pocket_teleop_input_plugin.hpp"

namespace ai_sapiens_sim2real
{
class InjectableRadiomaster : public RadiomasterPocketTeleopInputPlugin
{
public:
  using TeleopInputPluginTemplate<ai_sapiens_interfaces::msg::RcStatus>::handle_raw_message;
};
TEST(RadiomasterDualInput, IndependentTicksGroupSwitchAndSelectorMapping)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto node = std::make_shared<rclcpp::Node>("radiomaster_dual_test");
  InjectableRadiomaster individual, group;
  individual.configure(node, YAML::LoadFile(TEST_CONFIG_DIR "/teleop/radiomaster_pocket.yaml"));
  group.configure(node, YAML::LoadFile(TEST_CONFIG_DIR "/teleop/radiomaster_group.yaml"));
  EXPECT_NE(individual.topic_name(), group.topic_name());
  ai_sapiens_interfaces::msg::RcStatus msg;
  msg.status_data_valid = true; msg.is_control_input_safe = true; msg.realtime_tick = 10;
  for (uint8_t id = 1; id <= 16; ++id) {
    ai_sapiens_interfaces::msg::RcChannel channel;
    channel.rc_channel = id; channel.valid = true; channel.rc_us = 1500;
    msg.channels.push_back(channel);
  }
  msg.channels[4].rc_us = 2000; // CH5 group
  msg.channels[6].rc_us = 2000; // CH7 active
  msg.channels[5].rc_us = 2000; // CH6 mimic
  msg.channels[10].rc_us = 1020; // CH11 selector 201
  msg.channels[2].axis = 0.5f;
  individual.handle_raw_message(msg);
  group.handle_raw_message(msg); // same tick accepted by separate stream
  TeleopInputCommand a, b;
  ASSERT_TRUE(individual.read_latest_accepted_command(a));
  ASSERT_TRUE(group.read_latest_accepted_command(b));
  EXPECT_TRUE(a.group_requested); EXPECT_FALSE(b.group_requested);
  EXPECT_EQ(a.input_code, 4); EXPECT_EQ(a.selector_code, 201);
  EXPECT_FLOAT_EQ(a.velocity.x(), 0.5f);
  const auto previous = a.received_at;
  const auto group_previous = b.received_at;
  msg.channels[4].rc_us = 1000;
  individual.handle_raw_message(msg); // repeated tick cannot refresh watchdog
  individual.read_latest_accepted_command(a);
  EXPECT_EQ(a.received_at, previous); EXPECT_TRUE(a.group_requested);
  ++msg.realtime_tick;
  individual.handle_raw_message(msg);
  individual.read_latest_accepted_command(a); group.read_latest_accepted_command(b);
  EXPECT_FALSE(a.group_requested); EXPECT_EQ(b.received_at, group_previous);
  msg.channels[4].valid = false; ++msg.realtime_tick;
  const auto last = a.received_at;
  individual.handle_raw_message(msg); individual.read_latest_accepted_command(a);
  EXPECT_EQ(a.received_at, last); // required group switch missing: no fresh command
}
TEST(RadiomasterDualInput, UnhealthyGroupDoesNotBlockIndividualReadyToWalk)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto node = std::make_shared<rclcpp::Node>("radiomaster_unavailable_test");
  InjectableRadiomaster individual, group;
  individual.configure(node, YAML::LoadFile(TEST_CONFIG_DIR "/teleop/radiomaster_pocket.yaml"));
  group.configure(node, YAML::LoadFile(TEST_CONFIG_DIR "/teleop/radiomaster_group.yaml"));
  ai_sapiens_interfaces::msg::RcStatus msg;
  msg.status_data_valid = true; msg.is_control_input_safe = true; msg.realtime_tick = 10;
  for (uint8_t id = 1; id <= 16; ++id) {
    ai_sapiens_interfaces::msg::RcChannel channel;
    channel.rc_channel = id; channel.valid = true; channel.rc_us = 1500;
    msg.channels.push_back(channel);
  }
  msg.channels[4].rc_us = 2000;
  msg.channels[5].rc_us = 1000;
  individual.handle_raw_message(msg);
  TeleopInputCommand local, common;
  ASSERT_TRUE(individual.read_latest_accepted_command(local));
  EXPECT_EQ(local.input_code, 2);
  auto invalid = msg;
  invalid.is_control_input_safe = false;
  group.handle_raw_message(invalid);
  group.handle_raw_message(invalid);
  EXPECT_FALSE(group.read_latest_accepted_command(common));
  ++msg.realtime_tick;
  msg.channels[6].rc_us = 2000;
  individual.handle_raw_message(msg);
  ASSERT_TRUE(individual.read_latest_accepted_command(local));
  EXPECT_EQ(local.input_code, 3);
  EXPECT_TRUE(local.group_requested);
  // Recovery resets the warning latch only when a fresh command is accepted.
  group.handle_raw_message(msg);
  ASSERT_TRUE(group.read_latest_accepted_command(common));
  const auto accepted_at = common.received_at;
  group.handle_raw_message(invalid);
  group.handle_raw_message(invalid);
  group.read_latest_accepted_command(common);
  EXPECT_EQ(common.received_at, accepted_at);
}
TEST(RadiomasterDualInput, DisabledGroupPreservesLegacyAcceptanceWithoutChannelFive)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto node = std::make_shared<rclcpp::Node>("radiomaster_legacy_channel_test");
  const auto config = YAML::LoadFile(TEST_CONFIG_DIR "/teleop/radiomaster_pocket.yaml");
  InjectableRadiomaster single, dual;
  single.configure(node, individual_input_config(config, false));
  dual.configure(node, individual_input_config(config, true));
  ASSERT_TRUE(config["group_mode"]);  // preparing the single input must not mutate the shared YAML

  ai_sapiens_interfaces::msg::RcStatus msg;
  msg.status_data_valid = true;
  msg.is_control_input_safe = true;
  msg.realtime_tick = 10;
  for (uint8_t id = 1; id <= 16; ++id) {
    if (id == 5) {continue;}
    ai_sapiens_interfaces::msg::RcChannel channel;
    channel.rc_channel = id;
    channel.valid = true;
    channel.rc_us = 1500;
    msg.channels.push_back(channel);
  }
  single.handle_raw_message(msg);
  dual.handle_raw_message(msg);
  TeleopInputCommand command;
  ASSERT_TRUE(single.read_latest_accepted_command(command));
  EXPECT_EQ(command.input_code, 2);  // ReadyPose
  EXPECT_FALSE(command.group_requested);
  EXPECT_FALSE(dual.read_latest_accepted_command(command));

  ai_sapiens_interfaces::msg::RcChannel invalid;
  invalid.rc_channel = 5;
  invalid.valid = false;
  msg.channels.push_back(invalid);
  ++msg.realtime_tick;
  single.handle_raw_message(msg);
  dual.handle_raw_message(msg);
  EXPECT_TRUE(single.read_latest_accepted_command(command));
  EXPECT_FALSE(dual.read_latest_accepted_command(command));
}

}  // namespace ai_sapiens_sim2real
