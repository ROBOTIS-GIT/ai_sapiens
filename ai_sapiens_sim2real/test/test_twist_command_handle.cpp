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

#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "ai_sapiens_sim2real/sensor_handles/twist_command_handle.hpp"

namespace ai_sapiens_sim2real
{
namespace
{
using namespace std::chrono_literals;

class TwistCommandHandleTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

// cppcheck-suppress syntaxError
TEST_F(TwistCommandHandleTest, InvalidUsedAxesDoNotReplaceOrRefreshCommand)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("twist_command_test", options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ApiInput api;
  AxisRanges ranges;
  TwistCommandHandle handle(node, &api, &ranges, "/test/velocity");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/test/velocity", 10);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::sleep_for(5ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::Twist msg;
  const auto send = [&]() {
      publisher->publish(msg);
      executor.spin_all(100ms);
      handle.update(node->now());
    };

  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  msg.linear.x = nan;
  send();
  EXPECT_EQ(api.velocity_received_at, std::chrono::steady_clock::time_point{});

  msg.linear.x = 2.0;
  msg.linear.z = nan;
  msg.angular.x = inf;
  send();
  const auto accepted_at = api.velocity_received_at;
  ASSERT_NE(accepted_at, std::chrono::steady_clock::time_point{});
  EXPECT_FLOAT_EQ(api.velocity_command_raw.x(), 2.0f);
  EXPECT_FLOAT_EQ(api.velocity_commands.x(), 1.0f);

  msg.linear.x = 0.5;
  for (double * axis : {&msg.linear.x, &msg.linear.y, &msg.angular.z}) {
    for (double invalid : {nan, -inf, inf, std::numeric_limits<double>::max()}) {
      *axis = invalid;
      send();
      EXPECT_EQ(api.velocity_received_at, accepted_at);
      EXPECT_FLOAT_EQ(api.velocity_command_raw.x(), 2.0f);
    }
    *axis = 0.5;
  }

  send();
  EXPECT_GT(api.velocity_received_at, accepted_at);
  EXPECT_FLOAT_EQ(api.velocity_commands.z(), 0.5f);
}
}  // namespace
}  // namespace ai_sapiens_sim2real
