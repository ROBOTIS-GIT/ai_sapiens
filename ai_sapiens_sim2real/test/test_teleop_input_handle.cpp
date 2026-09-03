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

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/sensor_handles/teleop_input_handle.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

using namespace std::chrono_literals;

class TestTeleopInputPlugin : public TeleopInputPluginBase
{
public:
  void configure(const rclcpp::Node::SharedPtr &, const YAML::Node &) override {}

  AxisRanges output_axis_ranges() const override
  {
    return AxisRanges{};
  }

  std::string name() const override
  {
    return "test_teleop_input";
  }

  void inject_velocity(const Eigen::Vector3f & velocity)
  {
    TeleopInputCommand command;
    command.velocity = velocity;
    accept_valid_command(command);
  }
};

class TeleopInputHandleTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("teleop_input_handle_test");
    plugin_ = std::make_shared<TestTeleopInputPlugin>();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TestTeleopInputPlugin> plugin_;
  TeleopInput teleop_;
  ModeRequests requests_;
  AxisRanges active_ranges_;
};

// cppcheck-suppress syntaxError
TEST_F(TeleopInputHandleTest, ZerosStaleVelocityBeforeInputFailsafe)
{
  TeleopInputHandle handle(
    node_, &teleop_, &requests_, &active_ranges_, plugin_, 1.0, 0.02);
  plugin_->inject_velocity(Eigen::Vector3f(0.5F, -0.25F, 0.75F));

  handle.update(node_->now());
  EXPECT_FALSE(teleop_.unavailable.load());
  EXPECT_FALSE(requests_.damping);
  EXPECT_TRUE(teleop_.velocity_commands.isApprox(Eigen::Vector3f(0.5F, -0.25F, 0.75F)));

  std::this_thread::sleep_for(40ms);
  handle.update(node_->now());
  EXPECT_FALSE(teleop_.unavailable.load());
  EXPECT_FALSE(requests_.damping);
  EXPECT_TRUE(teleop_.velocity_commands.isZero());
}

TEST_F(TeleopInputHandleTest, RejectsVelocityTimeoutLongerThanInputTimeout)
{
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, 0.1, 0.2),
    std::runtime_error);
}

TEST_F(TeleopInputHandleTest, RejectsNonPositiveTimeouts)
{
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, 0.0, 0.1),
    std::runtime_error);
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, -1.0, 0.1),
    std::runtime_error);
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, 1.0, 0.0),
    std::runtime_error);
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, 1.0, -0.1),
    std::runtime_error);
}

TEST_F(TeleopInputHandleTest, RejectsNonFiniteTimeouts)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, nan, 0.1),
    std::runtime_error);
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, infinity, 0.1),
    std::runtime_error);
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, 1.0, nan),
    std::runtime_error);
  EXPECT_THROW(
    TeleopInputHandle(
      node_, &teleop_, &requests_, &active_ranges_, plugin_, 1.0, infinity),
    std::runtime_error);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
