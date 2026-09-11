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

#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include "ai_sapiens_sim2real/sensor_handles/localization_sensor_handle.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

class LocalizationHandleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("localization_handle_test",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    handle_ = std::make_unique<LocalizationSensorHandle>(
      node_, &data_, 0.1, "/test_gloposition/odom");
    pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/test_gloposition/odom", rclcpp::SensorDataQoS());
  }
  nav_msgs::msg::Odometry message()
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "pelvis";
    msg.pose.pose.position.x = 1.25;
    msg.pose.pose.position.y = -2.5;
    msg.pose.pose.orientation.w = 2.0;  // normalized on receipt
    return msg;
  }
  void deliver(const nav_msgs::msg::Odometry & msg)
  {
    pub_->publish(std::make_unique<nav_msgs::msg::Odometry>(msg));
    rclcpp::spin_some(node_);
    handle_->update(node_->now());
  }
  rclcpp::Node::SharedPtr node_;
  LocalizationData data_;
  std::unique_ptr<LocalizationSensorHandle> handle_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

TEST_F(LocalizationHandleTest, OptionalStartupAndFreshPelvisPose)
{
  handle_->update(node_->now());
  EXPECT_TRUE(handle_->is_ready());
  EXPECT_FALSE(data_.valid);
  deliver(message());
  ASSERT_TRUE(data_.valid);
  EXPECT_TRUE(data_.position.isApprox(Eigen::Vector2f(1.25, -2.5)));
  EXPECT_FLOAT_EQ(data_.orientation.norm(), 1.0f);
}

TEST_F(LocalizationHandleTest, InvalidFramesAndValuesInvalidatePreviousPose)
{
  deliver(message());
  ASSERT_TRUE(data_.valid);
  auto msg = message();
  msg.child_frame_id = "torso_link";
  deliver(msg);
  EXPECT_FALSE(data_.valid);
  deliver(message());
  ASSERT_TRUE(data_.valid);
  msg = message();
  msg.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  deliver(msg);
  EXPECT_FALSE(data_.valid);
  msg = message();
  msg.pose.pose.orientation.w = 0;
  deliver(msg);
  EXPECT_FALSE(data_.valid);
  msg = message();
  msg.header.frame_id = "map";
  deliver(msg);
  EXPECT_FALSE(data_.valid);
}

TEST_F(LocalizationHandleTest, RejectsStaleSourceAndBackwardsTimestamps)
{
  auto msg = message();
  deliver(msg);
  ASSERT_TRUE(data_.valid);
  msg.header.stamp = rclcpp::Time(msg.header.stamp) - rclcpp::Duration::from_seconds(0.001);
  deliver(msg);
  EXPECT_FALSE(data_.valid);
  msg = message();
  deliver(msg);
  ASSERT_TRUE(data_.valid);
  handle_->update(node_->now() + rclcpp::Duration::from_seconds(0.2));
  EXPECT_FALSE(data_.valid);
  msg = message();
  msg.header.stamp = node_->now() + rclcpp::Duration::from_seconds(1.0);
  deliver(msg);
  EXPECT_FALSE(data_.valid);
}

TEST_F(LocalizationHandleTest, DuplicateKeepsLastPoseWithoutRefreshingTimeout)
{
  auto msg = message();
  deliver(msg);
  ASSERT_TRUE(data_.valid);
  const auto source_time = node_->now();
  // A same-timestamp contact publication must not replace the accepted sample.
  msg.pose.pose.position.x = 99.0;
  deliver(msg);
  EXPECT_TRUE(data_.valid);
  EXPECT_FLOAT_EQ(data_.position.x(), 1.25f);
  for (int i = 0; i < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(45));
    deliver(msg);
  }
  // Freeze ROS time to isolate the reception watchdog from source-age checks.
  handle_->update(source_time);
  EXPECT_FALSE(data_.valid);
  deliver(message());
  EXPECT_TRUE(data_.valid);
}

TEST_F(LocalizationHandleTest, DuplicateDoesNotHideInvalidPoseOrRestoreInvalidatedInput)
{
  const auto valid = message();
  deliver(valid);
  ASSERT_TRUE(data_.valid);
  auto invalid = valid;
  invalid.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  deliver(invalid);
  EXPECT_FALSE(data_.valid);
  deliver(valid);
  EXPECT_FALSE(data_.valid);
  deliver(message());
  EXPECT_TRUE(data_.valid);
}

TEST_F(LocalizationHandleTest, ReceptionTimeoutInvalidatesPose)
{
  deliver(message());
  ASSERT_TRUE(data_.valid);
  const auto source_time = node_->now();
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  handle_->update(source_time);
  EXPECT_FALSE(data_.valid);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
