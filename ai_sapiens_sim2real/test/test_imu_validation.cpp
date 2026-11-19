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

#include <future>
#include <limits>
#include <thread>

#include "ai_sapiens_sim2real/mode_runtime/input_executor.hpp"
#include "ai_sapiens_sim2real/sensor_handles/imu_sensor_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/imu_validation.hpp"

using ai_sapiens_sim2real::ImuSensorHandle;
using ai_sapiens_sim2real::InputExecutor;
using ai_sapiens_sim2real::ModeRequests;
using ai_sapiens_sim2real::SensorData;
using ai_sapiens_sim2real::valid_imu_sample;

TEST(ImuValidation, RejectsUnavailableZeroNonfiniteAndGrossNormErrors)
{
  const auto zero = Eigen::Vector3f::Zero().eval();
  EXPECT_TRUE(valid_imu_sample(Eigen::Quaternionf(1.01f, 0, 0, 0), zero, true, true));
  EXPECT_FALSE(valid_imu_sample(Eigen::Quaternionf(0, 0, 0, 0), zero, true, true));
  EXPECT_FALSE(valid_imu_sample(Eigen::Quaternionf(2, 0, 0, 0), zero, true, true));
  EXPECT_FALSE(valid_imu_sample(Eigen::Quaternionf::Identity(), zero, false, true));
  EXPECT_FALSE(valid_imu_sample(Eigen::Quaternionf::Identity(), zero, true, false));
  for (float bad : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()})
  {
    EXPECT_FALSE(valid_imu_sample(Eigen::Quaternionf(bad, 0, 0, 0), zero, true, true));
    EXPECT_FALSE(valid_imu_sample(Eigen::Quaternionf::Identity(), Eigen::Vector3f(bad, 0, 0), true,
      true));
  }
}

class ImuSensorHandleTest : public ::testing::Test
{
protected:
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(ImuSensorHandleTest, WaitsForValidSampleThenLatchesFaultWithoutReplacingPose)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("imu_validation_test");
  SensorData sensors;
  ModeRequests requests;
  ImuSensorHandle handle(node, &sensors, &requests, 10.0, "/test_imu_validation");
  auto publisher = node->create_publisher<sensor_msgs::msg::Imu>("/test_imu_validation", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  for (int i = 0; i < 200 && publisher->get_subscription_count() == 0; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  auto send = [&](const sensor_msgs::msg::Imu & message) {
      for (int i = 0; i < 10; ++i) {
        publisher->publish(message);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      executor.spin_some();
      handle.update(node->now());
    };
  sensor_msgs::msg::Imu message;
  message.orientation.w = 0;  // Explicit zero; ROS defaults can be identity.
  send(message);
  EXPECT_FALSE(handle.is_ready());
  EXPECT_FALSE(sensors.is_imu_usable);
  EXPECT_FALSE(requests.damping);
  message.orientation.w = 0.71417785;  // 90 degrees around Y, norm approximately 1.01.
  message.orientation.y = 0.71417785;
  send(message);
  ASSERT_TRUE(handle.is_ready());
  ASSERT_TRUE(sensors.is_imu_usable);
  EXPECT_NEAR(sensors.orientation.norm(), 1.0f, 1e-6f);
  const auto last_good = sensors.orientation;
  message.angular_velocity_covariance[0] = -1;
  send(message);
  EXPECT_TRUE(requests.damping);
  EXPECT_FALSE(sensors.is_imu_usable);
  EXPECT_TRUE(sensors.orientation.isApprox(last_good));
  requests.damping = false;
  message.orientation.w = 1;
  message.orientation.y = 0;
  message.angular_velocity_covariance[0] = 0;
  send(message);
  EXPECT_TRUE(requests.damping);  // Recovery cannot automatically resume policy.
  EXPECT_FALSE(sensors.is_imu_usable);
}

TEST_F(ImuSensorHandleTest, ReceivesWhileOrdinaryExecutorIsBlocked)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("isolated_imu_test");
  InputExecutor inputs(node);
  const auto & options = inputs.subscription_options();
  SensorData sensors;
  ModeRequests requests;
  ImuSensorHandle handle(node, &sensors, &requests, 10.0, "/isolated_imu_test", options);
  auto publisher = node->create_publisher<sensor_msgs::msg::Imu>("/isolated_imu_test", 10);
  rclcpp::executors::SingleThreadedExecutor ordinary;
  ordinary.add_node(node);

  std::promise<void> entered;
  auto entered_future = entered.get_future();
  std::promise<void> release;
  auto release_future = release.get_future();
  std::atomic<bool> is_blocked{false};
  auto timer = node->create_wall_timer(std::chrono::milliseconds(1), [&]() {
        is_blocked = true;
        entered.set_value();
        release_future.wait_for(std::chrono::seconds(3));
        is_blocked = false;
        ordinary.cancel();
    });
  inputs.start();
  std::thread worker([&]() {ordinary.spin();});
  const bool did_enter = entered_future.wait_for(std::chrono::seconds(2)) ==
    std::future_status::ready;
  sensor_msgs::msg::Imu msg;
  msg.orientation.w = 1;
  msg.angular_velocity.x = 0.75;
  for (int i = 0; i < 200 && !handle.is_ready(); ++i) {
    publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const bool received_while_blocked = handle.is_ready() && is_blocked.load();
  release.set_value();
  worker.join();
  inputs.stop();
  handle.update(node->now());
  EXPECT_TRUE(did_enter);
  EXPECT_TRUE(received_while_blocked);
  EXPECT_NEAR(sensors.angular_velocity.x(), 0.75, 1e-6);
  EXPECT_FALSE(requests.damping);
}
