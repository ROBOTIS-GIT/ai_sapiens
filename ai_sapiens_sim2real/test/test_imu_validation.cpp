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
#include "ai_sapiens_sim2real/sensor_handles/sensor_input_validation.hpp"

using ai_sapiens_sim2real::ImuSensorHandle;
using ai_sapiens_sim2real::InputExecutor;
using ai_sapiens_sim2real::ModeRequests;
using ai_sapiens_sim2real::SensorData;
using ai_sapiens_sim2real::valid_imu_sample;

TEST(ImuValidation, RejectsUnavailableZeroNonfiniteAndGrossNormErrors)
{
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const auto identity = Eigen::Quaterniond::Identity();
  EXPECT_TRUE(valid_imu_sample(Eigen::Quaterniond(1.01, 0, 0, 0), zero, true, true));
  EXPECT_FALSE(valid_imu_sample(Eigen::Quaterniond(0, 0, 0, 0), zero, true, true));
  EXPECT_FALSE(valid_imu_sample(Eigen::Quaterniond(2, 0, 0, 0), zero, true, true));
  EXPECT_FALSE(valid_imu_sample(identity, zero, false, true));
  EXPECT_FALSE(valid_imu_sample(identity, zero, true, false));
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()})
  {
    EXPECT_FALSE(valid_imu_sample(Eigen::Quaterniond(bad, 0, 0, 0), zero, true, true));
    EXPECT_FALSE(valid_imu_sample(identity, Eigen::Vector3d(bad, 0, 0), true, true));
  }
}

class ImuSensorHandleTest : public ::testing::Test
{
protected:
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(ImuSensorHandleTest, RejectsInvalidWatchdogTimeouts)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("imu_timeout_test");
  SensorData sensors;
  ModeRequests requests;
  for (double timeout : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(ImuSensorHandle(node, &sensors, &requests, timeout), std::runtime_error);
  }
}

TEST_F(ImuSensorHandleTest, WatchdogUsesOnlyAcceptedSamplesAndLatchesOnTimeout)
{
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("imu_validation_test", options);
  SensorData sensors;
  ModeRequests requests;
  constexpr auto timeout = 500ms;
  ImuSensorHandle handle(node, &sensors, &requests, std::chrono::duration<double>(timeout).count(),
    "/test_imu_validation");
  auto publisher = node->create_publisher<sensor_msgs::msg::Imu>("/test_imu_validation", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  sensor_msgs::msg::Imu msg;
  const auto send = [&]() {
      publisher->publish(msg);
      executor.spin_all(20ms);
      handle.update(node->now());
    };
  msg.orientation.w = 0;
  send();
  EXPECT_FALSE(handle.is_ready());
  EXPECT_FALSE(sensors.is_imu_usable);
  EXPECT_FALSE(requests.damping);

  msg.orientation.w = 0.71417785;
  msg.orientation.y = 0.71417785;
  msg.angular_velocity.x = 0.75;
  send();
  ASSERT_TRUE(handle.is_ready());
  ASSERT_TRUE(sensors.is_imu_usable);
  EXPECT_NEAR(sensors.orientation.norm(), 1.0f, 1e-6f);
  const auto last_good = sensors.orientation;

  msg.orientation.w = 1;
  msg.orientation.y = 0;
  msg.angular_velocity.x = 2;
  msg.angular_velocity_covariance[0] = -1;
  send();
  EXPECT_FALSE(requests.damping);
  EXPECT_TRUE(sensors.is_imu_usable);
  EXPECT_TRUE(sensors.orientation.isApprox(last_good));
  EXPECT_FLOAT_EQ(sensors.angular_velocity.x(), 0.75f);

  std::this_thread::sleep_for(300ms);
  msg.angular_velocity_covariance[0] = 0;
  send();
  ASSERT_FALSE(requests.damping);
  ASSERT_TRUE(sensors.orientation.isApprox(Eigen::Quaternionf::Identity()));
  ASSERT_FLOAT_EQ(sensors.angular_velocity.x(), 2.0f);

  msg.angular_velocity.x = std::numeric_limits<double>::max();
  const auto deadline = std::chrono::steady_clock::now() + timeout + 30ms;
  // The first sample is now stale, but its valid replacement extended the deadline.
  std::this_thread::sleep_for(250ms);
  send();
  EXPECT_FALSE(requests.damping);
  EXPECT_TRUE(sensors.is_imu_usable);
  do {
    std::this_thread::sleep_for(10ms);
    send();
  } while (std::chrono::steady_clock::now() < deadline);
  EXPECT_TRUE(requests.damping);
  EXPECT_FALSE(sensors.is_imu_usable);
  EXPECT_FLOAT_EQ(sensors.angular_velocity.x(), 2.0f);
  EXPECT_TRUE(rclcpp::ok());

  requests.damping = false;
  msg.angular_velocity.x = 0;
  send();
  EXPECT_TRUE(requests.damping);
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
