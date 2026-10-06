// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <thread>
#include "ai_sapiens_sim2real/sensor_handles/depth_sensor_handle.hpp"
TEST(DepthSensor, ValidityReplayWatchdogAndEpisodeWarmFill)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("depth_sensor_test");
  ai_sapiens_sim2real::SensorData sensors;
  ai_sapiens_depth::DepthConfig config;
  config.topic = "/test/parkour/depth"; config.frame = "camera_link";
  {
    ai_sapiens_sim2real::DepthSensorHandle handle(node, &sensors, config);
    auto publisher = node->create_publisher<ai_sapiens_interfaces::msg::DepthHistory>(
      config.topic, rclcpp::SensorDataQoS());
    for (int i = 0; i < 50 && publisher->get_subscription_count() == 0; ++i) {
      rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    handle.update(node->now()); EXPECT_FALSE(handle.is_ready());
    ai_sapiens_interfaces::msg::DepthHistory message;
    message.header.frame_id = config.frame;
    message.width = 32; message.height = 18; message.frames = 8;
    auto send = [&]() {
        publisher->publish(message);
        for (int i = 0; i < 5; ++i) {
          rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        handle.update(node->now());
      };
    message.header.stamp = node->now();
    const double stamp = rclcpp::Time(message.header.stamp).seconds();
    for (int i = 0; i < 8; ++i) {
      message.frame_stamps[i] = stamp - (7 - i) * 0.1;
      std::fill(message.data.begin() + i * 576, message.data.begin() + (i + 1) * 576, i / 10.0f);
    }
    send(); ASSERT_TRUE(handle.is_ready());
    EXPECT_FLOAT_EQ(sensors.depth_history.front(), 0);
    handle.start_episode();
    EXPECT_FLOAT_EQ(sensors.depth_history.front(), 0.7f);
    handle.update(node->now());
    EXPECT_FLOAT_EQ(sensors.depth_history.front(), 0.7f);
    // Replayed acquisition stamps must not extend freshness.
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    send(); EXPECT_FALSE(handle.is_ready());
    message.header.stamp = node->now(); message.frame_stamps.fill(node->now().seconds() - 0.01);
    message.data.fill(0.5); message.data[10] = NAN;
    send(); EXPECT_FALSE(handle.is_ready());
    message.header.stamp = node->now(); message.frame_stamps.fill(node->now().seconds() - 0.01);
    message.data.fill(0.5);
    send(); EXPECT_TRUE(handle.is_ready());
    handle.update(node->now() + rclcpp::Duration::from_seconds(0.11));
    EXPECT_FALSE(handle.is_ready());
  }
  node.reset(); rclcpp::shutdown();
}
