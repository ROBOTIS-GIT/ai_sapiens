#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include "ai_sapiens_sim2real/sensor_handles/imu_validation.hpp"
#include "ai_sapiens_sim2real/sensor_handles/imu_sensor_handle.hpp"

using namespace ai_sapiens_sim2real;

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
    EXPECT_FALSE(valid_imu_sample(Eigen::Quaternionf::Identity(), Eigen::Vector3f(bad, 0, 0), true, true));
  }
}

TEST(ImuSensorHandle, WaitsForValidSampleThenLatchesFaultWithoutReplacingPose)
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
  EXPECT_FALSE(sensors.imu_usable);
  EXPECT_FALSE(requests.damping);
  message.orientation.w = 0.71417785;  // 90 degrees around Y, norm approximately 1.01.
  message.orientation.y = 0.71417785;
  send(message);
  ASSERT_TRUE(handle.is_ready());
  ASSERT_TRUE(sensors.imu_usable);
  EXPECT_NEAR(sensors.orientation.norm(), 1.0f, 1e-6f);
  const auto last_good = sensors.orientation;
  message.angular_velocity_covariance[0] = -1;
  send(message);
  EXPECT_TRUE(requests.damping);
  EXPECT_FALSE(sensors.imu_usable);
  EXPECT_TRUE(sensors.orientation.isApprox(last_good));
  requests.damping = false;
  message.orientation.w = 1;
  message.orientation.y = 0;
  message.angular_velocity_covariance[0] = 0;
  send(message);
  EXPECT_TRUE(requests.damping);  // Recovery cannot automatically resume policy.
  EXPECT_FALSE(sensors.imu_usable);
}
