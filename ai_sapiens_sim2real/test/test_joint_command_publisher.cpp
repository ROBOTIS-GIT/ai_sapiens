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

#include <rcl/error_handling.h>
#include <rcl/publisher.h>

#include <chrono>
#include <cstring>
#include <future>
#include <sstream>
#include <string>
#include <thread>

#include "ai_sapiens_sim2real/command_publishers/joint_command_publisher.hpp"

#include "ros_log_capture.hpp"

using ai_sapiens_sim2real::test::RosLogCapture;

namespace
{
constexpr char kTopic[] = "/test_joint_command_publish_fault";
thread_local bool reject_command = false;
}  // namespace

// Intercept only this test's command topic at the rclcpp/rcl boundary.
extern "C" rcl_ret_t __real_rcl_publish(
  const rcl_publisher_t *, const void *, rmw_publisher_allocation_t *);
extern "C" rcl_ret_t __wrap_rcl_publish(
  const rcl_publisher_t * pub, const void * msg, rmw_publisher_allocation_t * allocation)
{
  if (reject_command && rcl_publisher_get_topic_name(pub) &&
    std::strcmp(rcl_publisher_get_topic_name(pub), kTopic) == 0)
  {
    RCL_SET_ERROR_MSG("test joint command transport failure");
    return RCL_RET_ERROR;
  }
  return __real_rcl_publish(pub, msg, allocation);
}

TEST(JointCommandPublisher, CountsOnlyConsecutiveFailuresAndRecordsOnlyPublishedCommands)
{
  using ai_sapiens_sim2real::CommandPublishMode;
  ai_sapiens_sim2real::SharedControlData data;
  data.resize(1, 1);
  auto node = std::make_shared<rclcpp::Node>("joint_command_publisher_test");
  auto & output = data.output;
  ai_sapiens_sim2real::JointCommandPublisher publisher(node, &output, kTopic, {"joint"}, true);
  const auto publish = [&]() {publisher.publish(node->now(), CommandPublishMode::Control);};
  reject_command = true;
  publish();
  EXPECT_FALSE(output.has_published_command);

  reject_command = false;
  output.position_limits[0] = std::make_pair(-1.0f, 1.0f);
  output.command.position[0] = 2.0f;
  output.command.stiffness[0] = 3.0f;
  output.command.damping[0] = 4.0f;
  publish();
  ASSERT_TRUE(output.has_published_command);
  EXPECT_FLOAT_EQ(output.last_published.position[0], 1.0f);

  reject_command = true;
  output.command.position[0] = -0.25f;
  output.command.stiffness[0] = 0.0f;
  output.command.damping[0] = 0.5f;
  for (int i = 0; i < 10; ++i) {
    publish();
  }
  EXPECT_FALSE(output.is_command_publish_fault_latched);
  EXPECT_FLOAT_EQ(output.last_published.position[0], 1.0f);
  EXPECT_FLOAT_EQ(output.last_published.stiffness[0], 3.0f);
  EXPECT_FLOAT_EQ(output.last_published.damping[0], 4.0f);

  reject_command = false;
  publish();
  EXPECT_FLOAT_EQ(output.last_published.position[0], -0.25f);
  EXPECT_FLOAT_EQ(output.last_published.stiffness[0], 0.0f);
  EXPECT_FLOAT_EQ(output.last_published.damping[0], 0.5f);

  reject_command = true;
  for (int i = 0; i < 10; ++i) {
    publish();
  }
  EXPECT_FALSE(output.is_command_publish_fault_latched);
  publish();
  EXPECT_TRUE(output.is_command_publish_fault_latched);
  reject_command = false;
  publish();
  EXPECT_TRUE(output.is_command_publish_fault_latched);
}

TEST(JointCommandPublisher, AlternatingFailuresRateLimitRetryLogsButStillReportALatch)
{
  using namespace std::chrono_literals;
  using Clock = std::chrono::steady_clock;
  ai_sapiens_sim2real::SharedControlData data;
  data.resize(1, 1);
  auto node = std::make_shared<rclcpp::Node>("joint_command_retry_log_test");
  ai_sapiens_sim2real::JointCommandPublisher publisher(node, &data.output, kTopic, {"joint"});
  const auto publish = [&](bool reject) {
      reject_command = reject;
      publisher.publish(rclcpp::Time(0), ai_sapiens_sim2real::CommandPublishMode::Control);
    };

  RosLogCapture capture(node->get_logger().get_name());
  const auto start = Clock::now();
  for (int tick = 0; tick < 1200; ++tick) {
    std::this_thread::sleep_until(start + tick * 1ms);
    publish(tick % 2 == 0);
    publisher.flush_diagnostics();
  }
  const auto max_pairs = 1 + (Clock::now() - start) / 1s;
  std::istringstream logs(capture.take());
  int failures = 0;
  int recoveries = 0;
  std::string line;
  while (std::getline(logs, line)) {
    failures += line.find("Joint command publish failed on attempt 1") != std::string::npos;
    recoveries += line.find("recovered on attempt 2 (retry 1,") != std::string::npos;
  }
  EXPECT_GE(failures, 1);
  EXPECT_LE(failures, max_pairs);
  EXPECT_EQ(recoveries, failures);
  RecordProperty("failure_warnings", failures);
  RecordProperty("recovery_notices", recoveries);
  EXPECT_FALSE(data.output.is_command_publish_fault_latched);

  for (int i = 0; i < 11; ++i) {
    publish(true);
  }
  publish(false);
  EXPECT_TRUE(capture.take().empty());
  publisher.flush_diagnostics();
  const auto latch_logs = capture.take();
  EXPECT_TRUE(data.output.is_command_publish_fault_latched);
  EXPECT_NE(latch_logs.find("Damping latched after 11 consecutive"), std::string::npos);
  EXPECT_NE(latch_logs.find("recovered on attempt 12"), std::string::npos);
}

TEST(JointCommandPublisher, BlockedLogOutputDoesNotBlockRetryLatchOrEmergency)
{
  using namespace std::chrono_literals;
  using ai_sapiens_sim2real::CommandPublishMode;
  ai_sapiens_sim2real::SharedControlData data;
  data.resize(1, 1);
  auto node = std::make_shared<rclcpp::Node>("blocked_command_log_test");
  ai_sapiens_sim2real::JointCommandPublisher publisher(node, &data.output, kTopic, {"joint"});
  const auto publish = [&](bool reject, CommandPublishMode mode = CommandPublishMode::Control) {
      reject_command = reject;
      publisher.publish(rclcpp::Time(0), mode);
    };

  publish(true);
  std::promise<void> entered;
  auto waiting = entered.get_future();
  std::promise<void> release;
  auto released = release.get_future().share();
  std::once_flag first_log;
  RosLogCapture capture(node->get_logger().get_name(), [&]() {
      std::call_once(first_log, [&]() {
        entered.set_value();
        released.wait();
    });
    });
  auto consumer = std::async(std::launch::async, [&]() {publisher.flush_diagnostics();});
  const auto log_started = waiting.wait_for(2s);
  if (log_started != std::future_status::ready) {
    release.set_value();
    FAIL() << "No first-failure log received";
  }

  auto producer = std::async(std::launch::async, [&]() {
        publish(false);
        for (int i = 0; i < 11; ++i) {
          publish(true);
        }
        publish(false);
        publish(true, CommandPublishMode::EmergencyDamping);
  });
  const auto completed_while_blocked = producer.wait_for(1s);
  release.set_value();
  producer.get();
  consumer.get();
  publisher.flush_diagnostics();
  reject_command = false;

  EXPECT_EQ(completed_while_blocked, std::future_status::ready);
  EXPECT_TRUE(data.output.is_command_publish_fault_latched);
  const auto logs = capture.take();
  for (const char * message : {"up to 10 more attempts", "recovered on attempt 2",
      "Damping latched after 11", "recovered on attempt 12", "Emergency Damping publish failed",
      "test joint command transport failure"})
  {
    EXPECT_NE(logs.find(message), std::string::npos) << message;
  }
  EXPECT_NE(logs.find("rcl=" + std::to_string(RCL_RET_ERROR)), std::string::npos);
}

TEST(JointCommandPublisher, IntraProcessSubscriberReceivesCommand)
{
  using namespace std::chrono_literals;
  using Command = ai_sapiens_interfaces::msg::JointImpedanceCommand;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("local_joint_command_test", options);
  std::promise<double> received_position;
  auto received = received_position.get_future();
  auto subscription = node->create_subscription<Command>(
    kTopic, rclcpp::SystemDefaultsQoS(), [&](Command::ConstSharedPtr msg) {
      received_position.set_value(msg->positions.at(0));
    });

  ai_sapiens_sim2real::SharedControlData data;
  data.resize(1, 1);
  data.output.command.position[0] = 0.25f;
  ai_sapiens_sim2real::JointCommandPublisher publisher(node, &data.output, kTopic, {"joint"});
  reject_command = false;
  publisher.publish(rclcpp::Time(0), ai_sapiens_sim2real::CommandPublishMode::Control);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, received, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_DOUBLE_EQ(received.get(), 0.25);
  EXPECT_TRUE(data.output.has_published_command);
}

TEST(JointCommandPublisher, EmergencyAfterContextShutdownIsUnconfirmed)
{
  using ai_sapiens_sim2real::CommandPublishMode;
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  auto node = std::make_shared<rclcpp::Node>("closed_command_context_test", options);
  ai_sapiens_sim2real::SharedControlData data;
  data.resize(1, 1);
  ai_sapiens_sim2real::JointCommandPublisher publisher(node, &data.output, kTopic, {"joint"});
  reject_command = false;
  context->shutdown("test closed command transport");
  RosLogCapture capture(node->get_logger().get_name());
  publisher.publish(rclcpp::Time(0), CommandPublishMode::EmergencyDamping);
  EXPECT_FALSE(data.output.has_published_command);
  EXPECT_TRUE(capture.take().empty());
  publisher.flush_diagnostics();
  const auto logs = capture.take();
  EXPECT_NE(logs.find("Emergency Damping publish unconfirmed"), std::string::npos);
  EXPECT_NE(logs.find("ROS context shut down"), std::string::npos);
  EXPECT_EQ(logs.find("rcl="), std::string::npos);
}
