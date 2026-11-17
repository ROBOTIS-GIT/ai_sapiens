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

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "ai_sapiens_sim2real/mode_runtime/input_executor.hpp"
#include "ai_sapiens_sim2real/mode_runtime/startup_gate.hpp"

using ai_sapiens_sim2real::InputExecutor;
using namespace std::chrono_literals;

class InputExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(InputExecutorTest, ImmediateStopIsRepeatableAndJoinsTheThread)
{
  auto node = std::make_shared<rclcpp::Node>("input_executor_stop_test");
  InputExecutor inputs(node);
  inputs.stop();

  for (int i = 0; i < 20; ++i) {
    inputs.start();
    inputs.stop();
    inputs.stop();
    EXPECT_FALSE(inputs.has_failed());
  }

  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(InputExecutorTest, DestructorStopsCallbacksBeforeReturning)
{
  auto node = std::make_shared<rclcpp::Node>("input_executor_destructor_test");
  std::atomic<int> calls{0};
  rclcpp::TimerBase::SharedPtr timer;
  {
    InputExecutor inputs(node);
    timer = node->create_wall_timer(1ms, [&]() {++calls;},
      inputs.subscription_options().callback_group);
    inputs.start();

    for (int i = 0; i < 200 && calls.load() == 0; ++i) {
      std::this_thread::sleep_for(5ms);
    }
  }

  const int stopped_calls = calls.load();
  EXPECT_GT(stopped_calls, 0);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(calls.load(), stopped_calls);
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(InputExecutorTest, CallbackFailureShutsDownAndRemainsObservable)
{
  auto node = std::make_shared<rclcpp::Node>("input_executor_failure_test");
  InputExecutor inputs(node);
  auto timer = node->create_wall_timer(1ms, []() {throw std::runtime_error("test failure");},
    inputs.subscription_options().callback_group);
  inputs.start();

  for (int i = 0; i < 200 && rclcpp::ok(); ++i) {
    std::this_thread::sleep_for(5ms);
  }

  inputs.stop();
  EXPECT_TRUE(inputs.has_failed());
  EXPECT_FALSE(rclcpp::ok());
}

TEST_F(InputExecutorTest, StartupSpinOnlyRunsInputCallbacks)
{
  auto node = std::make_shared<rclcpp::Node>("input_executor_startup_test");
  InputExecutor inputs(node);
  int input_calls = 0;
  int ordinary_calls = 0;
  auto input_timer = node->create_wall_timer(1ms, [&]() {++input_calls;},
    inputs.subscription_options().callback_group);
  auto ordinary_timer = node->create_wall_timer(1ms, [&]() {++ordinary_calls;});

  for (int i = 0; i < 100 && input_calls == 0; ++i) {
    inputs.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_GT(input_calls, 0);
  EXPECT_EQ(ordinary_calls, 0);
}
