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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__INPUT_EXECUTOR_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__INPUT_EXECUTOR_HPP_

#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace ai_sapiens_sim2real
{

// Owns the serial input callback group and its execution thread.
// Lifecycle methods are called by the node thread; stop() is idempotent.
class InputExecutor
{
public:
  explicit InputExecutor(const rclcpp::Node::SharedPtr & node);
  ~InputExecutor();

  InputExecutor(const InputExecutor &) = delete;
  InputExecutor & operator=(const InputExecutor &) = delete;

  const rclcpp::SubscriptionOptions & subscription_options() const;
  void spin_some();  // Startup only, before start().
  void start();
  void stop();
  bool has_failed() const;

private:
  void run();

  rclcpp::Logger logger_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::SubscriptionOptions subscription_options_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> has_failed_{false};
  std::atomic<bool> has_finished_{false};
  std::thread thread_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__INPUT_EXECUTOR_HPP_
