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

#include "ai_sapiens_sim2real/mode_runtime/input_executor.hpp"

#include <stdexcept>

namespace ai_sapiens_sim2real
{
namespace
{
rclcpp::ExecutorOptions executor_options(const rclcpp::Context::SharedPtr & context)
{
  rclcpp::ExecutorOptions options;
  options.context = context;
  return options;
}
}  // namespace

InputExecutor::InputExecutor(const rclcpp::Node::SharedPtr & node)
: logger_(node->get_logger()),
  context_(node->get_node_base_interface()->get_context()),
  callback_group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false)),
  executor_(executor_options(context_))
{
  subscription_options_.callback_group = callback_group_;
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());
}

InputExecutor::~InputExecutor()
{
  stop();
}

const rclcpp::SubscriptionOptions & InputExecutor::subscription_options() const
{
  return subscription_options_;
}

void InputExecutor::spin_some()
{
  executor_.spin_some();
}

void InputExecutor::start()
{
  if (thread_.joinable()) {
    throw std::logic_error("input executor is already started");
  }

  has_failed_ = false;
  has_finished_ = false;
  thread_ = std::thread(&InputExecutor::run, this);

  // A cancel issued before spin() starts can be lost. Establish startup before
  // allowing the caller to stop, including when spin() immediately fails.
  while (!executor_.is_spinning() && !has_finished_.load()) {
    std::this_thread::yield();
  }
}

void InputExecutor::stop()
{
  if (!thread_.joinable()) {
    return;
  }

  executor_.cancel();
  thread_.join();
}

bool InputExecutor::has_failed() const
{
  return has_failed_.load();
}

void InputExecutor::run()
{
  try {
    executor_.spin();
  } catch (const std::exception & e) {
    has_failed_ = true;
    RCLCPP_FATAL(logger_, "Input executor failed: %s", e.what());
    rclcpp::shutdown(context_);
  } catch (...) {
    has_failed_ = true;
    RCLCPP_FATAL(logger_, "Input executor failed with an unknown exception");
    rclcpp::shutdown(context_);
  }

  has_finished_ = true;
}

}  // namespace ai_sapiens_sim2real
