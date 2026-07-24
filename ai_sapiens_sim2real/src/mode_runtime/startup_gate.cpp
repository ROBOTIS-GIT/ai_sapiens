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

#include "ai_sapiens_sim2real/mode_runtime/startup_gate.hpp"

#include <chrono>
#include <string>
#include <thread>

namespace ai_sapiens_sim2real
{

bool wait_for_teleop_input_sample(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<TeleopInputPluginBase> & plugin,
  double timeout_seconds)
{
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(timeout_seconds);
  const std::string topic_name = plugin->topic_name();

  while (rclcpp::ok() && !plugin->is_ready()) {
    rclcpp::spin_some(node);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double elapsed_seconds = std::chrono::duration<double>(elapsed).count();
    RCLCPP_INFO_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      1000,
      "[startup_wait] teleop_input plugin(%s) topic(%s) wait(%.1f/%.1fs)",
      plugin->name().c_str(),
      topic_name.c_str(),
      elapsed_seconds,
      timeout_seconds);

    const auto timeout_elapsed = std::chrono::steady_clock::now() - start;
    const bool has_startup_timed_out = timeout_seconds > 0.0 && timeout_elapsed > timeout;
    if (has_startup_timed_out) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[startup_error] teleop_input plugin(%s) topic(%s) reason(no_valid_sample) "
        "timeout(%.1fs)",
        plugin->name().c_str(),
        topic_name.c_str(),
        timeout_seconds);
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  const auto elapsed = std::chrono::steady_clock::now() - start;
  RCLCPP_INFO(
    node->get_logger(),
    "[startup_ready] teleop_input plugin(%s) topic(%s) ready(%.1fs)",
    plugin->name().c_str(),
    topic_name.c_str(),
    std::chrono::duration<double>(elapsed).count());
  return true;
}

}  // namespace ai_sapiens_sim2real
