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

#include "ai_sapiens_sim2real/mode_runtime/node_executor.hpp"

#include <stdexcept>
#include <string>

#include <rcpputils/scope_exit.hpp>

namespace ai_sapiens_sim2real
{

NodeExecutor::NodeExecutor(const rclcpp::Node::SharedPtr & node)
: logger_(node->get_logger())
{
  add_node(node);
}

auto NodeExecutor::begin_spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("Node executor is already spinning");
  }

  wait_result_.reset();
  entities_need_rebuild_ = true;
  return rcpputils::make_scope_exit([this]() {
             wait_result_.reset();
             spinning.store(false);
  });
}

bool NodeExecutor::is_running() const
{
  return spinning.load() && rclcpp::ok(context_);
}

void NodeExecutor::spin()
{
  const auto spin_guard = begin_spin();
  while (is_running()) {
    rclcpp::AnyExecutable work;
    if (get_next_executable(work)) {
      dispatch(work);
    }
  }
}

void NodeExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  using namespace std::chrono_literals;
  const auto spin_guard = begin_spin();
  if (!is_running()) {
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  wait_for_work(0ns);
  while (is_running()) {
    if (max_duration != 0ns && std::chrono::steady_clock::now() - start >= max_duration) {
      return;
    }

    rclcpp::AnyExecutable work;
    if (!get_next_ready_executable(work)) {
      return;
    }
    dispatch(work);
  }
}

void NodeExecutor::dispatch(rclcpp::AnyExecutable & work)
{
  if (!work.service) {
    execute_any_executable(work);
    return;
  }

  try {
    execute_any_executable(work);
  } catch (const rclcpp::exceptions::RCLError & error) {
    RCLCPP_ERROR(logger_, "Service '%s' failed (rcl=%d): %s",
      work.service->get_service_name(), error.ret, error.what());
    // Jazzy identifies send_response() by its operation prefix. Other operations
    // can report the same RCL_RET_ERROR, including inside the service callback.
    if (error.ret != RCL_RET_ERROR ||
      error.what() != "failed to send response: " + error.formatted_message)
    {
      throw;
    }
  } catch (...) {
    RCLCPP_ERROR(logger_, "Exception in service '%s'", work.service->get_service_name());
    throw;
  }
}

}  // namespace ai_sapiens_sim2real
