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

#include <dlfcn.h>

#include <rcl/error_handling.h>
#include <rcl/service.h>
#include <rcl/wait.h>

#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <new>
#include <string>
#include <thread>

#include "ai_sapiens_interfaces/srv/list_modes.hpp"
#include "ai_sapiens_sim2real/mode_runtime/node_executor.hpp"

#include "ros_log_capture.hpp"

namespace
{
using ai_sapiens_sim2real::NodeExecutor;
using ListModes = ai_sapiens_interfaces::srv::ListModes;
using namespace std::chrono_literals;

constexpr char kService[] = "/test_executor_list_modes";
thread_local bool fail_next_response = false;
thread_local bool fail_next_wait = false;
}  // namespace

extern "C" rcl_ret_t __real_rcl_send_response(
  const rcl_service_t *, rmw_request_id_t *, void *);
extern "C" rcl_ret_t __wrap_rcl_send_response(
  const rcl_service_t * service, rmw_request_id_t * header, void * response)
{
  if (fail_next_response && std::strcmp(rcl_service_get_service_name(service), kService) == 0) {
    fail_next_response = false;
    RCL_SET_ERROR_MSG("test service response failure");
    return RCL_RET_ERROR;
  }
  return __real_rcl_send_response(service, header, response);
}

// The wait call originates in librclcpp, outside the reach of the linker's --wrap.
extern "C" rcl_ret_t rcl_wait(rcl_wait_set_t * wait_set, int64_t timeout)
{
  if (fail_next_wait) {
    fail_next_wait = false;
    RCL_SET_ERROR_MSG("test executor wait failure");
    return RCL_RET_ERROR;
  }
  static const auto real_wait = reinterpret_cast<decltype(& rcl_wait)>(dlsym(RTLD_NEXT,
    "rcl_wait"));
  return real_wait(wait_set, timeout);
}

enum class SpinMode {Startup, Runtime};

class NodeServiceDispatchTest : public testing::TestWithParam<SpinMode>
{
protected:
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("node_executor_service_test");
  NodeExecutor executor{node};
  rclcpp::Client<ListModes>::SharedPtr client = node->create_client<ListModes>(kService);
  ai_sapiens_sim2real::test::RosLogCapture logs{node->get_logger().get_name()};

  void request_and_spin()
  {
    ASSERT_TRUE(client->wait_for_service(3s));
    auto reply = client->async_send_request(std::make_shared<ListModes::Request>(),
        [&](rclcpp::Client<ListModes>::SharedFuture) {executor.cancel();});
    if (GetParam() == SpinMode::Runtime) {
      auto timeout = node->create_wall_timer(3s, [&]() {executor.cancel();});
      executor.spin();
    } else {
      const auto deadline = std::chrono::steady_clock::now() + 3s;
      while (reply.wait_for(0s) != std::future_status::ready &&
        std::chrono::steady_clock::now() < deadline)
      {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
    }
    ASSERT_EQ(reply.wait_for(0s), std::future_status::ready);
    EXPECT_TRUE(reply.get()->success);
  }
};

TEST_P(NodeServiceDispatchTest, ContinuesAfterFailedResponseSend)
{
  int calls = 0;
  auto service = node->create_service<ListModes>(kService,
      [&](ListModes::Request::SharedPtr, ListModes::Response::SharedPtr response) {
        ++calls;
        response->success = true;
    });
  ASSERT_TRUE(client->wait_for_service(3s));
  fail_next_response = true;
  auto first = client->async_send_request(std::make_shared<ListModes::Request>());
  ASSERT_NO_THROW(request_and_spin());

  EXPECT_FALSE(fail_next_response);
  EXPECT_EQ(calls, 2);
  EXPECT_EQ(first.wait_for(0s), std::future_status::timeout);
  const auto output = logs.take();
  EXPECT_NE(output.find(kService), std::string::npos);
  EXPECT_NE(output.find("test service response failure"), std::string::npos);
}

TEST_P(NodeServiceDispatchTest, PropagatesCallbackRclErrorWithSameReturnCode)
{
  auto service = node->create_service<ListModes>(kService,
      [](ListModes::Request::SharedPtr, ListModes::Response::SharedPtr) {
        RCL_SET_ERROR_MSG("failed to send response: text supplied by the middleware");
        rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR);
    });
  EXPECT_THROW(request_and_spin(), rclcpp::exceptions::RCLError);
  EXPECT_FALSE(executor.is_spinning());
  EXPECT_NE(logs.take().find(kService), std::string::npos);
}

TEST_P(NodeServiceDispatchTest, PropagatesCallbackAllocationFailure)
{
  auto service = node->create_service<ListModes>(kService,
      [](ListModes::Request::SharedPtr, ListModes::Response::SharedPtr) {
        throw std::bad_alloc();
    });
  EXPECT_THROW(request_and_spin(), std::bad_alloc);
  EXPECT_FALSE(executor.is_spinning());
  EXPECT_NE(logs.take().find(kService), std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(StartupAndRuntime, NodeServiceDispatchTest,
  testing::Values(SpinMode::Startup, SpinMode::Runtime));

TEST(NodeExecutor, PropagatesWaitFailureAndClearsSpinning)
{
  auto node = std::make_shared<rclcpp::Node>("node_executor_wait_test");
  NodeExecutor executor(node);
  auto timeout = node->create_wall_timer(3s, [&]() {executor.cancel();});
  fail_next_wait = true;
  EXPECT_THROW(executor.spin_some(), rclcpp::exceptions::RCLError);
  EXPECT_FALSE(fail_next_wait);
  EXPECT_FALSE(executor.is_spinning());
  fail_next_wait = true;
  EXPECT_THROW(executor.spin(), rclcpp::exceptions::RCLError);
  EXPECT_FALSE(fail_next_wait);
  EXPECT_FALSE(executor.is_spinning());
}
