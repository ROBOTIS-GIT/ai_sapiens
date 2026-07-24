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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__API_HEARTBEAT_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__API_HEARTBEAT_HANDLE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_interfaces/msg/api_heartbeat.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

struct ApiHeartbeatSnapshot
{
  uint32_t sequence{0};
  std::chrono::steady_clock::time_point received_at{};
};

class ApiHeartbeatHandle : public SensorHandleBase
{
public:
  ApiHeartbeatHandle(
    rclcpp::Node::SharedPtr node,
    ApiInput * api,
    const std::string & topic,
    double timeout_seconds);

  void update(const rclcpp::Time & /*time*/) override;

  std::string get_name() const override;

  bool is_ready() const override
  {
    return true;
  }

private:
  void log_stale_heartbeat_once(
    const ApiHeartbeatSnapshot & snapshot,
    std::chrono::steady_clock::time_point now);

  rclcpp::Node::SharedPtr node_;
  ApiInput * api_;
  std::string topic_;
  std::chrono::duration<double> timeout_;

  // Callback writes heartbeat snapshots; update() validates timeout/sequence progress.
  rclcpp::Subscription<ai_sapiens_interfaces::msg::ApiHeartbeat>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<ApiHeartbeatSnapshot> buffer_;

  std::atomic<bool> received_once_{false};
  std::optional<uint32_t> last_sequence_seen_;
  std::chrono::steady_clock::time_point last_sequence_advance_time_{
    std::chrono::steady_clock::now()};
  bool heartbeat_stale_logged_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__API_HEARTBEAT_HANDLE_HPP_
