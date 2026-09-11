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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__LOCALIZATION_SENSOR_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__LOCALIZATION_SENSOR_HANDLE_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_sim2real/policy/localization_pose.hpp"

namespace ai_sapiens_sim2real
{

struct LocalizationSample
{
  LocalizationData pose;
  std::chrono::steady_clock::time_point received_at{};
  int64_t stamp_ns{0};
};

class LocalizationSensorHandle : public SensorHandleBase
{
public:
  LocalizationSensorHandle(
    rclcpp::Node::SharedPtr node, LocalizationData * localization,
    double timeout_seconds = 0.5,
    const std::string & topic = "/state_estimator/odom",
    const std::string & world_frame = "odom", const std::string & base_frame = "pelvis");

  void update(const rclcpp::Time & time) override;
  std::string get_name() const override {return "localization";}
  // Do not block Damping/ReadyPose while the estimator is being initialized.
  // The policy checks fresh localization before entry and on every control tick.
  bool is_ready() const override {return true;}

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  LocalizationData * localization_;
  double timeout_seconds_;
  std::string world_frame_;
  std::string base_frame_;
  int64_t last_stamp_ns_{0};
  realtime_tools::RealtimeBuffer<LocalizationSample> buffer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__LOCALIZATION_SENSOR_HANDLE_HPP_
