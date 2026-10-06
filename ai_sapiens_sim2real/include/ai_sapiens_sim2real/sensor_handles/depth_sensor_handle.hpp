// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__DEPTH_SENSOR_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__DEPTH_SENSOR_HANDLE_HPP_
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <ai_sapiens_depth/depth_processor.hpp>
#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"
namespace ai_sapiens_sim2real
{
class DepthSensorHandle : public SensorHandleBase
{
public:
  DepthSensorHandle(
    rclcpp::Node::SharedPtr node, SensorData * sensors,
    const ai_sapiens_depth::DepthConfig & config);
  void start_episode();
  void update(const rclcpp::Time & time) override;
  std::string get_name() const override {return "depth_image";}
  bool is_ready() const override {return sensors_->depth_valid;}

private:
  struct Sample
  {
    ai_sapiens_interfaces::msg::DepthHistory message;
    std::chrono::steady_clock::time_point received{};
    bool valid{false};
  };
  void receive(ai_sapiens_interfaces::msg::DepthHistory::ConstSharedPtr message);
  SensorData * sensors_;
  ai_sapiens_depth::DepthConfig config_;
  realtime_tools::RealtimeBuffer<Sample> buffer_;
  rclcpp::Subscription<ai_sapiens_interfaces::msg::DepthHistory>::SharedPtr subscription_;
  double last_stamp_{-1};
  double entry_stamp_{-1};
  std::array<float, 576> entry_frame_{};
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__DEPTH_SENSOR_HANDLE_HPP_
