// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef MUJOCO_HARDWARE_INTERFACE__DEPTH_CAMERA_NODE_HPP_
#define MUJOCO_HARDWARE_INTERFACE__DEPTH_CAMERA_NODE_HPP_
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ai_sapiens_depth/depth_processor.hpp>
#include "ai_sapiens_mujoco/mujoco_simulation.hpp"
namespace mujoco_hardware_interface
{
// Runs on the non-control executor. Copy simulation state under its lock;
// ray casting only touches the private mjData snapshot, never the RT state.
class DepthCameraNode : public rclcpp::Node
{
public:
  DepthCameraNode(
    std::shared_ptr<ai_sapiens_mujoco::MujocoSimulation> sim,
    const std::string & policy_yaml);
  ~DepthCameraNode() override;

private:
  void capture();
  std::shared_ptr<ai_sapiens_mujoco::MujocoSimulation> sim_;
  ai_sapiens_depth::DepthConfig config_;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> snapshot_{nullptr, mj_deleteData};
  int parent_{-1}, excluded_{-1};
  double last_time_{-1};
  std::vector<mjtNum> rays_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mujoco_hardware_interface
#endif  // MUJOCO_HARDWARE_INTERFACE__DEPTH_CAMERA_NODE_HPP_
