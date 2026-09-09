// Copyright 2026 ROBOTIS CO., LTD. Licensed under Apache-2.0.
#ifndef AI_SAPIENS_MUJOCO__VIEWER_TELEOP_HPP_
#define AI_SAPIENS_MUJOCO__VIEWER_TELEOP_HPP_
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ai_sapiens_interfaces/msg/keyboard_input.hpp>
#include <ai_sapiens_interfaces/msg/mode_status.hpp>
#include "ai_sapiens_mujoco/mujoco_simulation.hpp"
#include "ai_sapiens_mujoco/mujoco_viewer_ui.hpp"
namespace ai_sapiens_mujoco {
/// Viewer-thread-only bridge; sends device-neutral commands on a dedicated topic.
class ViewerTeleop {
public:
  explicit ViewerTeleop(std::shared_ptr<MujocoSimulation> simulation);
  void tick();
  void request(ViewerUiAction action);
  ViewerControlState state;
private:
  using Clock = std::chrono::steady_clock;
  std::shared_ptr<MujocoSimulation> sim_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ai_sapiens_interfaces::msg::KeyboardInput>::SharedPtr publisher_;
  rclcpp::Subscription<ai_sapiens_interfaces::msg::ModeStatus>::SharedPtr subscription_;
  Clock::time_point last_status_{}, last_publish_{}, requested_at_{};
  uint32_t sequence_{0};
  uint16_t code_{1}, damping_{1}, ready_{2}, velocity_{3}, mimic_{4};
  int pulse_samples_{0}, neutral_samples_{0};
  std::string authority_;
  ViewerUiAction deferred_{ViewerUiAction::kNone};
};
}
#endif
