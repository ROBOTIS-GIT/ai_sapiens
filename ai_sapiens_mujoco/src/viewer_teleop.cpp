// Copyright 2026 ROBOTIS CO., LTD. Licensed under Apache-2.0.
#include "ai_sapiens_mujoco/viewer_teleop.hpp"
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
namespace ai_sapiens_mujoco {
ViewerTeleop::ViewerTeleop(std::shared_ptr<MujocoSimulation> simulation) : sim_(simulation) {
  node_ = std::make_shared<rclcpp::Node>("mujoco_viewer_teleop",
    rclcpp::NodeOptions().use_global_arguments(false));
  const auto share = ament_index_cpp::get_package_share_directory("ai_sapiens_sim2real");
  const auto root = YAML::LoadFile(share + "/config/k1_config.yaml");
  const auto config = YAML::LoadFile(share + "/config/teleop/gui.yaml");
  const auto codes = config["input_code"];
  damping_ = codes["damping"].as<uint16_t>(); ready_ = codes["ready_pose"].as<uint16_t>();
  velocity_ = codes["velocity"].as<uint16_t>(); mimic_ = codes["mimic"].as<uint16_t>();
  code_ = damping_;
  const auto selector = config["selector_navigation"]["selector"].as<std::string>();
  for (const auto & entry : root["selectors"][selector]["table"]) {
    state.motions.push_back({entry.first.as<uint16_t>(), entry.second.as<std::string>()});
  }
  if (state.motions.empty()) throw std::runtime_error("GUI teleop requires a mimic selector");
  const auto behavior = root["state_machine"]["states"]["Velocity"]["run"].as<std::string>();
  state.velocity_asset = root["state_behaviors"][behavior]["asset"].as<std::string>();
  publisher_ = node_->create_publisher<ai_sapiens_interfaces::msg::KeyboardInput>(
    config["topic"].as<std::string>(), rclcpp::SensorDataQoS());
  subscription_ = node_->create_subscription<ai_sapiens_interfaces::msg::ModeStatus>(
    config["ui"]["mode_status_topic"].as<std::string>(), 10,
    [this](const ai_sapiens_interfaces::msg::ModeStatus & msg) {
      state.mode = msg.active_mode; authority_ = msg.authority;
      last_status_ = Clock::now();
    });
}
void ViewerTeleop::tick() {
  rclcpp::spin_some(node_);
  const auto now = Clock::now();
  const bool was_connected = state.connected;
  state.connected = publisher_->get_subscription_count() > 0 &&
    now - last_status_ < std::chrono::milliseconds(500) && authority_ == "MANUAL";
  state.paused = sim_->paused();
  if (!state.connected) {
    code_ = damping_; pulse_samples_ = neutral_samples_ = 0;
    std::fill(std::begin(state.velocity), std::end(state.velocity), 0.0f);
    deferred_ = ViewerUiAction::kNone; state.busy = false;
    state.note = "Start with --sim --gui; waiting for controller";
  } else if (!was_connected) {
    state.note = "GUI connected. Select Walkready, then Start Velocity.";
  }
  if (deferred_ != ViewerUiAction::kNone) {
    if (now - requested_at_ > std::chrono::seconds(3)) {
      deferred_ = ViewerUiAction::kNone; state.busy = false;
      state.note = "Damping acknowledgement timed out; operation cancelled";
    } else if (state.mode == "Damping" && last_status_ > requested_at_ &&
      now - requested_at_ > std::chrono::milliseconds(150)) {
      if (deferred_ == ViewerUiAction::kReset) {
        sim_->reset(); sim_->set_paused(false);
        state.note = "Reset complete. Robot attached; select Walkready.";
      } else {
        sim_->set_paused(true); state.note = "Paused in Damping";
      }
      deferred_ = ViewerUiAction::kNone; state.busy = false;
    }
  }
  if (now - last_publish_ < std::chrono::milliseconds(50)) return;
  last_publish_ = now;
  ai_sapiens_interfaces::msg::KeyboardInput msg;
  msg.header.stamp = node_->now(); msg.sequence = ++sequence_;
  msg.selector_code = state.motions.at(state.selected_motion).code;
  msg.input_code = code_;
  if (neutral_samples_ > 0) { msg.input_code = 0; --neutral_samples_; }
  else if (pulse_samples_ > 0) { --pulse_samples_; if (pulse_samples_ == 0) code_ = 0; }
  if (state.connected && !state.paused && !state.busy && state.mode == "Velocity") {
    auto normalized = [](float v) { return std::isfinite(v) ? std::clamp(v, -1.0f, 1.0f) : 0.0f; };
    msg.linear_x = normalized(state.velocity[0]);
    msg.linear_y = normalized(state.velocity[1]);
    msg.angular_z = normalized(state.velocity[2]);
  }
  publisher_->publish(msg);
}
void ViewerTeleop::request(ViewerUiAction action) {
  if (!state.connected || state.busy) return;
  if (action == ViewerUiAction::kStopVelocity) {
    std::fill(std::begin(state.velocity), std::end(state.velocity), 0.0f); return;
  }
  std::fill(std::begin(state.velocity), std::end(state.velocity), 0.0f);
  pulse_samples_ = neutral_samples_ = 0;
  switch (action) {
    case ViewerUiAction::kReset:
    case ViewerUiAction::kPause:
      code_ = damping_;
      if (action == ViewerUiAction::kPause && sim_->paused()) {
        sim_->set_paused(false); state.note = "Resumed in Damping"; break;
      }
      deferred_ = action; requested_at_ = Clock::now(); state.busy = true;
      state.note = "Waiting for Damping acknowledgement..."; break;
    case ViewerUiAction::kDamping: code_ = damping_; state.note = "Damping requested"; break;
    case ViewerUiAction::kReadyPose:
      sim_->set_paused(false); code_ = ready_; state.note = "Walkready requested"; break;
    case ViewerUiAction::kVelocity:
      if (state.mode == "Damping" || state.paused) {
        state.note = "Select Walkready before starting Velocity"; break;
      }
      code_ = velocity_; state.note = "Velocity requested"; break;
    case ViewerUiAction::kMimic:
      if (state.mode != "Velocity" || state.paused) {
        state.note = "Start Velocity before running a motion"; break;
      }
      // An explicit neutral interval re-arms edge-triggered mimic requests.
      neutral_samples_ = 2; pulse_samples_ = 3; code_ = mimic_;
      state.note = "Motion requested: " + state.motions.at(state.selected_motion).name; break;
    default: break;
  }
}
}  // namespace ai_sapiens_mujoco
