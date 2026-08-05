// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Kiwoong Park

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <ai_sapiens_interfaces/msg/mode_status.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/dualsense_teleop/dualsense_teleop_ui.hpp"
#include "ai_sapiens_sim2real/plugins/teleop_input/dualsense_teleop_input_plugin.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

class DualSenseTeleopUiNode : public rclcpp::Node
{
public:
  static std::shared_ptr<DualSenseTeleopUiNode> create()
  {
    auto node = std::shared_ptr<DualSenseTeleopUiNode>(
      new DualSenseTeleopUiNode());
    node->initialize();
    return node;
  }

private:
  DualSenseTeleopUiNode()
  : Node("dualsense_teleop_ui")
  {
  }

  void initialize()
  {
    const auto default_config =
      ament_index_cpp::get_package_share_directory("ai_sapiens_sim2real") +
      "/config/teleop/dualsense.yaml";
    declare_parameter<std::string>("config_path", default_config);
    const auto config_path = get_parameter("config_path").as_string();
    if (config_path.empty()) {
      throw std::runtime_error("DualSense teleop UI config_path must not be empty");
    }

    const auto yaml = YAML::LoadFile(config_path);
    config_ = DualSenseTeleopUiConfig::from_yaml(yaml);
    ui_ = std::make_unique<DualSenseTeleopUi>(config_);
    input_plugin_ = std::make_shared<DualSenseTeleopInputPlugin>();
    auto ui_plugin_config = YAML::Clone(yaml);
    ui_plugin_config["log_input_guide_enabled"] = false;
    input_plugin_->configure(shared_from_this(), ui_plugin_config);

    mode_status_subscription_ =
      create_subscription<ai_sapiens_interfaces::msg::ModeStatus>(
      config_.mode_status_topic, 10,
      [this](const ai_sapiens_interfaces::msg::ModeStatus::SharedPtr message) {
        state_.mode_status_received = true;
        mode_status_received_at_ = std::chrono::steady_clock::now();
        state_.teleop_input_valid = message->teleop_input_valid;
        state_.active_mode = message->active_mode;
        state_.authority = message->authority;
        state_.transition_reason = message->last_transition_reason;
      });

    const auto period = std::chrono::duration<double>(1.0 / config_.update_rate);
    render_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {render();});

    const char * term = std::getenv("TERM");
    use_color_ =
      isatty(STDOUT_FILENO) &&
      std::getenv("NO_COLOR") == nullptr &&
      (term == nullptr || std::string(term) != "dumb");
    render();
  }

  void render()
  {
    TeleopInputCommand command;
    state_.joy_received = input_plugin_->read_latest_accepted_command(command);
    if (state_.joy_received) {
      state_.command = command;
      const auto age = std::chrono::steady_clock::now() - command.received_at;
      state_.joy_fresh =
        age <= std::chrono::duration<double>(config_.stale_timeout);
    } else {
      state_.joy_fresh = false;
    }
    if (state_.mode_status_received) {
      const auto status_age =
        std::chrono::steady_clock::now() - mode_status_received_at_;
      state_.mode_status_fresh =
        status_age <= std::chrono::duration<double>(config_.stale_timeout);
    } else {
      state_.mode_status_fresh = false;
    }

    std::cout << "\033[2J\033[H"
              << ui_->dashboard(state_, use_color_)
              << std::flush;
  }

  DualSenseTeleopUiConfig config_;
  DualSenseTeleopUiState state_;
  std::unique_ptr<DualSenseTeleopUi> ui_;
  std::shared_ptr<DualSenseTeleopInputPlugin> input_plugin_;
  rclcpp::Subscription<ai_sapiens_interfaces::msg::ModeStatus>::SharedPtr
    mode_status_subscription_;
  rclcpp::TimerBase::SharedPtr render_timer_;
  std::chrono::steady_clock::time_point mode_status_received_at_{};
  bool use_color_{false};
};

}  // namespace
}  // namespace ai_sapiens_sim2real

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(ai_sapiens_sim2real::DualSenseTeleopUiNode::create());
  } catch (const std::exception & error) {
    std::cerr << "DualSense teleop UI error: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
