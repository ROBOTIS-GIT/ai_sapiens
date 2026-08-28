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

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <ai_sapiens_interfaces/msg/mode_status.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/teleop_devtools/keyboard/keyboard_teleop.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

class TerminalGuard
{
public:
  TerminalGuard()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error(
              "keyboard teleop requires an interactive terminal on stdin");
    }
    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      throw std::runtime_error(
              std::string("failed to read terminal settings: ") + std::strerror(errno));
    }
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ < 0) {
      throw std::runtime_error(
              std::string("failed to read terminal flags: ") + std::strerror(errno));
    }

    auto raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error(
              std::string("failed to enable keyboard raw mode: ") + std::strerror(errno));
    }
    if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) != 0) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      throw std::runtime_error(
              std::string("failed to enable non-blocking stdin: ") + std::strerror(errno));
    }
    active_ = true;
  }

  ~TerminalGuard()
  {
    if (!active_) {
      return;
    }
    fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
  }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard & operator=(const TerminalGuard &) = delete;

private:
  termios original_termios_{};
  int original_flags_{0};
  bool active_{false};
};

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
  : Node("keyboard_teleop")
  {
    const auto package_share =
      ament_index_cpp::get_package_share_directory("ai_sapiens_sim2real");
    const auto default_config = package_share + "/config/teleop/keyboard.yaml";
    const auto default_root_config = package_share + "/config/k1_config.yaml";
    declare_parameter<std::string>("config_path", default_config);
    declare_parameter<std::string>("root_config_path", default_root_config);
    const auto config_path = get_parameter("config_path").as_string();
    const auto root_config_path = get_parameter("root_config_path").as_string();
    if (config_path.empty()) {
      throw std::runtime_error("keyboard teleop config_path must not be empty");
    }
    if (root_config_path.empty()) {
      throw std::runtime_error("keyboard teleop root_config_path must not be empty");
    }

    config_ = KeyboardTeleopConfig::from_yaml(
      resolve_keyboard_selector_config(
        YAML::LoadFile(config_path), YAML::LoadFile(root_config_path)));
    state_ = std::make_unique<KeyboardTeleopState>(config_);
    publisher_ = create_publisher<ai_sapiens_interfaces::msg::KeyboardInput>(
      config_.topic, rclcpp::SensorDataQoS());
    mode_status_subscription_ =
      create_subscription<ai_sapiens_interfaces::msg::ModeStatus>(
      config_.mode_status_topic, 10,
      [this](const ai_sapiens_interfaces::msg::ModeStatus::SharedPtr message) {
        ui_status_.mode_status_received = true;
        mode_status_received_at_ = std::chrono::steady_clock::now();
        ui_status_.teleop_input_valid = message->teleop_input_valid;
        ui_status_.active_mode = message->active_mode;
        ui_status_.authority = message->authority;
        ui_status_.transition_reason = message->last_transition_reason;
      });
    terminal_ = std::make_unique<TerminalGuard>();
    clear_screen_ = isatty(STDOUT_FILENO);
    const char * term = std::getenv("TERM");
    use_color_ =
      clear_screen_ &&
      std::getenv("NO_COLOR") == nullptr &&
      (term == nullptr || std::string(term) != "dumb");

    const auto publish_period =
      std::chrono::duration<double>(1.0 / config_.publish_rate);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      [this]() {publish_state();});
    keyboard_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {read_keyboard();});
    const auto render_period =
      std::chrono::duration<double>(1.0 / config_.ui_update_rate);
    render_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(render_period),
      [this]() {print_guide();});

    RCLCPP_INFO(
      get_logger(), "keyboard teleop: topic=%s rate=%.1f Hz",
      config_.topic.c_str(), config_.publish_rate);
    print_guide();
  }

private:
  void publish_state()
  {
    const bool had_pending_mimic_request = state_->mimic_request_pending();
    auto message = state_->take_message(++sequence_);
    message.header.stamp = now();
    message.header.frame_id = "keyboard";
    publisher_->publish(message);
    if (had_pending_mimic_request && !state_->mimic_request_pending()) {
      print_guide();
    }
  }

  void read_keyboard()
  {
    char buffer[64];
    while (true) {
      const ssize_t count = read(STDIN_FILENO, buffer, sizeof(buffer));
      if (count > 0) {
        for (const auto action : decoder_.feed(
            std::string_view(buffer, static_cast<std::size_t>(count))))
        {
          if (state_->apply(action)) {
            last_action_ = KeyboardTeleopState::action_description(action);
            print_guide();
          }
        }
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      RCLCPP_ERROR(
        get_logger(), "keyboard read failed: %s", std::strerror(errno));
      rclcpp::shutdown();
      return;
    }
  }

  void print_guide() const
  {
    auto ui_status = ui_status_;
    if (ui_status.mode_status_received) {
      const auto status_age =
        std::chrono::steady_clock::now() - mode_status_received_at_;
      ui_status.mode_status_fresh =
        status_age <= std::chrono::duration<double>(config_.ui_stale_timeout);
    }
    if (clear_screen_) {
      std::cout << "\033[2J\033[H";
    }
    std::cout << state_->dashboard(last_action_, ui_status, use_color_) << std::flush;
  }

  KeyboardTeleopConfig config_;
  std::unique_ptr<KeyboardTeleopState> state_;
  KeyboardInputDecoder decoder_;
  uint32_t sequence_{0};
  bool clear_screen_{false};
  bool use_color_{false};
  std::string last_action_{"Started safely in Damping"};
  KeyboardTeleopUiStatus ui_status_;
  std::chrono::steady_clock::time_point mode_status_received_at_{};
  rclcpp::Publisher<ai_sapiens_interfaces::msg::KeyboardInput>::SharedPtr publisher_;
  rclcpp::Subscription<ai_sapiens_interfaces::msg::ModeStatus>::SharedPtr
    mode_status_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;
  rclcpp::TimerBase::SharedPtr render_timer_;
  std::unique_ptr<TerminalGuard> terminal_;
};

}  // namespace
}  // namespace ai_sapiens_sim2real

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ai_sapiens_sim2real::KeyboardTeleopNode>());
  } catch (const std::exception & error) {
    std::cerr << "Keyboard teleop error: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
