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

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/keyboard_teleop/keyboard_teleop.hpp"

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
    const auto default_config =
      ament_index_cpp::get_package_share_directory("ai_sapiens_sim2real") +
      "/config/teleop/keyboard.yaml";
    declare_parameter<std::string>("config_path", default_config);
    const auto config_path = get_parameter("config_path").as_string();
    if (config_path.empty()) {
      throw std::runtime_error("keyboard teleop config_path must not be empty");
    }

    config_ = KeyboardTeleopConfig::from_yaml(YAML::LoadFile(config_path));
    state_ = std::make_unique<KeyboardTeleopState>(config_);
    publisher_ = create_publisher<ai_sapiens_interfaces::msg::KeyboardInput>(
      config_.topic, rclcpp::SensorDataQoS());
    terminal_ = std::make_unique<TerminalGuard>();

    const auto publish_period =
      std::chrono::duration<double>(1.0 / config_.publish_rate);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      [this]() {publish_state();});
    keyboard_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {read_keyboard();});

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
    std::cout << "\033[2J\033[H"
              << "AI Sapiens keyboard teleop\n"
              << state_->status_line() << '\n'
              << KeyboardTeleopState::key_map() << '\n'
              << std::flush;
  }

  KeyboardTeleopConfig config_;
  std::unique_ptr<KeyboardTeleopState> state_;
  KeyboardInputDecoder decoder_;
  uint32_t sequence_{0};
  rclcpp::Publisher<ai_sapiens_interfaces::msg::KeyboardInput>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;
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
