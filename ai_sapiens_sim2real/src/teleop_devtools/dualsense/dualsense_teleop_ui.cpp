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

#include "ai_sapiens_sim2real/teleop_devtools/dualsense/dualsense_teleop_ui.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ai_sapiens_sim2real
{
namespace
{

constexpr int kAxisBarHalfWidth = 10;
constexpr std::size_t kControlLabelWidth = 8;

constexpr const char * kAnsiReset = "\033[0m";
constexpr const char * kAnsiBoldCyan = "\033[1;36m";
constexpr const char * kAnsiBoldGreen = "\033[1;32m";
constexpr const char * kAnsiBoldYellow = "\033[1;33m";
constexpr const char * kAnsiBoldMagenta = "\033[1;35m";
constexpr const char * kAnsiBoldRed = "\033[1;31m";
constexpr const char * kAnsiDim = "\033[2m";

std::string paint(
  const std::string & value, const char * color, bool use_color)
{
  if (!use_color) {
    return value;
  }
  return std::string(color) + value + kAnsiReset;
}

std::string axis_bar(float value)
{
  const float clamped = std::clamp(value, -1.0F, 1.0F);
  const int marker = static_cast<int>(
    std::lround((clamped + 1.0F) * kAxisBarHalfWidth));
  std::string bar(static_cast<std::size_t>(2 * kAxisBarHalfWidth + 1), '-');
  bar[static_cast<std::size_t>(kAxisBarHalfWidth)] = '|';
  bar[static_cast<std::size_t>(marker)] = 'o';
  return '[' + bar + ']';
}

std::string format_axis(float value, bool reverse_bar)
{
  std::ostringstream output;
  output << axis_bar(reverse_bar ? -value : value)
         << "  " << std::showpos << std::fixed << std::setprecision(2) << value;
  return output.str();
}

std::string code_fallback(uint16_t code)
{
  return code == 0 ? "Idle" : "Code " + std::to_string(code);
}

std::string read_optional_label(const YAML::Node & item, uint16_t code)
{
  if (item["label"]) {
    const auto label = item["label"].as<std::string>();
    if (label.empty()) {
      throw std::runtime_error("DualSense input labels must not be empty");
    }
    return label;
  }

  switch (code) {
    case 1:
      return "Damping";
    case 2:
      return "ReadyPose";
    case 3:
      return "Velocity";
    case 4:
      return "Mimic";
    default:
      return code_fallback(code);
  }
}

std::string control_section(const std::string & label, bool use_color)
{
  const std::size_t padding = kControlLabelWidth > label.size() ?
    kControlLabelWidth - label.size() : 1;
  return "  " + paint(label, kAnsiBoldCyan, use_color) + std::string(padding, ' ');
}

}  // namespace

DualSenseTeleopUiConfig DualSenseTeleopUiConfig::from_yaml(
  const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error("DualSense teleop UI config must be a map");
  }

  DualSenseTeleopUiConfig config;
  const auto ui = node["ui"];
  if (ui) {
    if (ui["update_rate"]) {
      config.update_rate = ui["update_rate"].as<double>();
    }
    if (ui["stale_timeout"]) {
      config.stale_timeout = ui["stale_timeout"].as<double>();
    }
    if (ui["mode_status_topic"]) {
      config.mode_status_topic = ui["mode_status_topic"].as<std::string>();
    }
  }
  if (!std::isfinite(config.update_rate) || config.update_rate <= 0.0) {
    throw std::runtime_error("DualSense UI update_rate must be positive");
  }
  if (!std::isfinite(config.stale_timeout) || config.stale_timeout <= 0.0) {
    throw std::runtime_error("DualSense UI stale_timeout must be positive");
  }
  if (config.mode_status_topic.empty()) {
    throw std::runtime_error("DualSense UI mode_status_topic must not be empty");
  }

  const auto buttons = node["input_code"] ? node["input_code"]["buttons"] : YAML::Node();
  if (!buttons) {
    throw std::runtime_error("DualSense UI requires input_code.buttons");
  }
  if (buttons.IsSequence()) {
    for (const auto & item : buttons) {
      if (!item["code"]) {
        throw std::runtime_error("DualSense input button entries require code");
      }
      const auto code = item["code"].as<uint16_t>();
      if (code == 0 || !config.input_labels.emplace(
          code, read_optional_label(item, code)).second)
      {
        throw std::runtime_error("DualSense input button codes must be non-zero and unique");
      }
    }
  } else if (buttons.IsMap()) {
    for (const auto & item : buttons) {
      const auto code = item.second.as<uint16_t>();
      if (code == 0 || !config.input_labels.emplace(code, code_fallback(code)).second) {
        throw std::runtime_error("DualSense input button codes must be non-zero and unique");
      }
    }
  } else {
    throw std::runtime_error("DualSense input_code.buttons must be a sequence or map");
  }

  const auto navigation = node["selector_navigation"];
  const auto options = navigation ? navigation["options"] : YAML::Node();
  if (!options || !options.IsSequence() || options.size() == 0) {
    throw std::runtime_error(
            "DualSense UI requires non-empty selector_navigation.options");
  }
  std::set<uint16_t> selector_codes;
  for (const auto & option : options) {
    DualSenseSelectorUiOption parsed;
    parsed.code = option.as<uint16_t>();
    if (parsed.code == 0 || !selector_codes.insert(parsed.code).second) {
      throw std::runtime_error(
              "DualSense selector option codes must be non-zero and unique");
    }
    config.selector_options.push_back(std::move(parsed));
  }

  return config;
}

DualSenseTeleopUi::DualSenseTeleopUi(DualSenseTeleopUiConfig config)
: config_(std::move(config))
{
  if (config_.input_labels.empty() || config_.selector_options.empty()) {
    throw std::runtime_error("DualSense teleop UI requires input and selector labels");
  }
}

std::string DualSenseTeleopUi::dashboard(
  const DualSenseTeleopUiState & state, bool use_color) const
{
  std::string joy_status = "WAITING";
  const char * joy_color = kAnsiBoldYellow;
  if (state.joy_received && state.joy_fresh) {
    joy_status = "CONNECTED";
    joy_color = kAnsiBoldGreen;
  } else if (state.joy_received) {
    joy_status = "STALE";
    joy_color = kAnsiBoldRed;
  }

  const std::string controller_status =
    !state.mode_status_received ? "WAITING" :
    !state.mode_status_fresh ? "STALE" :
    state.teleop_input_valid ? "READY" : "INPUT LOST";
  const char * controller_color =
    state.mode_status_received && state.mode_status_fresh &&
    state.teleop_input_valid ?
    kAnsiBoldGreen :
    state.mode_status_received ? kAnsiBoldRed : kAnsiBoldYellow;

  const std::string active_mode =
    state.mode_status_received && !state.active_mode.empty() ?
    state.active_mode : "--";
  const std::string authority =
    state.mode_status_received && !state.authority.empty() ?
    state.authority : "--";
  const std::string requested_authority =
    state.command.api_mode ? "API (PS held)" : "MANUAL";

  const auto request_label = input_label(state.command.input_code);
  std::ostringstream request;
  request << request_label;
  if (state.command.input_code != 0) {
    request << " (" << state.command.input_code << ')';
  }

  std::size_t selector_index = 0;
  const auto motion_label =
    selector_label(state.command.selector_code, &selector_index);
  std::ostringstream motion;
  if (state.command.selector_code == 0) {
    motion << "--";
  } else {
    motion << '[' << selector_index + 1 << '/' << config_.selector_options.size()
           << "]  " << motion_label << " (" << state.command.selector_code << ')';
  }

  std::ostringstream dashboard;
  dashboard
    << paint("AI SAPIENS  /  DUALSENSE TELEOP", kAnsiBoldCyan, use_color) << '\n'
    << "======================================================================\n"
    << paint("CONNECTION", kAnsiBoldCyan, use_color) << '\n'
    << "  DualSense    " << paint(joy_status, joy_color, use_color) << '\n'
    << "  Controller   "
    << paint(controller_status, controller_color, use_color) << '\n'
    << '\n'
    << paint("CONTROL STATE", kAnsiBoldCyan, use_color) << '\n'
    << "  Active mode  " << paint(active_mode, kAnsiBoldGreen, use_color) << '\n'
    << "  Authority    " << paint(authority, kAnsiBoldCyan, use_color)
    << "  (requested: " << requested_authority << ")\n"
    << "  Request      " << paint(request.str(), kAnsiBoldMagenta, use_color) << '\n'
    << "  Motion       <  "
    << paint(motion.str(), kAnsiBoldYellow, use_color) << "  >\n"
    << '\n'
    << paint("VELOCITY  (normalized -1.0 ... +1.0)", kAnsiBoldCyan, use_color) << '\n'
    << "  X    back  " << format_axis(state.command.velocity.x(), false)
    << "  forward\n"
    << "  Y    left  " << format_axis(state.command.velocity.y(), true)
    << "  right\n"
    << "  Yaw  left  " << format_axis(state.command.velocity.z(), true)
    << "  right\n"
    << '\n'
    << paint("CONTROLS", kAnsiBoldCyan, use_color) << '\n'
    << controls(use_color)
    << "----------------------------------------------------------------------\n"
    << "  Last state   "
    << paint(
      state.transition_reason.empty() ?
      "Waiting for controller status..." : state.transition_reason,
      kAnsiDim, use_color)
    << '\n';
  return dashboard.str();
}

std::string DualSenseTeleopUi::input_label(uint16_t code) const
{
  const auto label = config_.input_labels.find(code);
  return label == config_.input_labels.end() ?
         code_fallback(code) : label->second;
}

std::string DualSenseTeleopUi::selector_label(
  uint16_t code, std::size_t * index) const
{
  const auto option = std::find_if(
    config_.selector_options.begin(), config_.selector_options.end(),
    [code](const DualSenseSelectorUiOption & candidate) {
      return candidate.code == code;
    });
  if (option == config_.selector_options.end()) {
    *index = 0;
    return code == 0 ? "--" : "Unknown";
  }
  *index = static_cast<std::size_t>(
    std::distance(config_.selector_options.begin(), option));
  return "Selector";
}

std::string DualSenseTeleopUi::controls(bool use_color) const
{
  std::ostringstream controls;
  controls
    << control_section("MODE", use_color)
    << paint("○", kAnsiBoldRed, use_color) << " Damping   "
    << paint("×", kAnsiBoldCyan, use_color) << " Ready pose   "
    << paint("△", kAnsiBoldGreen, use_color) << " Velocity   "
    << paint("□", kAnsiBoldMagenta, use_color) << " Mimic\n"
    << control_section("VELOCITY", use_color)
    << paint("Left stick", kAnsiBoldYellow, use_color) << " Linear X/Y   "
    << paint("Right stick", kAnsiBoldYellow, use_color) << " Yaw\n"
    << control_section("MIMIC", use_color)
    << paint("◀ ▶", kAnsiBoldYellow, use_color) << " Select motion   "
    << paint("SYSTEM", kAnsiBoldCyan, use_color) << "  "
    << paint("PS", kAnsiBoldCyan, use_color) << " API authority\n";
  return controls.str();
}

}  // namespace ai_sapiens_sim2real
