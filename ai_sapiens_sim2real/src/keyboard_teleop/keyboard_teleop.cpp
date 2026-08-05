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

#include "ai_sapiens_sim2real/keyboard_teleop/keyboard_teleop.hpp"

#include <algorithm>
#include <cctype>
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

constexpr uint8_t kMimicRequestSampleCount = 2;
constexpr int kAxisBarHalfWidth = 10;

constexpr const char * kAnsiReset = "\033[0m";
constexpr const char * kAnsiBoldCyan = "\033[1;36m";
constexpr const char * kAnsiBoldGreen = "\033[1;32m";
constexpr const char * kAnsiBoldYellow = "\033[1;33m";
constexpr const char * kAnsiBoldMagenta = "\033[1;35m";
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

uint16_t read_input_code(
  const YAML::Node & input_codes, const std::string & name, uint16_t fallback)
{
  const auto code_node = input_codes ? input_codes[name] : YAML::Node();
  const uint16_t code = code_node ? code_node.as<uint16_t>() : fallback;
  if (code == 0) {
    throw std::runtime_error("input_code." + name + " must be non-zero");
  }
  return code;
}

KeyboardAction decode_character(unsigned char character)
{
  switch (std::tolower(character)) {
    case '1':
      return KeyboardAction::kDamping;
    case '2':
      return KeyboardAction::kReadyPose;
    case '3':
      return KeyboardAction::kVelocity;
    case '4':
      return KeyboardAction::kMimic;
    case 'w':
      return KeyboardAction::kForward;
    case 's':
      return KeyboardAction::kBackward;
    case 'a':
      return KeyboardAction::kLeft;
    case 'd':
      return KeyboardAction::kRight;
    case 'q':
      return KeyboardAction::kYawLeft;
    case 'e':
      return KeyboardAction::kYawRight;
    case ' ':
      return KeyboardAction::kStop;
    case 'p':
      return KeyboardAction::kToggleApi;
    case 'h':
      return KeyboardAction::kHelp;
    default:
      return KeyboardAction::kUnknown;
  }
}

}  // namespace

KeyboardTeleopConfig KeyboardTeleopConfig::from_yaml(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error("keyboard teleop config must be a map");
  }

  KeyboardTeleopConfig config;
  if (node["topic"]) {
    config.topic = node["topic"].as<std::string>();
  }
  if (config.topic.empty()) {
    throw std::runtime_error("keyboard teleop topic must not be empty");
  }

  if (node["publish_rate"]) {
    config.publish_rate = node["publish_rate"].as<double>();
  }
  if (!std::isfinite(config.publish_rate) || config.publish_rate <= 0.0) {
    throw std::runtime_error("keyboard teleop publish_rate must be positive");
  }

  if (node["velocity_step"]) {
    config.velocity_step = node["velocity_step"].as<float>();
  }
  if (!std::isfinite(config.velocity_step) ||
    config.velocity_step <= 0.0F || config.velocity_step > 1.0F)
  {
    throw std::runtime_error("keyboard teleop velocity_step must be in (0, 1]");
  }

  const auto input_codes = node["input_code"];
  config.damping_code = read_input_code(input_codes, "damping", config.damping_code);
  config.ready_pose_code =
    read_input_code(input_codes, "ready_pose", config.ready_pose_code);
  config.velocity_code = read_input_code(input_codes, "velocity", config.velocity_code);
  config.mimic_code = read_input_code(input_codes, "mimic", config.mimic_code);
  const std::set<uint16_t> unique_input_codes{
    config.damping_code,
    config.ready_pose_code,
    config.velocity_code,
    config.mimic_code};
  if (unique_input_codes.size() != 4) {
    throw std::runtime_error("keyboard teleop input codes must be unique");
  }

  const auto navigation = node["selector_navigation"];
  if (!navigation || !navigation["options"]) {
    throw std::runtime_error("keyboard teleop selector_navigation.options is required");
  }
  const auto options = navigation["options"];
  if (!options.IsSequence() || options.size() == 0) {
    throw std::runtime_error(
            "keyboard teleop selector_navigation.options must be a non-empty sequence");
  }

  std::set<uint16_t> selector_codes;
  for (const auto & option : options) {
    if (!option["code"] || !option["label"]) {
      throw std::runtime_error("keyboard selector options require code and label");
    }
    KeyboardSelectorOption parsed;
    parsed.code = option["code"].as<uint16_t>();
    parsed.label = option["label"].as<std::string>();
    if (parsed.code == 0 || parsed.label.empty()) {
      throw std::runtime_error("keyboard selector option code and label must be non-empty");
    }
    if (!selector_codes.insert(parsed.code).second) {
      throw std::runtime_error("keyboard selector option codes must be unique");
    }
    config.selector_options.push_back(std::move(parsed));
  }

  const uint16_t initial_code =
    navigation["initial_code"] ?
    navigation["initial_code"].as<uint16_t>() :
    config.selector_options.front().code;
  const auto initial = std::find_if(
    config.selector_options.begin(), config.selector_options.end(),
    [initial_code](const KeyboardSelectorOption & option) {
      return option.code == initial_code;
    });
  if (initial == config.selector_options.end()) {
    throw std::runtime_error("keyboard selector initial_code is not in options");
  }
  config.initial_selector_index =
    static_cast<std::size_t>(std::distance(config.selector_options.begin(), initial));

  return config;
}

std::vector<KeyboardAction> KeyboardInputDecoder::feed(std::string_view bytes)
{
  pending_.append(bytes.data(), bytes.size());
  std::vector<KeyboardAction> actions;
  while (!pending_.empty()) {
    const auto first = static_cast<unsigned char>(pending_.front());
    if (first != 0x1bU) {
      const auto action = decode_character(first);
      if (action != KeyboardAction::kUnknown) {
        actions.push_back(action);
      }
      pending_.erase(0, 1);
      continue;
    }

    if (pending_.size() < 2) {
      break;
    }
    if (pending_[1] != '[' && pending_[1] != 'O') {
      pending_.erase(0, 1);
      continue;
    }
    if (pending_.size() < 3) {
      break;
    }

    if (pending_[2] == 'D') {
      actions.push_back(KeyboardAction::kPreviousSelector);
    } else if (pending_[2] == 'C') {
      actions.push_back(KeyboardAction::kNextSelector);
    }
    pending_.erase(0, 3);
  }
  return actions;
}

KeyboardTeleopState::KeyboardTeleopState(KeyboardTeleopConfig config)
: config_(std::move(config)),
  input_code_(config_.damping_code),
  input_label_("Damping"),
  selector_index_(config_.initial_selector_index)
{
  if (config_.selector_options.empty() ||
    selector_index_ >= config_.selector_options.size())
  {
    throw std::runtime_error("keyboard teleop state requires a valid selector option");
  }
}

bool KeyboardTeleopState::apply(KeyboardAction action)
{
  switch (action) {
    case KeyboardAction::kDamping:
      set_mode(config_.damping_code, "Damping");
      return true;
    case KeyboardAction::kReadyPose:
      set_mode(config_.ready_pose_code, "ReadyPose");
      return true;
    case KeyboardAction::kVelocity:
      set_mode(config_.velocity_code, "Velocity");
      return true;
    case KeyboardAction::kMimic:
      queue_mimic_request();
      return true;
    case KeyboardAction::kForward:
      add_clamped(linear_x_, config_.velocity_step);
      return true;
    case KeyboardAction::kBackward:
      add_clamped(linear_x_, -config_.velocity_step);
      return true;
    case KeyboardAction::kLeft:
      add_clamped(linear_y_, config_.velocity_step);
      return true;
    case KeyboardAction::kRight:
      add_clamped(linear_y_, -config_.velocity_step);
      return true;
    case KeyboardAction::kYawLeft:
      add_clamped(angular_z_, config_.velocity_step);
      return true;
    case KeyboardAction::kYawRight:
      add_clamped(angular_z_, -config_.velocity_step);
      return true;
    case KeyboardAction::kStop:
      clear_velocity();
      return true;
    case KeyboardAction::kPreviousSelector:
      select_previous();
      return true;
    case KeyboardAction::kNextSelector:
      select_next();
      return true;
    case KeyboardAction::kToggleApi:
      mimic_request_samples_remaining_ = 0;
      api_mode_ = !api_mode_;
      if (api_mode_) {
        clear_velocity();
      }
      return true;
    case KeyboardAction::kHelp:
      return true;
    case KeyboardAction::kUnknown:
    default:
      return false;
  }
}

ai_sapiens_interfaces::msg::KeyboardInput KeyboardTeleopState::take_message(
  uint32_t sequence)
{
  ai_sapiens_interfaces::msg::KeyboardInput message;
  message.sequence = sequence;
  message.api_mode = api_mode_;
  if (mimic_request_pending()) {
    message.input_code = config_.mimic_code;
    --mimic_request_samples_remaining_;
  } else {
    message.input_code = input_code_;
  }
  message.selector_code = config_.selector_options[selector_index_].code;
  message.linear_x = linear_x_;
  message.linear_y = linear_y_;
  message.angular_z = angular_z_;
  return message;
}

bool KeyboardTeleopState::mimic_request_pending() const
{
  return mimic_request_samples_remaining_ > 0;
}

std::string KeyboardTeleopState::status_line() const
{
  const auto & selector = config_.selector_options[selector_index_];
  const uint16_t displayed_input_code =
    mimic_request_pending() ? config_.mimic_code : input_code_;
  const std::string displayed_input_label =
    mimic_request_pending() ? "Mimic" : input_label_;
  std::ostringstream status;
  status << "authority=" << (api_mode_ ? "API" : "MANUAL")
         << " | request=" << displayed_input_label << '(' << displayed_input_code << ')'
         << " | selector=[" << selector_index_ + 1 << '/'
         << config_.selector_options.size() << "] "
         << selector.label << '(' << selector.code << ')'
         << " | velocity=(" << std::fixed << std::setprecision(1)
         << linear_x_ << ", " << linear_y_ << ", " << angular_z_ << ')';
  return status.str();
}

std::string KeyboardTeleopState::dashboard(
  std::string_view last_action, bool use_color) const
{
  const auto & selector = config_.selector_options[selector_index_];
  const uint16_t displayed_input_code =
    mimic_request_pending() ? config_.mimic_code : input_code_;
  const std::string displayed_input_label =
    mimic_request_pending() ? "Mimic" : input_label_;

  const char * request_color = kAnsiBoldYellow;
  if (displayed_input_code == config_.damping_code) {
    request_color = kAnsiBoldMagenta;
  } else if (displayed_input_code == config_.velocity_code) {
    request_color = kAnsiBoldGreen;
  }

  std::ostringstream request;
  request << displayed_input_label << " (" << displayed_input_code << ')';
  if (mimic_request_pending()) {
    request << "  sending trigger";
  } else if (displayed_input_code == 0) {
    request << "  ready for next command";
  }

  std::ostringstream motion;
  motion << '[' << selector_index_ + 1 << '/' << config_.selector_options.size()
         << "]  " << selector.label << " (" << selector.code << ')';

  std::ostringstream dashboard;
  dashboard
    << paint("AI SAPIENS  /  KEYBOARD TELEOP", kAnsiBoldCyan, use_color) << '\n'
    << "======================================================================\n"
    << paint("CONTROL STATE", kAnsiBoldCyan, use_color) << '\n'
    << "  Authority    "
    << paint(
      api_mode_ ? "API" : "MANUAL",
      api_mode_ ? kAnsiBoldCyan : kAnsiBoldGreen, use_color) << '\n'
    << "  Request      "
    << paint(request.str(), request_color, use_color) << '\n'
    << "  Motion       <  "
    << paint(motion.str(), kAnsiBoldYellow, use_color) << "  >\n"
    << '\n'
    << paint("VELOCITY  (normalized -1.0 ... +1.0)", kAnsiBoldCyan, use_color) << '\n'
    << "  X    back  " << format_axis(linear_x_, false) << "  forward\n"
    << "  Y    left  " << format_axis(linear_y_, true) << "  right\n"
    << "  Yaw  left  " << format_axis(angular_z_, true) << "  right\n"
    << '\n'
    << paint("KEYS", kAnsiBoldCyan, use_color) << '\n'
    << key_map() << '\n'
    << "----------------------------------------------------------------------\n"
    << "  Last input   "
    << paint(
      last_action.empty() ? "Waiting for a key..." : std::string(last_action),
      kAnsiDim, use_color)
    << '\n';
  return dashboard.str();
}

std::string KeyboardTeleopState::action_description(KeyboardAction action)
{
  switch (action) {
    case KeyboardAction::kDamping:
      return "Request Damping";
    case KeyboardAction::kReadyPose:
      return "Request Ready Pose";
    case KeyboardAction::kVelocity:
      return "Request Velocity";
    case KeyboardAction::kMimic:
      return "Run selected motion";
    case KeyboardAction::kForward:
      return "Increase forward velocity";
    case KeyboardAction::kBackward:
      return "Increase backward velocity";
    case KeyboardAction::kLeft:
      return "Increase left velocity";
    case KeyboardAction::kRight:
      return "Increase right velocity";
    case KeyboardAction::kYawLeft:
      return "Increase left yaw";
    case KeyboardAction::kYawRight:
      return "Increase right yaw";
    case KeyboardAction::kStop:
      return "Stop: velocity set to zero";
    case KeyboardAction::kPreviousSelector:
      return "Select previous motion";
    case KeyboardAction::kNextSelector:
      return "Select next motion";
    case KeyboardAction::kToggleApi:
      return "Toggle MANUAL / API authority";
    case KeyboardAction::kHelp:
      return "Refresh keyboard guide";
    case KeyboardAction::kUnknown:
    default:
      return "Unknown key ignored";
  }
}

std::string KeyboardTeleopState::key_map()
{
  return
    "  MODE      [1] Damping   [2] Ready Pose   [3] Velocity   [4] Run motion\n"
    "  MOVE      [W/S] Forward/back   [A/D] Left/right   [Q/E] Turn\n"
    "  MOTION    [Left/Right] Select motion     [Space] Stop velocity\n"
    "  SYSTEM    [P] Manual/API   [H] Refresh guide   [Ctrl-C] Quit";
}

void KeyboardTeleopState::select_previous()
{
  selector_index_ =
    selector_index_ == 0 ? config_.selector_options.size() - 1 : selector_index_ - 1;
}

void KeyboardTeleopState::select_next()
{
  selector_index_ = (selector_index_ + 1) % config_.selector_options.size();
}

void KeyboardTeleopState::clear_velocity()
{
  linear_x_ = 0.0F;
  linear_y_ = 0.0F;
  angular_z_ = 0.0F;
}

void KeyboardTeleopState::set_mode(uint16_t input_code, std::string label)
{
  mimic_request_samples_remaining_ = 0;
  api_mode_ = false;
  input_code_ = input_code;
  input_label_ = std::move(label);
  clear_velocity();
}

void KeyboardTeleopState::queue_mimic_request()
{
  api_mode_ = false;
  input_code_ = 0;
  input_label_ = "Neutral";
  mimic_request_samples_remaining_ = kMimicRequestSampleCount;
  clear_velocity();
}

void KeyboardTeleopState::add_clamped(float & value, float increment)
{
  value = std::clamp(value + increment, -1.0F, 1.0F);
  if (std::abs(value) < 1.0e-6F) {
    value = 0.0F;
  }
}

}  // namespace ai_sapiens_sim2real
