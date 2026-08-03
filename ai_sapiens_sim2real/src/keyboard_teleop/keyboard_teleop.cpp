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
      set_mode(config_.mimic_code, "Mimic");
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

ai_sapiens_interfaces::msg::KeyboardInput KeyboardTeleopState::make_message(
  uint32_t sequence) const
{
  ai_sapiens_interfaces::msg::KeyboardInput message;
  message.sequence = sequence;
  message.api_mode = api_mode_;
  message.input_code = input_code_;
  message.selector_code = config_.selector_options[selector_index_].code;
  message.linear_x = linear_x_;
  message.linear_y = linear_y_;
  message.angular_z = angular_z_;
  return message;
}

std::string KeyboardTeleopState::status_line() const
{
  const auto & selector = config_.selector_options[selector_index_];
  std::ostringstream status;
  status << "authority=" << (api_mode_ ? "API" : "MANUAL")
         << " | request=" << input_label_ << '(' << input_code_ << ')'
         << " | selector=[" << selector_index_ + 1 << '/'
         << config_.selector_options.size() << "] "
         << selector.label << '(' << selector.code << ')'
         << " | velocity=(" << std::fixed << std::setprecision(1)
         << linear_x_ << ", " << linear_y_ << ", " << angular_z_ << ')';
  return status.str();
}

std::string KeyboardTeleopState::key_map()
{
  return
    "1:Damping | 2:ReadyPose | 3:Velocity | 4:run selected mimic | "
    "W/S:forward/back | A/D:left/right | Q/E:yaw | Space:stop | "
    "Left/Right:select mimic | P:API/MANUAL | H:help | Ctrl-C:quit";
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
  api_mode_ = false;
  input_code_ = input_code;
  input_label_ = std::move(label);
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
