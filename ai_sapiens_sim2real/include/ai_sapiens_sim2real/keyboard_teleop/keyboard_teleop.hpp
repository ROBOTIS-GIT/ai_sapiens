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

#ifndef AI_SAPIENS_SIM2REAL__KEYBOARD_TELEOP__KEYBOARD_TELEOP_HPP_
#define AI_SAPIENS_SIM2REAL__KEYBOARD_TELEOP__KEYBOARD_TELEOP_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <ai_sapiens_interfaces/msg/keyboard_input.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

enum class KeyboardAction
{
  kUnknown,
  kDamping,
  kReadyPose,
  kVelocity,
  kMimic,
  kForward,
  kBackward,
  kLeft,
  kRight,
  kYawLeft,
  kYawRight,
  kStop,
  kPreviousSelector,
  kNextSelector,
  kToggleApi,
  kHelp,
};

struct KeyboardSelectorOption
{
  uint16_t code{0};
  std::string label;
};

struct KeyboardTeleopConfig
{
  std::string topic{"/keyboard_teleop/input"};
  double publish_rate{20.0};
  float velocity_step{0.2F};
  uint16_t damping_code{1};
  uint16_t ready_pose_code{2};
  uint16_t velocity_code{3};
  uint16_t mimic_code{4};
  std::vector<KeyboardSelectorOption> selector_options;
  std::size_t initial_selector_index{0};

  static KeyboardTeleopConfig from_yaml(const YAML::Node & node);
};

class KeyboardInputDecoder
{
public:
  std::vector<KeyboardAction> feed(std::string_view bytes);

private:
  std::string pending_;
};

class KeyboardTeleopState
{
public:
  explicit KeyboardTeleopState(KeyboardTeleopConfig config);

  bool apply(KeyboardAction action);
  ai_sapiens_interfaces::msg::KeyboardInput make_message(uint32_t sequence) const;
  std::string status_line() const;

  static std::string key_map();

private:
  void select_previous();
  void select_next();
  void clear_velocity();
  void set_mode(uint16_t input_code, std::string label);
  void add_clamped(float & value, float increment);

  KeyboardTeleopConfig config_;
  bool api_mode_{false};
  uint16_t input_code_{0};
  std::string input_label_;
  std::size_t selector_index_{0};
  float linear_x_{0.0F};
  float linear_y_{0.0F};
  float angular_z_{0.0F};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__KEYBOARD_TELEOP__KEYBOARD_TELEOP_HPP_
