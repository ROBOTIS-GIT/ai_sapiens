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

#ifndef AI_SAPIENS_SIM2REAL__DUALSENSE_TELEOP__DUALSENSE_TELEOP_UI_HPP_
#define AI_SAPIENS_SIM2REAL__DUALSENSE_TELEOP__DUALSENSE_TELEOP_UI_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/teleop_input/teleop_input_command.hpp"

namespace ai_sapiens_sim2real
{

struct DualSenseSelectorUiOption
{
  uint16_t code{0};
  std::string label;
};

struct DualSenseTeleopUiConfig
{
  double update_rate{10.0};
  double stale_timeout{0.5};
  std::string mode_status_topic{"/ai_sapiens/mode_status"};
  std::map<uint16_t, std::string> input_labels;
  std::vector<DualSenseSelectorUiOption> selector_options;

  static DualSenseTeleopUiConfig from_yaml(const YAML::Node & node);
};

struct DualSenseTeleopUiState
{
  bool joy_received{false};
  bool joy_fresh{false};
  bool mode_status_received{false};
  bool mode_status_fresh{false};
  bool teleop_input_valid{false};
  std::string active_mode;
  std::string authority;
  std::string transition_reason;
  TeleopInputCommand command;
};

class DualSenseTeleopUi
{
public:
  explicit DualSenseTeleopUi(DualSenseTeleopUiConfig config);

  std::string dashboard(const DualSenseTeleopUiState & state, bool use_color) const;

private:
  std::string input_label(uint16_t code) const;
  std::string selector_label(uint16_t code, std::size_t * index) const;
  std::string controls(bool use_color) const;

  DualSenseTeleopUiConfig config_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__DUALSENSE_TELEOP__DUALSENSE_TELEOP_UI_HPP_
