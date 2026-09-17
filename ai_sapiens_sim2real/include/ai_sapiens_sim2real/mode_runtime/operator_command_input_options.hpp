// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__OPERATOR_COMMAND_INPUT_OPTIONS_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__OPERATOR_COMMAND_INPUT_OPTIONS_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace ai_sapiens_sim2real
{

// Configuration used only by the group input adapter.
struct GroupCommandMapping
{
  uint16_t damping_code{1};
  uint16_t ready_code{2};
  uint16_t locomotion_code{3};
  uint16_t mimic_code{4};
  std::string locomotion_state{"Velocity"};
};

struct GroupInputOptions
{
  std::string plugin;
  std::string config_path;
  double timeout{0.2};
  double vel_command_timeout{0.2};
  GroupCommandMapping commands;
};

// Parsed startup settings passed from RootConfig to input-handle construction.
// Runtime participation and arbitration decisions live in SharedControlData.
struct OperatorCommandInputOptions
{
  // teleop input plugin drives manual mode and always acts as the primary watchdog.
  std::string teleop_input_plugin;
  std::string teleop_input_config_path;
  double teleop_input_timeout{0.2};
  double teleop_vel_command_timeout{0.2};

  std::optional<GroupInputOptions> group;

  // API heartbeat gates service/cmd_vel authority in API mode.
  std::string api_heartbeat_topic{"/ai_sapiens/api_heartbeat"};
  double api_heartbeat_timeout{0.2};

  std::string cmd_vel_topic{"/cmd_vel"};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__OPERATOR_COMMAND_INPUT_OPTIONS_HPP_
