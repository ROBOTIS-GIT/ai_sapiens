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

#include <string>

namespace ai_sapiens_sim2real
{

struct OperatorCommandInputOptions
{
  // teleop input plugin drives manual mode and always acts as the primary watchdog.
  std::string teleop_input_plugin;
  std::string teleop_input_config_path;
  double teleop_input_timeout{0.2};

  // API heartbeat gates service/cmd_vel authority in API mode.
  std::string api_heartbeat_topic{"/ai_sapiens/api_heartbeat"};
  double api_heartbeat_timeout{0.2};

  std::string cmd_vel_topic{"/cmd_vel"};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__OPERATOR_COMMAND_INPUT_OPTIONS_HPP_
