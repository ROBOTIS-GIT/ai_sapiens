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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__STARTUP_GATE_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__STARTUP_GATE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"

namespace ai_sapiens_sim2real
{

// Startup guard: do not expose mode services before the teleop input plugin is alive.
bool wait_for_teleop_input_sample(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<TeleopInputPluginBase> & plugin,
  double timeout_seconds);

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__STARTUP_GATE_HPP_
