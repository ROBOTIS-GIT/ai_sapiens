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

#ifndef AI_SAPIENS_SIM2REAL__TELEOP_INPUT__TELEOP_INPUT_COMMAND_HPP_
#define AI_SAPIENS_SIM2REAL__TELEOP_INPUT__TELEOP_INPUT_COMMAND_HPP_

#include <array>
#include <chrono>
#include <cstdint>

#include <Eigen/Dense>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

struct TeleopInputCommand
{
  // Device-neutral manual control input produced by any teleop input plugin.
  bool api_mode{false};
  uint16_t input_code{0};
  uint16_t selector_code{0};
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};
  // 2026-09-30 carry port (from carry_ws sim2sim runtime): raw RC pulse widths (index = channel).
  std::array<uint16_t, 17> carry_rc_us{};
  std::array<bool, 17> carry_rc_valid{};
  std::chrono::steady_clock::time_point received_at{};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__TELEOP_INPUT__TELEOP_INPUT_COMMAND_HPP_
