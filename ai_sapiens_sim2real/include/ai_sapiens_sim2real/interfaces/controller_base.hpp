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
// Author: Woojin Wie

#ifndef AI_SAPIENS_SIM2REAL__INTERFACES__CONTROLLER_BASE_HPP_
#define AI_SAPIENS_SIM2REAL__INTERFACES__CONTROLLER_BASE_HPP_

#include <string>

namespace ai_sapiens_sim2real
{

/**
 * @brief Base interface for a named, resettable control component.
 *
 * The two stages of the control step (ModeController = decide,
 * PolicyController = act) are reset and named through this interface during
 * startup. Their per-tick update() is intentionally not declared here: each
 * stage consumes different inputs, so its update signature documents that
 * (the act stage takes the decide stage's ModeDecision as an argument) and is
 * always called through the concrete type, never polymorphically.
 *
 * Similar to ros2_control's ControllerInterface pattern.
 */
class ControllerBase
{
public:
  virtual ~ControllerBase() = default;

  /**
   * @brief Get the name of this controller.
   * @return Controller name for logging/debugging
   */
  virtual std::string get_name() const = 0;

  /**
   * @brief Reset controller state.
   *
   * Called when entering active state or on reset request.
   */
  virtual void reset() {}
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__INTERFACES__CONTROLLER_BASE_HPP_
