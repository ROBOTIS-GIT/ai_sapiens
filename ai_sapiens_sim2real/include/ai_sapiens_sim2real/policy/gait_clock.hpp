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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__GAIT_CLOCK_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__GAIT_CLOCK_HPP_

#include <functional>
#include <optional>
#include <utility>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

struct SharedControlData;

/**
 * @brief The complete per-policy-step gait-phase rule.
 *
 * Given the current phase and the robot/command state, returns the next phase.
 * Rules stay pure -- they read the state passed to them and capture only their
 * own config -- while the clock supplies the state, so a new behaviour
 * (free-running, freezing on a standing command, resetting to zero,
 * stability-gated, anything not yet imagined) is just one of these, added
 * without touching GaitClock.
 */
using GaitPhaseUpdate = std::function<float(float phase, const SharedControlData & shared)>;

/**
 * @brief Bounded gait-phase clock.
 *
 * The phase value lives in shared policy state so the gait_phase observation
 * stays a pure read; this owns only the update rule. The state the rule reads is
 * passed in per step rather than held, so the dependency is explicit and the
 * clock is not a state owner. Phase stays in [0, 1).
 */
class GaitClock
{
public:
  explicit GaitClock(GaitPhaseUpdate update)
  : update_(std::move(update)) {}

  /// Reset @p phase to its episode-start value.
  void reset(float & phase) const {phase = 0.0f;}

  /// Advance @p phase by one policy step, given the current robot state.
  void advance(float & phase, const SharedControlData & shared) const;

private:
  GaitPhaseUpdate update_;
};

/**
 * @brief Build a clock from the policy's observation config.
 *
 * Owns the gait_phase param schema. Returns nullopt when the policy has no
 * gait_phase observation. Throws if gait_phase is present but misconfigured.
 */
std::optional<GaitClock> make_gait_clock(const YAML::Node & observations, double step_dt);

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__GAIT_CLOCK_HPP_
