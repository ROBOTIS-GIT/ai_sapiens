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

// Raw per-episode state, owned by PolicyState rather than the clock/rule.
// Observations encode these values; the clock never produces an observation vector.
struct PhaseState
{
  float phase{0.0f};
  float period{0.0f};
  float swing_ratio{0.0f};
  bool first_step{true};
  float activation{1.0f};  // Optional command envelope; legacy clocks stay fully active.
};

// Same rule injection as main. The scalar phase is extended to raw state so a
// stateful rule can remain pure: config is captured, next state is returned.
using PhaseUpdate =
  std::function<PhaseState(PhaseState, const SharedControlData &)>;

class GaitClock
{
public:
  explicit GaitClock(
    PhaseUpdate update, PhaseState initial_state = {},
    bool resolve_command_before_update = false)
  : update_(std::move(update)), initial_state_(initial_state),
    resolve_command_before_update_(resolve_command_before_update) {}

  void reset(PhaseState & state) const {state = initial_state_;}
  void advance(PhaseState & state, const SharedControlData & shared) const;

  // Some exports require the current resolved command before clock update and
  // entry-history seeding. Others retain the original runtime ordering.
  bool resolve_command_before_update() const {return resolve_command_before_update_;}

private:
  PhaseUpdate update_;
  PhaseState initial_state_;
  bool resolve_command_before_update_;
};

// Config dispatch is the only place that selects a clock implementation.
std::optional<GaitClock> make_gait_clock(const YAML::Node & observations, double step_dt);

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__GAIT_CLOCK_HPP_
