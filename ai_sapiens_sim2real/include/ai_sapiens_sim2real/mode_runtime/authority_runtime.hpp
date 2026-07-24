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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__AUTHORITY_RUNTIME_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__AUTHORITY_RUNTIME_HPP_

#include <atomic>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/authority.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

class AuthorityPolicy
{
public:
  void set_warmup_duration(double seconds);
  void set_switch_velocity_threshold(float threshold);
  std::chrono::duration<double> warmup_duration() const;

  // Pure policy checks; no mutable runtime state.
  bool can_accept_api_velocity(Authority mode) const;
  bool is_api_heartbeat_valid(const SharedControlData & shared_data) const;
  bool is_api_session_valid(const SharedControlData & shared_data) const
  {
    return shared_data.teleop.api_mode_requested && is_api_heartbeat_valid(shared_data);
  }

  // Returns nullptr when API entry is allowed, otherwise a short rejection
  // reason for operator-facing logs.
  const char * api_entry_rejection_reason(
    const SharedControlData & shared_data,
    bool current_state_can_enter_api) const;

private:
  bool is_command_below_switch_threshold(const Eigen::Vector3f & command) const;

  std::chrono::duration<double> warmup_duration_{3.0};
  float switch_velocity_threshold_{0.05f};
};

class AuthorityRuntime
{
public:
  void set_warmup_duration(double seconds);
  void set_switch_velocity_threshold(float threshold);

  Authority current() const
  {
    return current_;
  }

  Authority read_snapshot() const;
  void set_current(Authority mode, ModeDecision & decision);

  // API authority has a short warmup window before external requests are accepted.
  void begin_warmup();
  bool is_warmup_finished() const;

  // Same check for a hypothetical authority value, so a tick can decide using
  // the authority it is about to transition to rather than the committed one.
  bool can_accept_api_velocity(Authority mode) const
  {
    return policy_.can_accept_api_velocity(mode);
  }

  bool is_api_heartbeat_valid(const SharedControlData & shared_data) const
  {
    return policy_.is_api_heartbeat_valid(shared_data);
  }

  const char * api_entry_rejection_reason(
    const SharedControlData & shared_data,
    bool current_state_can_enter_api) const;

private:
  // snapshot_ is read from non-RT status paths without taking a lock.
  Authority current_{Authority::Manual};
  std::atomic<int> snapshot_{static_cast<int>(Authority::Manual)};

  std::chrono::steady_clock::time_point warmup_deadline_{};
  AuthorityPolicy policy_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__AUTHORITY_RUNTIME_HPP_
