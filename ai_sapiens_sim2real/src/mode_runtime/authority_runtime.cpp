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

#include "ai_sapiens_sim2real/mode_runtime/authority_runtime.hpp"

namespace ai_sapiens_sim2real
{

void AuthorityPolicy::set_warmup_duration(double seconds)
{
  warmup_duration_ = std::chrono::duration<double>(seconds);
}

void AuthorityPolicy::set_switch_velocity_threshold(float threshold)
{
  switch_velocity_threshold_ = threshold;
}

std::chrono::duration<double> AuthorityPolicy::warmup_duration() const
{
  return warmup_duration_;
}

bool AuthorityPolicy::can_accept_api_velocity(Authority mode) const
{
  return mode == Authority::Api;
}

bool AuthorityPolicy::is_api_heartbeat_valid(const SharedControlData & shared_data) const
{
  return shared_data.api.heartbeat_received.load(std::memory_order_acquire) &&
         !shared_data.api.heartbeat_stale.load(std::memory_order_acquire);
}

const char * AuthorityPolicy::api_entry_rejection_reason(
  const SharedControlData & shared_data,
  bool current_state_can_enter_api) const
{
  if (!is_api_session_valid(shared_data)) {
    return "api heartbeat is not valid";
  }
  if (!current_state_can_enter_api) {
    return "active mode cannot enter API authority";
  }
  if (!is_command_below_switch_threshold(shared_data.teleop.velocity_commands)) {
    return "teleop velocity command is not neutral";
  }
  if (!is_command_below_switch_threshold(shared_data.api.velocity_commands)) {
    return "api velocity command is not neutral";
  }

  return nullptr;
}

bool AuthorityPolicy::is_command_below_switch_threshold(const Eigen::Vector3f & command) const
{
  return std::abs(command.x()) <= switch_velocity_threshold_ &&
         std::abs(command.y()) <= switch_velocity_threshold_ &&
         std::abs(command.z()) <= switch_velocity_threshold_;
}

void AuthorityRuntime::set_warmup_duration(double seconds)
{
  policy_.set_warmup_duration(seconds);
}

void AuthorityRuntime::set_switch_velocity_threshold(float threshold)
{
  policy_.set_switch_velocity_threshold(threshold);
}

Authority AuthorityRuntime::read_snapshot() const
{
  return static_cast<Authority>(snapshot_.load(std::memory_order_acquire));
}

void AuthorityRuntime::set_current(Authority mode, ModeDecision & decision)
{
  current_ = mode;
  decision.authority = mode;
  snapshot_.store(static_cast<int>(mode), std::memory_order_release);
}

void AuthorityRuntime::begin_warmup()
{
  warmup_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(policy_.warmup_duration());
}

bool AuthorityRuntime::is_warmup_finished() const
{
  return current_ == Authority::ApiWarmup &&
         std::chrono::steady_clock::now() >= warmup_deadline_;
}

const char * AuthorityRuntime::api_entry_rejection_reason(
  const SharedControlData & shared_data,
  bool current_state_can_enter_api) const
{
  return policy_.api_entry_rejection_reason(shared_data, current_state_can_enter_api);
}


}  // namespace ai_sapiens_sim2real
