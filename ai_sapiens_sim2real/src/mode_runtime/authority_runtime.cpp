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

void AuthorityPolicy::configure(const AuthorityConfig & config, bool require_api_heartbeat)
{
  warmup_duration_ = std::chrono::duration<double>(
    config.api_entry_warmup_duration.value_or(kDefaultWarmupSeconds));
  switch_velocity_threshold_ = config.api_entry_velocity_neutral_threshold.value_or(0.05f);
  require_api_heartbeat_ = require_api_heartbeat;
}

std::chrono::duration<double> AuthorityPolicy::warmup_duration() const
{
  return warmup_duration_;
}

bool AuthorityPolicy::has_fresh_api_velocity(const ApiInput & api) const
{
  constexpr std::chrono::seconds kCmdVelTimeout{1};
  return api.velocity_received_at != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - api.velocity_received_at <= kCmdVelTimeout;
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
  if (!shared_data.teleop.is_velocity_command_valid) {
    return "teleop velocity command is unavailable";
  }
  if (!is_command_below_switch_threshold(shared_data.teleop.velocity_commands)) {
    return "teleop velocity command is not neutral";
  }
  if (has_fresh_api_velocity(shared_data.api) &&
    !is_command_below_switch_threshold(shared_data.api.velocity_commands))
  {
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

void AuthorityRuntime::configure(const AuthorityConfig & config, bool require_api_heartbeat)
{
  policy_.configure(config, require_api_heartbeat);
  teleop_overrides_cmd_vel_ = config.teleop_overrides_cmd_vel;
}

VelocitySource AuthorityRuntime::velocity_source(
  Authority next,
  const SharedControlData & data) const
{
  if (next == Authority::ApiWarmup) {
    return VelocitySource::Zero;
  }
  if (next == Authority::Manual || (teleop_overrides_cmd_vel_ && teleop_has_priority(data))) {
    return VelocitySource::Teleop;
  }
  const bool is_current_session_command = data.api.velocity_received_at >= session_started_at_;
  if (is_current_session_command && policy_.has_fresh_api_velocity(data.api)) {
    return VelocitySource::Api;
  }
  return VelocitySource::Zero;
}

bool AuthorityRuntime::teleop_has_priority(const SharedControlData & data) const
{
  if (!data.teleop.is_velocity_command_valid) {
    // The handle zeros invalid input. Keep that zero until the operator sends a valid release.
    return data.mode.velocity_source == VelocitySource::Teleop;
  }
  constexpr float kNormalizedStickDeadband = 0.05f;
  return data.teleop.velocity_command_normalized.cwiseAbs().maxCoeff() > kNormalizedStickDeadband;
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
  session_started_at_ = std::chrono::steady_clock::now();
  warmup_deadline_ = session_started_at_ +
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
