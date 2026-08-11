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

#ifndef AI_SAPIENS_SIM2REAL__CONTROLLERS__MODE_CONTROLLER_HPP_
#define AI_SAPIENS_SIM2REAL__CONTROLLERS__MODE_CONTROLLER_HPP_

#include <cmath>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/config/authority_config.hpp"
#include "ai_sapiens_sim2real/interfaces/controller_base.hpp"
#include "ai_sapiens_sim2real/mode_runtime/authority_runtime.hpp"
#include "ai_sapiens_sim2real/mode_runtime/mode_state_machine.hpp"
#include "ai_sapiens_sim2real/mode_runtime/realtime_request_gate.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_command.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

// Defined in root_config.hpp; referenced here only by const ref.
class RootConfig;

// ModeController is the per-tick "decide" stage of the control loop, over two
// orthogonal axes:
//   - Authority (authority_) : WHO commands -> Manual / ApiWarmup / Api
//   - Behavior state         : WHAT runs    -> a Damping / Posture / Policy behavior
//
// update() = decide() -> apply() -> run_active_mode(). decide() reads inputs and
// returns the whole Decision without writing; apply() performs every write it
// implies. decide() follows a fixed priority pipeline (first match wins):
//   failsafe -> startup gate -> authority -> explicit request
//   -> authority-owned request (Manual: teleop / Api: service) -> keep current
//
// Writes go only through the narrow ModeDecision/ModeRequests/BehaviorOutput
// pointers; the whole-state view (state_) is const, so this controller cannot
// modify sensor or operator-input blocks.
class ModeController : public ControllerBase
{
public:
  ModeController(
    rclcpp::Node::SharedPtr node,
    const RootConfig & root_config,
    SharedControlData * shared_data);

  void update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/);
  void reset() override;
  std::string get_name() const override;

  // Fatal-path only: overwrite the output block with a zero-stiffness damping
  // command without committing a state transition.
  void apply_emergency_damping();

  // Snapshot of the mode subsystem published by ModeRosInterface.
  struct ModeStatusSnapshot
  {
    std::string active_state;
    std::vector<std::string> available_states;
    const char * authority{"UNKNOWN"};
    bool teleop_input_valid{false};
    bool api_heartbeat_valid{false};
    bool api_request_available{false};
    const char * last_transition_reason{"initial"};
  };

  // Result of a mode request routed in from a service.
  struct ModeRequestResult
  {
    bool success{false};
    std::string message;
    std::string active_mode;
  };

  // ROS-facing surface consumed by ModeRosInterface. The request is validated
  // and enqueued here; the RT update() consumes it through the request gate.
  ModeRequestResult set_mode_by_name(const std::string & mode_name);
  std::vector<std::string> concrete_state_names() const;
  std::vector<std::string> available_service_state_names() const;
  ModeStatusSnapshot status_snapshot() const;
  void flush_transition_log();

private:
  // Used by the startup gate, the damping kill-switch failsafe, and the
  // API->manual release motion-intent check.
  static constexpr const char * kDampingConditionName = "DampingRequested";
  static constexpr const char * kStartupReadyPoseConditionName = "ReadyPoseRequested";
  static constexpr const char * kMimicConditionName = "MimicRequested";
  static constexpr const char * kManualMimicServiceConditionName =
    "ManualMimicServiceAllowed";

  enum class TransitionReason
  {
    Initial,
    Reset,
    KeepCurrentState,
    TeleopInputCondition,
    ServiceRequest,
    RequestedState,
    TeleopInputTimeout,
    TeleopInputDamping,
    DampingRequest,
    ActionLimit,
    BadOrientation,
    TeleopInputUnavailable,
    ApiAuthorityRequested,
    ApiWarmupComplete,
    ApiAuthorityLost,
    ApiAuthorityReleased,
  };

  enum class StartupGateStatus
  {
    Open,
    AcceptInput,
    Hold,
  };

  struct StateRequest
  {
    std::string name;
    BehaviorKind behavior_kind;
    TransitionReason reason{TransitionReason::KeepCurrentState};
  };

  // A state selection plus the request slots apply() must consume after using it.
  struct StateResolution
  {
    StateRequest request;
    bool clear_explicit_request{false};
    bool consume_service_request{false};
  };

  // An authority transition expressed as data, so it can be computed purely and
  // committed in one place. No set_to (and no other flags) means "no change".
  struct AuthorityChange
  {
    bool begin_warmup{false};
    std::optional<Authority> set_to;
    TransitionReason reason{TransitionReason::KeepCurrentState};
    std::optional<bool> set_request_edge;  // new was_teleop_api_mode_requested_
    bool discard_service{false};
    bool zero_velocity{false};
    std::optional<StateRequest> implied_state;
  };

  // The whole outcome of one tick's decision, computed by decide() and committed
  // by apply(). A failsafe tick sets only `failsafe` and `state`; a normal tick
  // also fills `authority` and `velocity`.
  struct Decision
  {
    StateResolution state;
    std::optional<TransitionReason> failsafe;
    std::optional<VelocitySource> velocity;
    AuthorityChange authority;
    bool accept_startup_input{false};
    bool log_startup_input_wait{false};
  };

  static StateRequest make_state_request_from_fsm(
    const StateEntryRequest & request,
    TransitionReason reason);
  void apply_state_request(const StateRequest & requested);
  void run_active_mode();
  static const char * transition_reason_name(TransitionReason reason);
  static bool is_failsafe_transition_reason(TransitionReason reason);

  // Startup wiring: load FSM/authority config, enter the initial state.
  void initialize_joint_counts();
  void configure_mode_runtime(const RootConfig & root_config);
  void enter_initial_state();

  bool is_api_request_available() const;
  bool is_service_request_executable(const StateRequest & request) const;
  bool is_service_request_allowed(
    Authority authority, const StateRequest & request) const;
  bool is_manual_mimic_service_allowed() const;

  // State/behavior lookup helpers over the configured FSM.
  std::string current_state_name() const;
  Authority current_authority() const
  {
    return authority_.read_snapshot();
  }

  std::vector<float> make_controller_ordered_joint_positions() const;
  void copy_controller_positions_to_output(const std::vector<float> & controller_values);

  const StateBehavior & behavior_for_state(const std::string & state_name) const;
  std::string policy_for_state(const std::string & state_name) const;
  BehaviorKind behavior_kind_for_state(const std::string & state_name) const;

  std::optional<StateRequest> resolve_state_request_from_name(
    const std::string & state_name) const;

  Decision decide();
  void apply(const Decision & decision);
  StartupGateStatus startup_gate_status() const;
  Decision make_startup_hold_decision() const;
  void log_startup_input_wait() const;
  StateResolution resolve_next_state(
    Authority authority,
    const AuthorityChange & authority_change);
  bool is_startup_teleop_input_safe(const ModeFsmTeleopInput & current_teleop_input) const;
  StateResolution resolve_authority_owned_state(Authority authority);
  StateRequest make_keep_current_state_request() const;
  StateRequest resolve_state_request_from_transitions() const;
  ModeFsmTeleopInput make_current_teleop_input() const;

  // Failsafe runs first each tick. failsafe_reason() purely detects the
  // highest-priority emergency; apply_failsafe() performs the flag clears and
  // authority/discard effects before the forced transition to Damping.
  std::optional<TransitionReason> failsafe_reason() const;
  void apply_failsafe(TransitionReason reason);
  bool is_teleop_input_unavailable() const;
  bool is_teleop_damping_requested() const;
  bool is_policy_action_limit_exceeded() const
  {
    return current_behavior_kind_ == BehaviorKind::Policy && requests_->action_limit_exceeded;
  }

  bool is_policy_orientation_unsafe() const
  {
    return current_behavior_kind_ == BehaviorKind::Policy && is_orientation_unsafe();
  }

  StateRequest make_damping_state_request(TransitionReason reason) const
  {
    return {
      mode_state_machine_.initial_state_name(),
      behavior_kind_for_state(mode_state_machine_.initial_state_name()),
      reason};
  }

  void clear_pending_service_request();

  VelocitySource select_velocity_source(Authority authority, bool zero_velocity) const;
  void apply_velocity_source(VelocitySource source);
  void set_authority(Authority mode, TransitionReason reason);
  static const char * authority_name(Authority mode);

  // Authority axis as pure computation plus an explicit commit. The compute_*
  // helpers only read state and return the transition; apply_authority_change()
  // performs every authority write (warmup, set_authority, request-edge flag,
  // service discard, zero-velocity handoff).
  AuthorityChange compute_authority_change() const;
  AuthorityChange compute_warmup_entry_change() const;     // Manual: API switch rising edge
  AuthorityChange compute_api_authority_change() const;
  AuthorityChange compute_api_release_change() const;       // API: switch released
  AuthorityChange compute_api_loss_change() const;          // API: heartbeat lost
  void log_api_authority_rejected(const char * rejection_reason) const;
  void log_api_manual_release_blocked() const;
  AuthorityChange manual_authority_change(TransitionReason reason) const;
  void apply_authority_change(const AuthorityChange & change);
  std::optional<StateRequest> resolve_teleop_handoff_request(
    TransitionReason reason, bool allow_mimic_target) const;
  StateRequest make_velocity_state_request(TransitionReason reason) const;
  bool can_current_state_enter_api() const;
  bool is_api_to_manual_release_blocked() const;
  bool is_teleop_input_available() const;
  bool is_api_heartbeat_valid() const
  {
    return authority_.is_api_heartbeat_valid(*state_);
  }

  bool is_api_warmup_finished() const
  {
    return authority_.is_warmup_finished();
  }

  bool is_mimic_state() const;
  bool is_orientation_unsafe() const;
  bool is_transition_allowed(const StateRequest & request) const;
  bool is_transition_allowed(BehaviorKind from, BehaviorKind to) const;

  // State entry/run paths update SharedControlData without blocking the realtime loop.
  void enter_state(const StateRequest & request);
  void commit_state_transition(const StateRequest & request);
  void enter_requested_mode(const StateRequest & request);
  void enter_damping_state(const StateRequest & request);
  void enter_posture_state(const StateRequest & request);
  void enter_policy_state(const StateRequest & request);
  void record_transition_reason(TransitionReason reason);
  void mark_transition_log_pending();

  void run_damping();
  void run_posture();

  rclcpp::Node::SharedPtr node_;
  // Read-only whole-state view; writes go only through the block pointers below.
  const SharedControlData * state_;
  ModeDecision * mode_;
  ModeRequests * requests_;
  BehaviorOutput * output_;

  ModeStateMachine mode_state_machine_;
  AuthorityConfig authority_config_;
  AuthorityRuntime authority_;
  RealtimeRequestGate<StateRequest> service_request_gate_;

  // Active state snapshot read by status publishers and service callbacks.
  std::atomic<const char *> last_transition_reason_{"initial"};
  std::atomic<int> last_transition_reason_code_{static_cast<int>(TransitionReason::Initial)};
  std::atomic<const ModeState *> active_state_snapshot_{nullptr};
  std::atomic<uint64_t> pending_log_transition_count_{0};
  uint64_t logged_transition_count_{0};
  size_t controller_joint_count_{0};
  BehaviorKind current_behavior_kind_{BehaviorKind::Damping};
  std::chrono::steady_clock::time_point state_enter_time_{};
  bool startup_teleop_input_accepted_{false};
  // API entry fires only on a rising edge of the teleop API request, so entry
  // after boot, a failed attempt, or any manual takeover needs a fresh switch
  // toggle. Starts true so an already-high switch cannot enter API at boot.
  bool was_teleop_api_mode_requested_{true};
  // Last tick's teleop input; edge transitions fire only when the
  // condition is newly satisfied vs this sample. Starts unavailable so a
  // switch already held at boot cannot trigger.
  ModeFsmTeleopInput previous_teleop_input_{};

  // Posture interpolation buffers are pre-sized and reused during posture mode.
  double active_posture_duration_{3.0};
  std::chrono::steady_clock::time_point posture_start_time_{};
  std::vector<float> posture_start_controller_;
  std::vector<float> active_posture_target_;
  std::vector<float> posture_target_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONTROLLERS__MODE_CONTROLLER_HPP_
