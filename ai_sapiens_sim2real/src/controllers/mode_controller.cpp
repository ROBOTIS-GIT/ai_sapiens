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

#include "ai_sapiens_sim2real/controllers/mode_controller.hpp"

#include "ai_sapiens_sim2real/config/root_config.hpp"

namespace ai_sapiens_sim2real
{

ModeController::ModeController(
  rclcpp::Node::SharedPtr node,
  const RootConfig & root_config,
  SharedControlData * shared_data)
: node_(node),
  state_(shared_data),
  mode_(&shared_data->mode),
  requests_(&shared_data->requests),
  output_(&shared_data->output)
{
  service_request_gate_.init();
  initialize_joint_counts();
  configure_mode_runtime(root_config);
  enter_initial_state();
}

void ModeController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const Decision decision = decide();
  apply(decision);
  run_active_mode();

  // Triggered transitions and edge checks compare against last tick's input;
  // advancing it unconditionally makes every edge last exactly one tick.
  previous_teleop_input_ = make_current_teleop_input();
}

void ModeController::reset()
{
  startup_teleop_input_accepted_ = false;
  was_teleop_api_mode_requested_ = true;
  previous_teleop_input_ = ModeFsmTeleopInput{};

  const StateRequest initial_state{
    mode_state_machine_.initial_state_name(),
    mode_state_machine_.behavior_kind_for_state(mode_state_machine_.initial_state_name()),
    TransitionReason::Reset};

  enter_state(initial_state);
}

std::string ModeController::get_name() const
{
  return "mode_controller";
}

ModeController::StateRequest ModeController::make_state_request_from_fsm(
  const StateEntryRequest & request,
  TransitionReason reason)
{
  return StateRequest{
    request.name,
    request.behavior_kind,
    reason};
}

void ModeController::apply_state_request(const StateRequest & requested)
{
  const bool already_in_requested_state =
    requested.behavior_kind == current_behavior_kind_ &&
    requested.name == mode_->active_state_name;

  if (already_in_requested_state) {
    return;
  }

  if (is_transition_allowed(requested)) {
    enter_state(requested);
    return;
  }

  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "[ModeController] rejected transition active(%s) requested(%s) reason(%s)",
    mode_->active_state_name.c_str(),
    requested.name.c_str(),
    transition_reason_name(requested.reason));
}

void ModeController::run_active_mode()
{
  switch (current_behavior_kind_) {
    case BehaviorKind::Damping:
      run_damping();
      break;
    case BehaviorKind::Posture:
      run_posture();
      break;
    case BehaviorKind::Policy:
      break;
  }
}

// Pure: the whole decision for this tick by priority. No member writes; every
// write the decision implies is performed by apply().
ModeController::Decision ModeController::decide()
{
  // 1. Failsafe overrides everything and forces Damping.
  if (const auto reason = failsafe_reason()) {
    Decision decision;
    decision.failsafe = *reason;
    decision.state.request = make_damping_state_request(*reason);
    return decision;
  }

  // 2. Startup gate: hold the current state until startup input is safe.
  const auto startup_gate = startup_gate_status();
  if (startup_gate == StartupGateStatus::Hold) {
    return make_startup_hold_decision();
  }

  // 3. Authority transition for this tick, as data (committed in apply()).
  Decision decision;
  decision.accept_startup_input = startup_gate == StartupGateStatus::AcceptInput;
  decision.authority = compute_authority_change();
  const Authority next_authority = decision.authority.set_to.value_or(authority_.current());

  // 4. Velocity source under the authority we are moving to.
  decision.velocity = select_velocity_source(next_authority, decision.authority.zero_velocity);

  // 5. Behavior state under that authority.
  decision.state = resolve_next_state(next_authority, decision.authority);
  return decision;
}

ModeController::StartupGateStatus ModeController::startup_gate_status() const
{
  if (startup_teleop_input_accepted_ || !is_teleop_input_available()) {
    return StartupGateStatus::Open;
  }

  if (is_startup_teleop_input_safe(make_current_teleop_input())) {
    return StartupGateStatus::AcceptInput;
  }

  return StartupGateStatus::Hold;
}

ModeController::Decision ModeController::make_startup_hold_decision() const
{
  Decision decision;
  decision.state.request = make_keep_current_state_request();
  decision.log_startup_input_wait = true;
  return decision;
}

void ModeController::log_startup_input_wait() const
{
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "[ModeController] waiting for startup teleop input to match DampingRequested or "
    "ReadyPoseRequested");
}

// Behavior state for this tick: authority-implied -> explicit request ->
// authority-owned (Manual: teleop / Api: service) -> keep current.
ModeController::StateResolution ModeController::resolve_next_state(
  Authority authority,
  const AuthorityChange & authority_change)
{
  if (authority_change.implied_state) {
    return StateResolution{*authority_change.implied_state};
  }

  if (!requests_->state_name.empty()) {
    return StateResolution{
      StateRequest{
        requests_->state_name,
        behavior_kind_for_state(requests_->state_name),
        TransitionReason::RequestedState},
      true};
  }

  return resolve_authority_owned_state(authority);
}

// Commits a decision: the single place this tick's writes happen.
void ModeController::apply(const Decision & decision)
{
  if (decision.log_startup_input_wait) {
    log_startup_input_wait();
  }

  if (decision.failsafe) {
    apply_failsafe(*decision.failsafe);
  } else {
    apply_authority_change(decision.authority);
    if (decision.state.clear_explicit_request) {
      requests_->state_name.clear();
    }

    if (decision.state.consume_service_request) {
      static_cast<void>(service_request_gate_.consume());
    }

    if (decision.accept_startup_input) {
      startup_teleop_input_accepted_ = true;
    }

    if (decision.velocity) {
      apply_velocity_source(*decision.velocity);
    }
  }

  apply_state_request(decision.state.request);
}

bool ModeController::is_startup_teleop_input_safe(
  const ModeFsmTeleopInput & current_teleop_input) const
{
  const bool is_damping_requested = mode_state_machine_.does_teleop_input_match_condition(
    kDampingConditionName, current_teleop_input);
  const bool is_ready_pose_requested = mode_state_machine_.does_teleop_input_match_condition(
    kStartupReadyPoseConditionName, current_teleop_input);

  return is_damping_requested || is_ready_pose_requested;
}

// Pure detection: the highest-priority failsafe condition this tick, if any.
// No writes -- the flag clears and authority effects live in apply_failsafe().
std::optional<ModeController::TransitionReason> ModeController::failsafe_reason() const
{
  if (is_teleop_input_unavailable()) {
    return TransitionReason::TeleopInputTimeout;
  }
  if (is_teleop_damping_requested()) {
    return TransitionReason::TeleopInputDamping;
  }
  if (requests_->damping) {
    return TransitionReason::DampingRequest;
  }
  if (is_policy_action_limit_exceeded()) {
    return TransitionReason::ActionLimit;
  }
  if (is_policy_orientation_unsafe()) {
    return TransitionReason::BadOrientation;
  }

  return std::nullopt;
}

// Effects of acting on a failsafe (the writes failsafe_reason() does not do).
void ModeController::apply_failsafe(TransitionReason reason)
{
  const bool should_clear_damping_request =
    reason == TransitionReason::TeleopInputTimeout ||
    reason == TransitionReason::TeleopInputDamping ||
    reason == TransitionReason::DampingRequest;

  if (should_clear_damping_request) {
    requests_->damping = false;
  }

  if (reason == TransitionReason::ActionLimit) {
    requests_->action_limit_exceeded = false;
  }

  // The operator's damping switch or a lost teleop link revokes API authority;
  // re-entry then requires a fresh API switch edge. Other failsafes just drop
  // any pending service request.
  const bool should_revoke_api_authority =
    reason == TransitionReason::TeleopInputDamping ||
    reason == TransitionReason::TeleopInputTimeout;
  if (should_revoke_api_authority) {
    apply_authority_change(manual_authority_change(reason));
  } else {
    clear_pending_service_request();
  }
}

bool ModeController::is_teleop_input_unavailable() const
{
  return state_->teleop.unavailable.load(std::memory_order_acquire);
}

// State selected by the current authority. Manual follows teleop transitions;
// Api takes the pending service request. No request keeps the active state.
ModeController::StateResolution ModeController::resolve_authority_owned_state(
  Authority authority)
{
  StateResolution resolution{make_keep_current_state_request()};

  if (authority == Authority::Manual) {
    resolution.request = resolve_state_request_from_transitions();
    return resolution;
  }

  if (authority == Authority::Api) {
    if (const auto peeked = service_request_gate_.peek()) {
      resolution.consume_service_request = true;

      if (!peeked->name.empty()) {
        resolution.request = *peeked;
      }
    }
  }

  return resolution;
}

ModeController::StateRequest ModeController::make_keep_current_state_request() const
{
  return {
    mode_->active_state_name,
    current_behavior_kind_,
    TransitionReason::KeepCurrentState};
}

ModeController::StateRequest ModeController::resolve_state_request_from_transitions() const
{
  const auto requested = mode_state_machine_.resolve_state_request_with_trigger_rules(
    mode_->active_state_name, make_current_teleop_input(), previous_teleop_input_);
  if (requested) {
    return make_state_request_from_fsm(*requested, TransitionReason::TeleopInputCondition);
  }

  return make_keep_current_state_request();
}

ModeFsmTeleopInput ModeController::make_current_teleop_input() const
{
  return ModeFsmTeleopInput{
    is_teleop_input_available(),
    state_->teleop.input_code,
    state_->teleop.selector_code
  };
}

bool ModeController::is_teleop_damping_requested() const
{
  // Match the damping condition itself instead of resolving state transitions:
  // while the operator holds the damping position, every other input is
  // ignored and the robot stays pinned in Damping regardless of active state.
  return mode_state_machine_.does_teleop_input_match_condition(
    kDampingConditionName, make_current_teleop_input());
}

void ModeController::clear_pending_service_request()
{
  service_request_gate_.discard();
}

// Select the velocity command owner for this tick. Manual uses teleop, full API
// uses /cmd_vel, and API warmup holds zero. zero_velocity suppresses the
// operator stick for one tick when API control is lost.
VelocitySource ModeController::select_velocity_source(
  Authority authority, bool zero_velocity) const
{
  if (zero_velocity || authority == Authority::ApiWarmup) {
    return VelocitySource::Zero;
  }

  return authority_.can_accept_api_velocity(authority) ?
         VelocitySource::Api : VelocitySource::Teleop;
}

void ModeController::apply_velocity_source(VelocitySource source)
{
  // Remember the source for active-range rescaling.
  mode_->velocity_source = source;
  switch (source) {
    case VelocitySource::Zero:
      mode_->velocity_commands.setZero();
      break;
    case VelocitySource::Api:
      mode_->velocity_commands = state_->api.velocity_commands;
      break;
    case VelocitySource::Teleop:
      mode_->velocity_commands = state_->teleop.velocity_commands;
      break;
  }
}

const char * ModeController::transition_reason_name(TransitionReason reason)
{
  switch (reason) {
    case TransitionReason::Initial:
      return "initial";
    case TransitionReason::Reset:
      return "reset";
    case TransitionReason::KeepCurrentState:
      return "keep_current_state";
    case TransitionReason::TeleopInputCondition:
      return "teleop_input_condition";
    case TransitionReason::ServiceRequest:
      return "service_request";
    case TransitionReason::RequestedState:
      return "requested_state";
    case TransitionReason::TeleopInputTimeout:
      return "teleop_input_timeout";
    case TransitionReason::TeleopInputDamping:
      return "teleop_input_damping";
    case TransitionReason::DampingRequest:
      return "damping_request";
    case TransitionReason::ActionLimit:
      return "action_limit";
    case TransitionReason::BadOrientation:
      return "bad_orientation";
    case TransitionReason::TeleopInputUnavailable:
      return "teleop_input_unavailable";
    case TransitionReason::ApiAuthorityRequested:
      return "api_authority_requested";
    case TransitionReason::ApiWarmupComplete:
      return "api_warmup_complete";
    case TransitionReason::ApiAuthorityLost:
      return "api_authority_lost";
    case TransitionReason::ApiAuthorityReleased:
      return "api_authority_released";
  }

  return "unknown";
}

bool ModeController::is_failsafe_transition_reason(TransitionReason reason)
{
  switch (reason) {
    case TransitionReason::TeleopInputTimeout:
    case TransitionReason::TeleopInputDamping:
    case TransitionReason::DampingRequest:
    case TransitionReason::ActionLimit:
    case TransitionReason::BadOrientation:
      return true;
    default:
      return false;
  }
}

void ModeController::initialize_joint_counts()
{
  controller_joint_count_ = state_->joint_map.controller_joint_names.size();

  if (controller_joint_count_ == 0) {
    throw std::runtime_error("ModeController requires configured controller joint order");
  }
}

void ModeController::configure_mode_runtime(const RootConfig & root_config)
{
  mode_state_machine_.reset(
    root_config.state_machine_config(),
    root_config.state_behaviors_config(),
    root_config.teleop_conditions_config(),
    root_config.selectors_config());
  authority_config_ = root_config.authority_config();

  if (const auto warmup = authority_config_.api_entry_warmup_duration) {
    authority_.set_warmup_duration(*warmup);
  }

  if (const auto threshold = authority_config_.api_entry_velocity_neutral_threshold) {
    authority_.set_switch_velocity_threshold(*threshold);
  }
}

void ModeController::enter_initial_state()
{
  const StateRequest initial_state{
    mode_state_machine_.initial_state_name(),
    mode_state_machine_.behavior_kind_for_state(mode_state_machine_.initial_state_name()),
    TransitionReason::Initial};

  enter_state(initial_state);
}

ModeController::ModeRequestResult ModeController::set_mode_by_name(
  const std::string & mode_name)
{
  ModeRequestResult result;
  result.active_mode = current_state_name();
  // Read before validating, rechecked in try_accept: a revocation in between rejects.
  const auto request_epoch = service_request_gate_.epoch();
  const auto authority = current_authority();

  RCLCPP_INFO(
    node_->get_logger(),
    "[ModeController] service request mode(%s) active(%s) authority(%s)",
    mode_name.c_str(),
    result.active_mode.c_str(),
    authority_name(authority));

  const bool can_accept_service_request =
    authority == Authority::Api &&
    state_->teleop.api_mode_requested &&
    is_api_heartbeat_valid();
  if (!can_accept_service_request) {
    result.message =
      "service requests require active API authority, a held API switch, and a valid heartbeat";
    return result;
  }

  const auto resolved = resolve_state_request_from_name(mode_name);
  if (!resolved) {
    result.message = "unknown or abstract mode name";
    return result;
  }

  if (!is_service_request_executable(*resolved)) {
    result.message = "mode is not reachable from the current mode";
    return result;
  }

  if (!service_request_gate_.try_accept(*resolved, request_epoch)) {
    result.message = "service request was superseded by an authority change or a pending request";
    return result;
  }

  result.success = true;
  result.message = "accepted mode=" + resolved->name;
  return result;
}

void ModeController::flush_transition_log()
{
  const auto count = pending_log_transition_count_.load(std::memory_order_acquire);
  if (count == 0 || count == logged_transition_count_) {
    return;
  }

  logged_transition_count_ = count;

  const auto * state = active_state_snapshot_.load(std::memory_order_acquire);
  const auto reason = static_cast<TransitionReason>(
    last_transition_reason_code_.load(std::memory_order_acquire));
  const auto * reason_name = transition_reason_name(reason);
  if (state == nullptr || reason_name == nullptr) {
    return;
  }

  if (is_failsafe_transition_reason(reason)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[ModeController] %s reason(%s)",
      state->name.c_str(),
      reason_name);
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[ModeController] %s reason(%s)",
    state->name.c_str(),
    reason_name);
}

ModeController::ModeStatusSnapshot ModeController::status_snapshot() const
{
  const auto active_state = current_state_name();
  const auto authority = current_authority();
  return ModeStatusSnapshot{
    active_state,
    available_service_state_names(),
    authority_name(authority),
    is_teleop_input_available(),
    is_api_heartbeat_valid(),
    is_api_request_available(),
    last_transition_reason_.load(std::memory_order_acquire)
  };
}

bool ModeController::is_api_request_available() const
{
  return current_authority() == Authority::Api &&
         state_->teleop.api_mode_requested &&
         is_api_heartbeat_valid() &&
         !service_request_gate_.busy();
}

bool ModeController::is_service_request_executable(const StateRequest & request) const
{
  const auto active_state = current_state_name();
  if (active_state.empty()) {
    return false;
  }

  const auto active_behavior_kind = behavior_kind_for_state(active_state);
  const bool is_already_active =
    request.name == active_state && request.behavior_kind == active_behavior_kind;
  if (is_already_active) {
    return true;
  }

  return is_transition_allowed(active_behavior_kind, request.behavior_kind);
}

std::string ModeController::current_state_name() const
{
  const auto * state = active_state_snapshot_.load(std::memory_order_acquire);
  if (state == nullptr) {
    return std::string{};
  }

  return state->name;
}

std::vector<std::string> ModeController::concrete_state_names() const
{
  return mode_state_machine_.concrete_state_names();
}

std::vector<std::string> ModeController::available_service_state_names() const
{
  if (current_authority() != Authority::Api) {
    return {};
  }

  std::vector<std::string> names;
  for (const auto & name : concrete_state_names()) {
    const StateRequest request{
      name,
      behavior_kind_for_state(name),
      TransitionReason::ServiceRequest};

    if (is_service_request_executable(request)) {
      names.push_back(name);
    }
  }

  return names;
}

std::vector<float> ModeController::make_controller_ordered_joint_positions() const
{
  std::vector<float> controller_values(controller_joint_count_, 0.0f);
  for (size_t i = 0; i < controller_joint_count_; ++i) {
    controller_values[i] = state_->sensors.joint_pos[static_cast<Eigen::Index>(i)];
  }

  return controller_values;
}

void ModeController::copy_controller_positions_to_output(
  const std::vector<float> & controller_values)
{
  for (size_t i = 0; i < controller_joint_count_; ++i) {
    output_->processed_action[i] = controller_values[i];
    output_->published_action[i] = controller_values[i];
  }
}

const StateBehavior & ModeController::behavior_for_state(const std::string & state_name) const
{
  return mode_state_machine_.behavior_for_state(state_name);
}

std::string ModeController::policy_for_state(const std::string & state_name) const
{
  return mode_state_machine_.policy_for_state(state_name);
}

BehaviorKind ModeController::behavior_kind_for_state(const std::string & state_name) const
{
  return mode_state_machine_.behavior_kind_for_state(state_name);
}

std::optional<ModeController::StateRequest> ModeController::resolve_state_request_from_name(
  const std::string & state_name) const
{
  const auto requested = mode_state_machine_.resolve_state_request_from_state_name(
    current_state_name(), state_name);
  if (requested) {
    return make_state_request_from_fsm(*requested, TransitionReason::ServiceRequest);
  }

  return std::nullopt;
}

// Pure: this tick's authority transition (Manual / ApiWarmup / Api) as data.
ModeController::AuthorityChange ModeController::compute_authority_change() const
{
  if (!is_teleop_input_available()) {
    return manual_authority_change(TransitionReason::TeleopInputUnavailable);
  }

  if (authority_.current() == Authority::Manual) {
    return compute_warmup_entry_change();
  }

  return compute_api_authority_change();
}

// Manual: the request-edge flag tracks the switch every tick; a rising edge that
// passes the entry checks begins the warmup.
ModeController::AuthorityChange ModeController::compute_warmup_entry_change() const
{
  AuthorityChange change;
  change.set_request_edge = state_->teleop.api_mode_requested;

  const bool is_rising_edge =
    state_->teleop.api_mode_requested && !was_teleop_api_mode_requested_;
  if (!is_rising_edge) {
    return change;
  }

  const auto * rejection_reason = authority_.api_entry_rejection_reason(
    *state_, can_current_state_enter_api());
  if (rejection_reason != nullptr) {
    log_api_authority_rejected(rejection_reason);
    return change;
  }

  change.begin_warmup = true;
  change.set_to = Authority::ApiWarmup;
  change.reason = TransitionReason::ApiAuthorityRequested;
  return change;
}

// Resolve API/API_WARMUP authority. Heartbeat loss has priority over switch
// release so stale API control takes the conservative zero-velocity handoff.
ModeController::AuthorityChange ModeController::compute_api_authority_change() const
{
  if (!is_api_heartbeat_valid()) {
    return compute_api_loss_change();
  }

  if (!state_->teleop.api_mode_requested) {
    return compute_api_release_change();
  }

  if (is_api_warmup_finished()) {
    AuthorityChange change;
    change.set_to = Authority::Api;
    change.reason = TransitionReason::ApiWarmupComplete;
    return change;
  }

  return AuthorityChange{};
}

// API released by the operator: hand back to Manual at the current switch
// position, unless both the active state and teleop input are motion.
ModeController::AuthorityChange ModeController::compute_api_release_change() const
{
  if (is_api_to_manual_release_blocked()) {
    log_api_manual_release_blocked();
    return AuthorityChange{};
  }

  auto change = manual_authority_change(TransitionReason::ApiAuthorityReleased);
  change.implied_state =
    resolve_teleop_handoff_request(TransitionReason::ApiAuthorityReleased, true);
  return change;
}

void ModeController::log_api_authority_rejected(const char * rejection_reason) const
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[ModeController] API authority rejected (%s); active(%s) authority(%s) "
    "teleop(%s) heartbeat(%s) input(%u) selector(%u); "
    "toggle the API switch off and on to retry",
    rejection_reason,
    mode_->active_state_name.c_str(),
    authority_name(authority_.current()),
    is_teleop_input_available() ? "valid" : "invalid",
    is_api_heartbeat_valid() ? "valid" : "invalid",
    state_->teleop.input_code,
    state_->teleop.selector_code);
}

void ModeController::log_api_manual_release_blocked() const
{
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "[ModeController] API->manual release blocked; active(%s) authority(%s) "
    "teleop(MimicRequested) input(%u) selector(%u)",
    mode_->active_state_name.c_str(),
    authority_name(authority_.current()),
    state_->teleop.input_code,
    state_->teleop.selector_code);
}

// API heartbeat lost: hand back to Manual, zero velocity for this tick, and
// never continue or start a motion from this handoff.
ModeController::AuthorityChange ModeController::compute_api_loss_change() const
{
  auto change = manual_authority_change(TransitionReason::ApiAuthorityLost);
  change.zero_velocity = true;
  const auto requested =
    resolve_teleop_handoff_request(TransitionReason::ApiAuthorityLost, false);

  if (requested) {
    change.implied_state = requested;
  } else if (is_mimic_state()) {
    change.implied_state = make_velocity_state_request(TransitionReason::ApiAuthorityLost);
  }

  return change;
}

// Switch to Manual: discard any pending service request and refresh the
// request-edge flag so API re-entry needs a fresh switch toggle.
ModeController::AuthorityChange ModeController::manual_authority_change(
  TransitionReason reason) const
{
  AuthorityChange change;
  change.discard_service = true;
  change.set_request_edge = state_->teleop.api_mode_requested;
  change.set_to = Authority::Manual;
  change.reason = reason;
  return change;
}

bool ModeController::is_teleop_input_available() const
{
  return state_->teleop.received.load(std::memory_order_acquire) &&
         !state_->teleop.unavailable.load(std::memory_order_acquire);
}

// Commits an authority change. The single place authority state is written.
void ModeController::apply_authority_change(const AuthorityChange & change)
{
  if (change.discard_service) {
    clear_pending_service_request();
  }

  if (change.set_request_edge) {
    was_teleop_api_mode_requested_ = *change.set_request_edge;
  }

  if (change.begin_warmup) {
    authority_.begin_warmup();
    // Start each API session with an empty gate (drops any leaked stale request).
    clear_pending_service_request();
  }

  if (change.set_to) {
    set_authority(*change.set_to, change.reason);
  }

  // change.zero_velocity is consumed by decide() (velocity source), not stored.
}

// Block the release while both the active state and the operator's input are
// motion (the MimicRequested condition).
bool ModeController::is_api_to_manual_release_blocked() const
{
  const bool is_mimic_requested = mode_state_machine_.does_teleop_input_match_condition(
    kMimicConditionName, make_current_teleop_input());

  return is_mimic_state() && is_mimic_requested;
}

// Shared API->manual handoff: apply the operator's current switch position
// through the FSM. No matching transition keeps the current state.
std::optional<ModeController::StateRequest> ModeController::resolve_teleop_handoff_request(
  TransitionReason reason, bool allow_mimic_target) const
{
  const auto requested = mode_state_machine_.resolve_state_request_by_level_match(
    mode_->active_state_name, make_current_teleop_input());
  if (!requested) {
    return std::nullopt;
  }

  if (!allow_mimic_target && mode_state_machine_.is_mimic_state(requested->name)) {
    return make_velocity_state_request(reason);
  }

  return make_state_request_from_fsm(*requested, reason);
}

ModeController::StateRequest ModeController::make_velocity_state_request(
  TransitionReason reason) const
{
  const auto & state_name = authority_config_.default_velocity_state;
  return {
    state_name,
    behavior_kind_for_state(state_name),
    reason};
}

bool ModeController::can_current_state_enter_api() const
{
  const auto & states = authority_config_.api_entry_allowed_from_states;
  return std::find(states.begin(), states.end(), mode_->active_state_name) != states.end();
}

void ModeController::set_authority(Authority mode, TransitionReason reason)
{
  if (authority_.current() == mode) {
    authority_.set_current(mode, *mode_);
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[ModeController] authority %s -> %s reason(%s)",
    authority_name(authority_.current()),
    authority_name(mode),
    transition_reason_name(reason));
  authority_.set_current(mode, *mode_);
  record_transition_reason(reason);
}

const char * ModeController::authority_name(Authority mode)
{
  switch (mode) {
    case Authority::Manual:
      return "MANUAL";
    case Authority::ApiWarmup:
      return "API_WARMUP";
    case Authority::Api:
      return "API";
  }

  return "UNKNOWN";
}

bool ModeController::is_mimic_state() const
{
  return mode_state_machine_.is_mimic_state(mode_->active_state_name);
}

bool ModeController::is_orientation_unsafe() const
{
  float limit_angle = 2.0f;
  if (is_mimic_state()) {
    const auto now = std::chrono::steady_clock::now();
    if (now - state_enter_time_ < std::chrono::milliseconds(500)) {
      return false;
    }

    limit_angle = 3.0f;
  }

  const float z = std::clamp(-state_->sensors.projected_gravity.z(), -1.0f, 1.0f);
  return std::abs(std::acos(z)) > limit_angle;
}

bool ModeController::is_transition_allowed(const StateRequest & request) const
{
  return is_transition_allowed(current_behavior_kind_, request.behavior_kind);
}

bool ModeController::is_transition_allowed(BehaviorKind from, BehaviorKind to) const
{
  if (to == BehaviorKind::Damping) {
    return true;
  }

  const bool target_is_posture_or_policy =
    to == BehaviorKind::Posture || to == BehaviorKind::Policy;

  switch (from) {
    case BehaviorKind::Damping:
      return to == BehaviorKind::Posture;
    case BehaviorKind::Posture:
    case BehaviorKind::Policy:
      return target_is_posture_or_policy;
  }

  return false;
}

void ModeController::enter_state(const StateRequest & request)
{
  commit_state_transition(request);
  enter_requested_mode(request);
}

void ModeController::commit_state_transition(const StateRequest & request)
{
  current_behavior_kind_ = request.behavior_kind;
  mode_->active_behavior_kind = request.behavior_kind;
  mode_->active_state_name = request.name;

  record_transition_reason(request.reason);

  active_state_snapshot_.store(
    mode_state_machine_.find_state(request.name),
    std::memory_order_release);

  state_enter_time_ = std::chrono::steady_clock::now();
  ++mode_->transition_count;
  mark_transition_log_pending();
}

void ModeController::enter_requested_mode(const StateRequest & request)
{
  switch (request.behavior_kind) {
    case BehaviorKind::Damping:
      enter_damping_state(request);
      break;
    case BehaviorKind::Posture:
      enter_posture_state(request);
      break;
    case BehaviorKind::Policy:
      enter_policy_state(request);
      break;
  }
}

// Keeps the active behavior's damping gains: every behavior fills
// output_->damping, so zeroing stiffness/feedforward and mirroring measured
// positions turns the last published command into a viscous brake.
void ModeController::apply_emergency_damping()
{
  output_->position_limits.assign(controller_joint_count_, std::nullopt);
  std::fill(output_->feedforward.begin(), output_->feedforward.end(), 0.0f);
  std::fill(output_->stiffness.begin(), output_->stiffness.end(), 0.0f);
  run_damping();
}

void ModeController::enter_damping_state(const StateRequest & request)
{
  const auto & behavior = behavior_for_state(request.name);

  mode_->active_policy_name.clear();
  requests_->damping = false;
  requests_->action_limit_exceeded = false;

  output_->position_limits.assign(controller_joint_count_, std::nullopt);
  std::fill(output_->feedforward.begin(), output_->feedforward.end(), 0.0f);
  std::fill(output_->stiffness.begin(), output_->stiffness.end(), 0.0f);
  output_->damping = behavior.damping;

  run_damping();
}

void ModeController::enter_posture_state(const StateRequest & request)
{
  const auto & behavior = behavior_for_state(request.name);

  mode_->active_policy_name.clear();

  output_->position_limits.assign(controller_joint_count_, std::nullopt);
  std::fill(output_->feedforward.begin(), output_->feedforward.end(), 0.0f);
  output_->stiffness = behavior.stiffness;
  output_->damping = behavior.damping;

  active_posture_duration_ = behavior.duration;
  active_posture_target_ = behavior.target_position;
  posture_start_controller_ = make_controller_ordered_joint_positions();
  posture_target_.assign(controller_joint_count_, 0.0f);
  posture_start_time_ = std::chrono::steady_clock::now();

  run_posture();
}

void ModeController::enter_policy_state(const StateRequest & request)
{
  // Gains and defaults are installed by PolicyRuntime::enter on the same tick
  // (via the zero-period transition pass) before any command is published.
  mode_->active_policy_name = policy_for_state(request.name);
  requests_->action_limit_exceeded = false;
}

void ModeController::record_transition_reason(TransitionReason reason)
{
  last_transition_reason_.store(transition_reason_name(reason), std::memory_order_release);
  last_transition_reason_code_.store(static_cast<int>(reason), std::memory_order_release);
}

void ModeController::mark_transition_log_pending()
{
  pending_log_transition_count_.store(mode_->transition_count, std::memory_order_release);
}

void ModeController::run_damping()
{
  for (Eigen::Index i = 0; i < state_->sensors.joint_pos.size(); ++i) {
    output_->processed_action[static_cast<size_t>(i)] = state_->sensors.joint_pos[i];
    output_->published_action[static_cast<size_t>(i)] = state_->sensors.joint_pos[i];
  }
}

void ModeController::run_posture()
{
  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - posture_start_time_).count();
  const double posture_phase = elapsed / active_posture_duration_;
  const float alpha = static_cast<float>(std::clamp(posture_phase, 0.0, 1.0));

  for (size_t i = 0; i < controller_joint_count_; ++i) {
    posture_target_[i] =
      posture_start_controller_[i] + alpha *
      (active_posture_target_[i] - posture_start_controller_[i]);
  }

  copy_controller_positions_to_output(posture_target_);
}

}  // namespace ai_sapiens_sim2real
