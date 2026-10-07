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

#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"

#include <cmath>

#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

bool contains_joint_name(
  const std::vector<std::string> & joint_names,
  const std::string & joint_name)
{
  return std::find(joint_names.begin(), joint_names.end(), joint_name) != joint_names.end();
}

std::string action_size_mismatch_message(
  const std::string & label,
  size_t actual_size,
  size_t expected_size)
{
  return label + " action size " + std::to_string(actual_size) +
         " does not match policy_joints size " + std::to_string(expected_size);
}

}  // namespace

PolicyRuntime::PolicyRuntime(
  rclcpp::Node::SharedPtr node,
  std::string state_name,
  std::string model_path,
  const Sim2RealConfig & sim2real_config,
  const std::vector<std::string> & controller_joint_names,
  SharedControlData * shared_data,
  const MotionReference * reference_motion)
: sensors_(&shared_data->sensors)
  , policy_(&shared_data->policy)
  , requests_(&shared_data->requests)
  , shared_data_(shared_data)
  , node_(std::move(node))
  , output_(&shared_data->output)
  , active_velocity_command_ranges_(&shared_data->active_velocity_command_ranges)
  , mode_(&shared_data->mode)
  , state_name_(std::move(state_name))
  , model_path_(std::move(model_path))
  , sim2real_config_path_(sim2real_config.path())
{
  load_sim2real_config(sim2real_config, controller_joint_names);
  log_joint_coverage(controller_joint_names);
  log_loading();
  load_onnx_model();
  const size_t observation_size =
    create_observation_manager(sim2real_config, shared_data, reference_motion);
  gait_clock_ = make_gait_clock(sim2real_config.observations(), step_dt_);
  if (sim2real_config.observations()["arm_mode"]) {
    if (sim2real_config.observations()["mode_command"] ||
      joint_context_.policy_joint_names.size() != 23 || std::abs(step_dt_ - 0.02) > 1e-9)
    {
      throw std::runtime_error("Arm mode requires a separate 23-joint 50Hz policy");
    }
    arm_mode_.emplace(YAML::LoadFile(sim2real_config.path().string()),
      joint_context_.policy_joint_names, action_pipeline_.properties().offset, step_dt_);
  }
  // 2026-09-30 carry port (from carry_ws sim2sim runtime): only policies whose sim2real.yaml
  // declares a mode_command observation get a carry mode machine; every other policy is unchanged.
  const auto carry_term = sim2real_config.observations()["mode_command"];
  if (carry_term && !carry_term.IsNull()) {
    if (joint_context_.policy_joint_names.size() != 23 || std::abs(step_dt_ - 0.02) > 1e-9) {
      throw std::runtime_error("Carry requires the 23-joint 50Hz hangang contract");
    }
    const auto cfg = carry_term["params"];
    if (!cfg || !cfg["sa_on_min_us"] || cfg["sa_on_min_us"].IsNull() ||
      !cfg["sa_on_max_us"] || cfg["sa_on_max_us"].IsNull()) {
      throw std::runtime_error("Carry SA CH5 ON PWM bounds must be verified and configured");
    }
    carry_sa_min_ = cfg["sa_on_min_us"].as<int>();
    carry_sa_max_ = cfg["sa_on_max_us"].as<int>();
    if (carry_sa_min_ < 500 || carry_sa_max_ > 2500 || carry_sa_min_ > carry_sa_max_) {
      throw std::runtime_error("Invalid carry SA PWM bounds");
    }
    if (cfg["se_channel"] && !cfg["se_channel"].IsNull()) {
      carry_se_channel_ = cfg["se_channel"].as<int>();
      if (carry_se_channel_ < 1 || carry_se_channel_ > 16 ||
        carry_se_channel_ == 5) {
        throw std::runtime_error("Invalid SE channel; CH5 is SA. CH8 (SD) is allowed for the bow trigger");
      }
      carry_se_min_ = cfg["se_on_min_us"].as<int>();
      carry_se_max_ = cfg["se_on_max_us"].as<int>();
      if (carry_se_min_ < 500 || carry_se_max_ > 2500 || carry_se_min_ > carry_se_max_) {
        throw std::runtime_error("Invalid SE PWM bounds");
      }
    }
    carry_machine_.emplace();
  }
  validate_observation_size(observation_size);
  obs_buffer_[onnx_input_name_].resize(observation_size);
  log_ready(observation_size);
}

PolicyRuntime::~PolicyRuntime() = default;

void PolicyRuntime::reset()
{
  accumulated_period_ = step_dt_;
  if (obs_manager_) {
    obs_manager_->reset();
  }
}

void PolicyRuntime::enter()
{
  install_joint_properties();
  install_velocity_command_ranges();
  reset_episode_state();
  on_enter();
  obs_manager_->reset();  // after on_enter(): seeds history from the state it sets
}

void PolicyRuntime::install_joint_properties() const
{
  const size_t policy_joint_count = joint_context_.policy_joint_names.size();
  if (joint_properties_.default_position.size() != policy_joint_count ||
    joint_properties_.stiffness.size() != policy_joint_count ||
    joint_properties_.damping.size() != policy_joint_count ||
    joint_properties_.position_limits.size() != policy_joint_count)
  {
    throw std::runtime_error("Policy state '" + state_name_ + "' joint property size mismatch");
  }

  // Scatter each policy joint's properties into its robot-joint slot. Slots for
  // joints this policy does not control are never touched, so they keep the
  // previous behavior's gains and defaults (see BehaviorOutput).
  const auto & action_properties = action_pipeline_.properties();
  for (size_t policy_index = 0; policy_index < policy_joint_count; ++policy_index) {
    const size_t controller_index = joint_context_.policy_to_controller[policy_index];
    output_->default_joint_pos[static_cast<Eigen::Index>(controller_index)] =
      joint_properties_.default_position[policy_index];
    output_->feedforward[controller_index] = 0.0f;
    output_->stiffness[controller_index] = joint_properties_.stiffness[policy_index];
    output_->damping[controller_index] = joint_properties_.damping[policy_index];
    output_->action_scale[controller_index] = action_properties.scale[policy_index];
    output_->action_offset[controller_index] = action_properties.offset[policy_index];
    output_->position_limits[controller_index] =
      joint_properties_.position_limits[policy_index];
  }
}

void PolicyRuntime::install_velocity_command_ranges() const
{
  *active_velocity_command_ranges_ = velocity_command_ranges_;
}

void PolicyRuntime::reset_episode_state()
{
  policy_->episode_time = 0.0f;
  if (carry_machine_) carry_machine_->reset();
  policy_->carry_mode.fill(0.0f);
  policy_->arm_mode = 0.0F;
  if (arm_mode_) {
    const auto & rc = shared_data_->teleop;
    const int ch = arm_mode_->channel;
    arm_mode_->reset(rc.carry_rc_valid[ch] && rc.carry_rc_us[ch] >= arm_mode_->on_min &&
      rc.carry_rc_us[ch] <= arm_mode_->on_max);
  }
  action_limit_logged_ = false;
  if (policy_->last_action.size() < joint_context_.policy_joint_names.size()) {
    throw std::runtime_error("Policy state '" + state_name_ + "' last_action buffer too small");
  }

  std::fill(policy_->last_action.begin(), policy_->last_action.end(), 0.0f);
  accumulated_period_ = step_dt_;
  consecutive_inference_failures_ = 0;
}

void PolicyRuntime::on_enter()
{
  if (gait_clock_) {
    gait_clock_->reset(policy_->gait_phase);
  }
}

bool PolicyRuntime::prepare_observation()
{
  if (gait_clock_) {
    gait_clock_->advance(policy_->gait_phase, *shared_data_);
  }

  return true;
}

void PolicyRuntime::advance_clocks()
{
  policy_->episode_time += static_cast<float>(step_dt_);
}

void PolicyRuntime::update(const rclcpp::Duration & period)
{
  if (!advance_policy_tick(period)) {
    return;
  }

  if (!prepare_observation()) {
    return;
  }

  if (!update_arm_mode()) return;
  resolve_active_velocity_command();
  update_carry_mode();
  compute_observation();

  if (const auto raw_action = run_policy_inference()) {
    const auto & processed_action = action_pipeline_.process(
      *raw_action, arm_mode_ ? &arm_mode_->offsets() : nullptr);
    write_processed_action(*raw_action, processed_action);
  }

  advance_clocks();
}

bool PolicyRuntime::advance_policy_tick(const rclcpp::Duration & period)
{
  // Accumulate measured control periods and run the policy at configured step_dt.
  accumulated_period_ += period.seconds();
  if (accumulated_period_ < step_dt_) {
    return false;
  }

  // Drop whole elapsed steps without looping on large time jumps.
  accumulated_period_ = std::fmod(accumulated_period_, step_dt_);
  return true;
}

bool PolicyRuntime::update_arm_mode()
{
  if (!arm_mode_) return true;
  const auto & rc = shared_data_->teleop;
  const int ch = arm_mode_->channel;
  if (rc.unavailable || !rc.carry_rc_valid[ch]) {
    requests_->damping = true;
    mode_->velocity_commands.setZero();
    return false;
  }
  const bool previous = arm_mode_->target();
  arm_mode_->step(rc.carry_rc_us[ch] >= arm_mode_->on_min &&
    rc.carry_rc_us[ch] <= arm_mode_->on_max);
  policy_->arm_mode = arm_mode_->ratio();
  *active_velocity_command_ranges_ = arm_mode_->ranges();
  for (size_t j = 0; j < joint_context_.policy_to_controller.size(); ++j) {
    output_->action_offset[joint_context_.policy_to_controller[j]] = arm_mode_->offsets()[j];
  }
  if (previous != arm_mode_->target()) {
    RCLCPP_INFO(node_->get_logger(), "[ArmMode] target=%s", arm_mode_->target() ? "hold" : "walk");
  }
  return true;
}

void PolicyRuntime::resolve_active_velocity_command()
{
  const auto & active = *active_velocity_command_ranges_;
  switch (shared_data_->mode.velocity_source) {
    case VelocitySource::Zero:
      mode_->velocity_commands.setZero();
      break;
    case VelocitySource::Api: {
        const auto & raw = shared_data_->api.velocity_command_raw;
        mode_->velocity_commands.x() = clamp_to_axis(raw.x(), active.linear_x);
        mode_->velocity_commands.y() = clamp_to_axis(raw.y(), active.linear_y);
        mode_->velocity_commands.z() = clamp_to_axis(raw.z(), active.angular_z);
        break;
      }
    case VelocitySource::Teleop: {
        const auto & normalized = shared_data_->teleop.velocity_command_normalized;
        mode_->velocity_commands.x() = scale_unit_to_axis(normalized.x(), active.linear_x);
        mode_->velocity_commands.y() = scale_unit_to_axis(normalized.y(), active.linear_y);
        mode_->velocity_commands.z() = scale_unit_to_axis(normalized.z(), active.angular_z);
        break;
      }
  }
}

// 2026-09-30 carry port (from carry_ws sim2sim runtime):
// no-op unless carry_machine_ was created in the constructor.
void PolicyRuntime::update_carry_mode()
{
  if (!carry_machine_) return;
  const auto & teleop = shared_data_->teleop;
  if (teleop.unavailable || !teleop.carry_rc_valid[5]) {
    // Preserve the existing damping path on unavailable operator input.
    requests_->damping = true;
    mode_->velocity_commands.setZero();
    return;
  }
  const auto active = [&](int ch, int lo, int hi) {
      return ch > 0 && teleop.carry_rc_valid[ch] &&
             teleop.carry_rc_us[ch] >= lo && teleop.carry_rc_us[ch] <= hi;
    };
  const bool sa = active(5, carry_sa_min_, carry_sa_max_);
  const bool se = active(carry_se_channel_, carry_se_min_, carry_se_max_);
  float leg_sum = 0.0f;
  for (size_t j = 0; j < 12; ++j) {
    const float v = sensors_->joint_vel[static_cast<Eigen::Index>(joint_context_.policy_to_controller[j])];
    leg_sum += v * v;
  }
  const auto requested = mode_->velocity_commands;
  const bool quiet = requested.cwiseAbs().maxCoeff() < 0.1f &&
    std::sqrt(leg_sum / 12.0f) < 0.35f && sensors_->angular_velocity.norm() < 0.35f;
  policy_->carry_mode = carry_machine_->step(sa, se, quiet,
    {requested.x(), requested.y(), requested.z()});
  for (int j = 0; j < 3; ++j) mode_->velocity_commands[j] = carry_machine_->command[j];
}

void PolicyRuntime::compute_observation()
{
  // ObservationManager returns the full scaled/clipped/history observation.
  obs_buffer_[onnx_input_name_] = obs_manager_->compute();
  policy_->input = obs_buffer_[onnx_input_name_];
}

// Owns the inference failure policy: reset the streak on success, count and
// escalate on failure. Returns nullopt when this step produced no action.
std::optional<std::vector<float>> PolicyRuntime::run_policy_inference()
{
  try {
    auto raw_action = inference_->run(obs_buffer_);
    consecutive_inference_failures_ = 0;
    return raw_action;
  } catch (const std::exception & e) {
    handle_inference_failure(e.what());
    return std::nullopt;
  }
}

// The failed step keeps the previous action, which is benign for one or two
// policy steps but must not continue indefinitely: a persistent fault would
// leave the robot running on a frozen action while appearing healthy.
void PolicyRuntime::handle_inference_failure(const char * reason)
{
  ++consecutive_inference_failures_;
  RCLCPP_WARN(
    node_->get_logger(),
    "Policy '%s' inference failed (%zu/%zu consecutive): %s",
    state_name_.c_str(),
    consecutive_inference_failures_,
    kMaxConsecutiveInferenceFailures,
    reason);

  if (consecutive_inference_failures_ >= kMaxConsecutiveInferenceFailures &&
    !requests_->damping)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Policy '%s' inference keeps failing; requesting damping failsafe",
      state_name_.c_str());
    requests_->damping = true;
  }
}

void PolicyRuntime::write_processed_action(
  const std::vector<float> & raw_action,
  const std::vector<float> & processed_action)
{
  const size_t policy_joint_count = joint_context_.policy_joint_names.size();
  if (processed_action.size() != policy_joint_count) {
    throw std::runtime_error(
      action_size_mismatch_message("Processed", processed_action.size(), policy_joint_count));
  }

  if (raw_action.size() != policy_joint_count) {
    throw std::runtime_error(
      action_size_mismatch_message("Raw", raw_action.size(), policy_joint_count));
  }

  requests_->action_limit_exceeded = false;
  for (size_t policy_index = 0; policy_index < policy_joint_count;
    ++policy_index)
  {
    const float value = processed_action[policy_index];
    if (!std::isfinite(value) || std::abs(value) > kAbsActionLimitRad) {
      requests_->action_limit_exceeded = true;
      log_action_limit_once(policy_index, raw_action[policy_index], value);
      return;
    }
  }

  action_limit_logged_ = false;

  // Scatter only after the whole action is known to be safe. Slots for joints
  // this policy does not control keep the previous behavior's command.
  for (size_t policy_index = 0; policy_index < policy_joint_count;
    ++policy_index)
  {
    const size_t controller_index = joint_context_.policy_to_controller[policy_index];
    output_->processed_action[controller_index] = processed_action[policy_index];
  }

  // The buffer is controller-sized; this policy uses the first joint_names.size() slots.
  std::copy(raw_action.begin(), raw_action.end(), policy_->last_action.begin());
}

void PolicyRuntime::log_action_limit_once(
  size_t policy_index,
  float raw_value,
  float processed_value)
{
  if (action_limit_logged_) {
    return;
  }

  const auto & joint_name = joint_context_.policy_joint_names[policy_index];
  const auto & properties = action_pipeline_.properties();
  const auto & clip = properties.clip[policy_index];
  const float clip_min = clip ? clip->first : 0.0f;
  const float clip_max = clip ? clip->second : 0.0f;
  RCLCPP_ERROR(
    node_->get_logger(),
    "Policy action limit exceeded: state=%s joint=%s raw=%.6f processed=%.6f rad "
    "limit=%.6f rad scale=%.6f offset=%.6f clip_enabled=%s clip_min=%.6f "
    "clip_max=%.6f; rejecting action commit",
    state_name_.c_str(),
    joint_name.c_str(),
    raw_value,
    processed_value,
    kAbsActionLimitRad,
    properties.scale[policy_index],
    properties.offset[policy_index],
    clip ? "true" : "false",
    clip_min,
    clip_max);
  action_limit_logged_ = true;
}

const std::string & PolicyRuntime::state_name() const
{
  return state_name_;
}

void PolicyRuntime::load_sim2real_config(
  const Sim2RealConfig & sim2real_config,
  const std::vector<std::string> & controller_joint_names)
{
  joint_context_.policy_joint_names = sim2real_config.policy_joints();
  // Policy joints may be a subset of the robot joints; every policy joint
  // must exist on the robot, but not the other way around.
  joint_context_.policy_to_controller = make_source_indices_for_target(
    controller_joint_names, joint_context_.policy_joint_names, "robot_joint_order",
    "policy_joints");

  step_dt_ = sim2real_config.step_dt();
  accumulated_period_ = step_dt_;

  joint_properties_ = sim2real_config.joint_properties();
  action_pipeline_ = ActionPipeline(sim2real_config.action_properties());

  if (action_pipeline_.size() != joint_context_.policy_joint_names.size()) {
    throw std::runtime_error("Policy state '" + state_name_ +
      "' action size does not match policy_joints");
  }

  if (const auto & ranges = sim2real_config.velocity_command_ranges()) {
    velocity_command_ranges_ = *ranges;
  }
}

void PolicyRuntime::log_joint_coverage(
  const std::vector<std::string> & controller_joint_names) const
{
  const bool covers_all_controller_joints =
    joint_context_.policy_joint_names.size() == controller_joint_names.size();
  if (covers_all_controller_joints) {
    return;
  }

  std::string uncontrolled;
  for (const auto & name : controller_joint_names) {
    if (contains_joint_name(joint_context_.policy_joint_names, name)) {
      continue;
    }

    if (!uncontrolled.empty()) {
      uncontrolled += ", ";
    }

    uncontrolled += name;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[PolicyRuntime] '%s' controls %zu/%zu joints; uncontrolled joints keep their previous "
    "commands and gains (uncontrolled: %s)",
    state_name_.c_str(),
    joint_context_.policy_joint_names.size(),
    controller_joint_names.size(),
    uncontrolled.c_str());
}

void PolicyRuntime::log_loading() const
{
  std::cout << "\n"
            << "[PolicyRuntime] ==================================================\n"
            << "[PolicyRuntime] Loading behavior: " << state_name_ << "\n"
            << "[PolicyRuntime] Model: " << model_path_ << "\n"
            << "[PolicyRuntime] sim2real.yaml: " << sim2real_config_path_.string() << "\n"
            << "[PolicyRuntime] --------------------------------------------------"
            << std::endl;
  RCLCPP_INFO(
    node_->get_logger(),
    "[PolicyRuntime] loading %s from %s",
    state_name_.c_str(),
    model_path_.c_str());
}

void PolicyRuntime::load_onnx_model()
{
  inference_ = std::make_unique<OnnxInference>(model_path_);
  const auto & input_names = inference_->get_input_names();
  if (input_names.size() != 1) {
    throw std::runtime_error(
      "Policy state '" + state_name_ + "' must have exactly one ONNX input");
  }

  onnx_input_name_ = input_names[0];

  // Validate ONNX output size before RT updates.
  const auto output_size = static_cast<size_t>(inference_->get_output_size());
  const auto action_size = action_pipeline_.size();
  if (output_size != action_size) {
    throw std::runtime_error("Policy state '" + state_name_ + "' ONNX output size " +
      std::to_string(output_size) + " does not match action size " +
      std::to_string(action_size));
  }
}

size_t PolicyRuntime::create_observation_manager(
  const Sim2RealConfig & sim2real_config,
  const SharedControlData * shared_data,
  const MotionReference * reference_motion)
{
  obs_manager_ = std::make_unique<ObservationManager>(
    sim2real_config.observations(),
    shared_data,
    joint_context_,
    reference_motion);
  return obs_manager_->get_observation_size();
}

void PolicyRuntime::validate_observation_size(size_t observation_size) const
{
  const auto & input_sizes = inference_->get_input_sizes();
  const int64_t expected_size = input_sizes.empty() ? 0 : input_sizes[0];
  const int64_t actual_size = static_cast<int64_t>(observation_size);

  if (expected_size != actual_size) {
    throw std::runtime_error(
      "Policy state '" + state_name_ + "' ONNX input expects " +
      std::to_string(expected_size) +
      " values, but sim2real observations produce " + std::to_string(observation_size));
  }
}

void PolicyRuntime::log_ready(size_t observation_size) const
{
  std::cout << "[PolicyRuntime] Ready: " << state_name_
            << " (obs=" << observation_size
            << ", action=" << inference_->get_output_size() << ")\n"
            << "[PolicyRuntime] ==================================================\n"
            << std::endl;
}

}  // namespace ai_sapiens_sim2real
