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

  resolve_active_velocity_command();
  compute_observation();

  if (const auto raw_action = run_policy_inference()) {
    const auto & processed_action = action_pipeline_.process(*raw_action);
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
