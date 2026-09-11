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
#include "ai_sapiens_sim2real/policy/torso_orientation.hpp"

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
  if (!adapter_) {gait_clock_ = make_gait_clock(sim2real_config.observations(), step_dt_);}
  validate_observation_size(observation_size);
  obs_buffer_[onnx_input_name_].resize(observation_size);
  log_ready(observation_size);
}

PolicyRuntime::~PolicyRuntime() = default;

void PolicyRuntime::reset()
{
  accumulated_period_ = step_dt_;
  if (adapter_) {reset_adapter_history();} else if (obs_manager_) {obs_manager_->reset();}
}

void PolicyRuntime::enter()
{
  install_joint_properties();
  install_velocity_command_ranges();
  reset_episode_state();
  on_enter();
  if (adapter_) {reset_adapter_history();} else {obs_manager_->reset();} // after on_enter(): seeds history from the state it sets
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
  adapter_steps_ = 0;
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
  if (adapter_) {policy_->episode_time = static_cast<float>(++adapter_steps_ * step_dt_);} else {
    policy_->episode_time += static_cast<float>(step_dt_);
  }
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
  try {
    compute_observation();
  } catch (const std::exception & e) {
    if (!adapter_) {throw;}
    requests_->damping = true;
    RCLCPP_ERROR(node_->get_logger(), "Adapter observation failed: %s", e.what());
    return;
  }

  if (const auto raw_action = run_policy_inference()) {
    const auto processed_action = process_action(*raw_action);
    if (write_processed_action(*raw_action, processed_action) && adapter_) {
      commit_adapter_history(processed_action);
    }
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

std::vector<float> PolicyRuntime::process_action(const std::vector<float> & raw_action)
{
  return action_pipeline_.process(raw_action);
}

std::vector<float> PolicyRuntime::adapter_state_frame() const
{
  const size_t count = joint_context_.policy_joint_names.size();
  std::vector<float> frame(6 + 3 * count);
  for (size_t axis = 0; axis < 3; ++axis) {
    frame[axis] = sensors_->angular_velocity[axis] * joint_vel_scale_;
    frame[3 + axis] = sensors_->projected_gravity[axis];
  }
  for (size_t i = 0; i < count; ++i) {
    const auto controller = joint_context_.policy_to_controller[i];
    frame[6 + i] = sensors_->joint_pos[controller] - joint_properties_.default_position[i];
    frame[6 + count + i] = sensors_->joint_vel[controller] * joint_vel_scale_;
    frame[6 + 2 * count + i] = last_motor_targets_.at(i);
  }
  return frame;
}

void PolicyRuntime::reset_adapter_history()
{
  velocity_history_ready_ = false;
  const size_t count = joint_context_.policy_joint_names.size();
  last_motor_targets_.resize(count);
  for (size_t i = 0; i < count; ++i) {
    last_motor_targets_[i] = sensors_->joint_pos[joint_context_.policy_to_controller[i]];
  }
  adapter_current_frame_ = adapter_state_frame();
  adapter_history_.assign(history_length_, adapter_current_frame_);
}

void PolicyRuntime::compute_adapter_observation()
{
  if (adapter_history_.size() != static_cast<size_t>(history_length_)) {
    throw std::runtime_error("Adapter history is not initialized");
  }
  adapter_current_frame_ = adapter_state_frame();
  const size_t count = joint_context_.policy_joint_names.size();
  const auto ref_q = adapter_reference_->joint_pos();
  const auto ref_dq = adapter_reference_->joint_vel();
  auto & obs = obs_buffer_["obs"];
  obs.clear();
  for (size_t i = 0; i < count; ++i) {
    obs.push_back(ref_q[i] - sensors_->joint_pos[joint_context_.policy_to_controller[i]]);
  }
  for (size_t i = 0; i < count; ++i) {
    obs.push_back((ref_dq[i] - sensors_->joint_vel[joint_context_.policy_to_controller[i]]) *
        joint_vel_scale_);
  }
  obs.insert(obs.end(), adapter_current_frame_.begin() + 3, adapter_current_frame_.begin() + 6);
  obs.insert(obs.end(), adapter_current_frame_.begin(), adapter_current_frame_.begin() + 3);
  obs.insert(obs.end(), adapter_current_frame_.begin() + 6, adapter_current_frame_.end());
  if (orientation_tracking_) {
    // Pelvis IMU is aligned with the free root in K1's MuJoCo model.
    // Keep the initial yaw alignment fixed throughout disturbances.
    const auto orientation = pelvis_orientation_observation(sensors_->orientation,
      adapter_reference_->root_quaternion(), policy_->motion_init_quat);
    obs.insert(obs.end(), orientation.begin(), orientation.end());
  }
  const auto & feet = adapter_reference_->feet_height();
  obs.insert(obs.end(), feet.data(), feet.data() + feet.size());
  obs.push_back(adapter_reference_->root_height());
  for (float & value : obs) {
    if (!std::isfinite(value)) {throw std::runtime_error("Non-finite Adapter observation");}
    value = std::clamp(value, -100.0f, 100.0f);
  }
  policy_->input = obs;
  if (velocity_history_length_ > 0) {
    // v1 sensors are the contiguous obs terms gvec, gyro, joint_pos,
    // joint_vel, previous motor targets. Use the already scaled/clipped obs;
    // neither reference commands nor simulator velocity enter the estimator.
    const size_t frame_size = adapter_current_frame_.size();
    auto & velocity_history = obs_buffer_["velocity_history"];
    const auto sensors_begin = obs.begin() + 2 * count;
    if (!velocity_history_ready_) {
      for (int frame = 0; frame < velocity_history_length_; ++frame) {
        std::copy_n(sensors_begin, frame_size, velocity_history.begin() + frame * frame_size);
      }
    } else {
      std::copy_n(sensors_begin, frame_size, velocity_history.end() - frame_size);
    }
  }
  if (history_length_ == 0) {return;}
  auto & history = obs_buffer_["history"];
  for (size_t channel = 0; channel < adapter_current_frame_.size(); ++channel) {
    for (int frame = 0; frame < history_length_; ++frame) {
      const float value = adapter_history_[frame][channel];
      if (!std::isfinite(value)) {throw std::runtime_error("Non-finite Adapter history");}
      history[channel * history_length_ + frame] = value;
    }
  }
}

void PolicyRuntime::commit_adapter_history(const std::vector<float> & targets)
{
  last_motor_targets_ = targets;
  if (velocity_history_length_ > 0) {
    // Commit only after a valid action. Shift the preallocated time-major
    // buffer for the next tick; compute_observation replaces its final frame.
    // A failed inference/action therefore does not advance committed history.
    auto & velocity_history = obs_buffer_["velocity_history"];
    std::rotate(velocity_history.begin(),
      velocity_history.begin() + adapter_current_frame_.size(), velocity_history.end());
    velocity_history_ready_ = true;
  }
  if (history_length_ == 0) {return;}
  std::copy(targets.begin(), targets.end(), adapter_current_frame_.end() - targets.size());
  adapter_history_.pop_front();
  adapter_history_.push_back(adapter_current_frame_);
}

void PolicyRuntime::compute_observation()
{
  if (adapter_) {
    compute_adapter_observation();
    return;
  }
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

bool PolicyRuntime::write_processed_action(
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
      return false;
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
  return true;
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
  adapter_ = sim2real_config.is_adapter();
  const auto root = YAML::LoadFile(sim2real_config.path().string());
  if (adapter_) {
    const auto observations = sim2real_config.observations();
    orientation_tracking_ = static_cast<bool>(observations["motion_anchor_ori_b"]);
    if (orientation_tracking_) {
      const auto spec = root["orientation_tracking"];
      if (!spec || spec["version"].as<int>(0) != 1 ||
        spec["anchor"].as<std::string>("") != "pelvis" ||
        spec["alignment"].as<std::string>("") != "initial_yaw" ||
        spec["representation"].as<std::string>("") != "relative_rotation_first_two_columns_row_major" ||
        spec["quaternion_order"].as<std::string>("") != "wxyz" ||
        !spec["requires_pelvis_orientation_estimate"].as<bool>(false) ||
        spec["reference_time_offset_steps"].as<int>(0) != 1 ||
        observations["motion_anchor_ori_b"]["dimension"].as<int>(0) != 6)
      {
        throw std::runtime_error("Unsupported OpenTrack pelvis orientation contract");
      }
    }
  }
  history_length_ = sim2real_config.history_length();
  joint_vel_scale_ = sim2real_config.joint_vel_scale();
  joint_context_.policy_joint_names = sim2real_config.policy_joints();
  // Policy joints may be a subset of the robot joints; every policy joint
  // must exist on the robot, but not the other way around.
  joint_context_.policy_to_controller = make_source_indices_for_target(
    controller_joint_names, joint_context_.policy_joint_names, "robot_joint_order",
    "policy_joints");

  step_dt_ = sim2real_config.step_dt();
  accumulated_period_ = step_dt_;

  if (const auto spec = root["velocity_estimation"]) {
    // Single-file deployment contract: presence enables the estimator.
    // v1 fixes sensor order/layout/reset; only history length is variable.
    if (!adapter_ || !spec.IsMap() || spec.size() != 2 || !spec["history_length"] ||
      spec["version"].as<int>(0) != 1 || !orientation_tracking_ ||
      joint_context_.policy_joint_names.size() != 23 || std::abs(step_dt_ - 0.02) > 1e-9)
    {
      throw std::runtime_error("Unsupported sim2real.yaml velocity_estimation: require "
        "version: 1 and history_length for K1 23-joint orientation tracking at 50 Hz");
    }
    velocity_history_length_ = spec["history_length"].as<int>();
    if (velocity_history_length_ < 1 || velocity_history_length_ > 200) {
      throw std::runtime_error("velocity_estimation.history_length must be in [1,200]");
    }
  }

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
  if (adapter_) {
    const auto & sizes = inference_->get_input_sizes();
    const auto count = joint_context_.policy_joint_names.size();
    const auto history_input = std::find(input_names.begin(), input_names.end(), "history");
    const bool has_history = history_input != input_names.end();
    const bool has_velocity =
      std::find(input_names.begin(), input_names.end(), "velocity_history") != input_names.end();
    if (has_velocity != (velocity_history_length_ > 0)) {
      throw std::runtime_error("ONNX velocity_history does not match velocity_estimation in sim2real.yaml");
    }
    if (std::count(input_names.begin(), input_names.end(), "obs") != 1 ||
      input_names.size() != 1u + has_history + has_velocity)
    {
      throw std::runtime_error("OpenTrack requires obs and optional history/velocity_history inputs");
    }
    if (has_history) {
      const auto index = static_cast<size_t>(history_input - input_names.begin());
      const auto & shape = inference_->get_input_shapes()[index];
      if (shape.size() != 3 || shape[2] <= 0 || shape[2] > 10000) {
        throw std::runtime_error("Invalid OpenTrack ONNX history shape");
      }
      if (history_length_ != 0 && history_length_ != shape[2]) {
        throw std::runtime_error("OpenTrack YAML history length does not match ONNX");
      }
      history_length_ = static_cast<int>(shape[2]);
    } else if (history_length_ != 0) {
      throw std::runtime_error("OpenTrack policy has no history input but YAML requests history");
    }
    for (size_t i = 0; i < input_names.size(); ++i) {
      const auto expected = input_names[i] == "obs" ? observation_size() :
        input_names[i] == "history" ? (6 + 3 * count) * history_length_ :
        input_names[i] == "velocity_history" ? (6 + 3 * count) * velocity_history_length_ : 0;
      if (expected == 0 || sizes[i] != static_cast<int64_t>(expected)) {
        throw std::runtime_error("Adapter ONNX input name or size mismatch");
      }
      const std::vector<int64_t> shape = input_names[i] == "obs" ?
        std::vector<int64_t>{1, static_cast<int64_t>(observation_size())} :
        input_names[i] == "history" ?
        std::vector<int64_t>{1, static_cast<int64_t>(6 + 3 * count), history_length_} :
        std::vector<int64_t>{1, velocity_history_length_, static_cast<int64_t>(6 + 3 * count)};
      if (inference_->get_input_shapes()[i] != shape) {
        throw std::runtime_error("OpenTrack ONNX axes mismatch for " + input_names[i] +
          ": history uses batch/channel/time; velocity_history uses batch/time/channel");
      }
    }
  } else if (input_names.size() != 1) {
    throw std::runtime_error(
      "Policy state '" + state_name_ + "' must have exactly one ONNX input");
  }

  onnx_input_name_ = adapter_ ? "obs" : input_names[0];

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
  if (adapter_) {
    if (!reference_motion || !reference_motion->is_adapter() ||
      reference_motion->joint_order() != joint_context_.policy_joint_names)
    {
      throw std::runtime_error("Adapter requires an Adapter CSV in policy joint order");
    }
    adapter_reference_ = reference_motion;
    std::vector<std::string> expected = {"dif_joint_pos", "dif_joint_vel", "gvec_pelvis",
      "gyro_pelvis", "joint_pos", "joint_vel", "last_motor_targets", "ref_feet_height",
      "ref_root_height"};
    if (orientation_tracking_) expected.insert(expected.end()-2, "motion_anchor_ori_b");
    size_t i = 0;
    for (const auto & term : sim2real_config.observations()) {
      if (i >= expected.size() || term.first.as<std::string>() != expected[i++]) {
        throw std::runtime_error("Unsupported Adapter observation order");
      }
      const auto cfg = term.second;
      if (velocity_history_length_ > 0) {
        const auto name = term.first.as<std::string>();
        const size_t dimension = name == "gvec_pelvis" || name == "gyro_pelvis" ? 3u :
          name == "motion_anchor_ori_b" ? 6u : name == "ref_feet_height" ? 4u :
          name == "ref_root_height" ? 1u : joint_context_.policy_joint_names.size();
        if (cfg["dimension"].as<size_t>(0) != dimension) {
          throw std::runtime_error("Velocity-estimator observation dimension mismatch: " + name);
        }
      }
      if (cfg["history_length"].as<int>(1) != 1 || (cfg["clip"] && !cfg["clip"].IsNull())) {
        throw std::runtime_error("Adapter observations use separate history and fixed clipping");
      }
      if (const auto scale = cfg["scale"]; scale && !scale.IsNull()) {
        if (!scale.IsSequence()) {
          throw std::runtime_error("Adapter observation scale must be a sequence");
        }
        for (const auto & value : scale) {
          if (value.as<float>() != 1.0f) {
            throw std::runtime_error("Adapter observation scale must be 1");
          }
        }
      }
    }
    if (i != expected.size()) {throw std::runtime_error("Missing Adapter observations");}
    if (history_length_ > 0) {
      obs_buffer_["history"].resize((6 + 3 * joint_context_.policy_joint_names.size()) *
          history_length_);
    }
    if (velocity_history_length_ > 0) {
      obs_buffer_["velocity_history"].resize(75 * velocity_history_length_);
    }
    return observation_size();
  }
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
  const auto & names = inference_->get_input_names();
  const auto index = std::find(names.begin(), names.end(), onnx_input_name_) - names.begin();
  const int64_t expected_size = input_sizes.at(index);
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
            << ", action=" << inference_->get_output_size()
            << ", velocity_history=" << velocity_history_length_ << "x"
            << (velocity_history_length_ > 0 ? 75 : 0) << ")\n"
            << "[PolicyRuntime] ==================================================\n"
            << std::endl;
}

}  // namespace ai_sapiens_sim2real
