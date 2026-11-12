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
#include <algorithm>
#include <limits>

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
  const auto mimic_term = sim2real_config.observations()["mimic_gate"];
  if (mimic_term && !mimic_term.IsNull()) {
    configure_mimic_gate(mimic_term);  // before the observation manager sizes the mimic_gate term
  }
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
      !cfg["sa_on_max_us"] || cfg["sa_on_max_us"].IsNull())
    {
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
        carry_se_channel_ == 5)
      {
        throw std::runtime_error(
            "Invalid SE channel; CH5 is SA. CH8 (SD) is allowed for the bow trigger");
      }
      carry_se_min_ = cfg["se_on_min_us"].as<int>();
      carry_se_max_ = cfg["se_on_max_us"].as<int>();
      if (carry_se_min_ < 500 || carry_se_max_ > 2500 || carry_se_min_ > carry_se_max_) {
        throw std::runtime_error("Invalid SE PWM bounds");
      }
    }
    carry_machine_.emplace();
  }
  if (mimic_gate_) {
    if (obs_manager_->get_term_names().back() != "mimic_gate") {
      throw std::runtime_error("mimic_gate must be the last observation term");
    }
    // w sits right after the 124-D teacher block.
    mimic_w_index_ = static_cast<int>(observation_size) - mimic_gate_->observation_size() +
      k1_mimic_gate::kTeacherDim;
    const auto walk_n = std::count(mimic_gate_->params.walk_allowed.begin(),
        mimic_gate_->params.walk_allowed.end(), true);
    if (walk_n > 0 && mimic_arm_indices_.size() != 10U) {
      throw std::runtime_error("mimic_gate walk_clips needs 10 arm joints");
    }
    RCLCPP_INFO(node_->get_logger(),
        "[MimicGate] walk-allowed clips: %ld (arms-only %ld), w index %d", walk_n,
      std::count(mimic_gate_->params.arms_only_ref.begin(), mimic_gate_->params.arms_only_ref.end(),
        true), mimic_w_index_);
  }
  validate_observation_size(observation_size);
  obs_buffer_[onnx_input_name_].resize(observation_size);
  target_command_.resize(joint_context_.policy_joint_names.size());
  target_command_.stiffness = joint_properties_.stiffness;
  target_command_.damping = joint_properties_.damping;
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
  if (carry_machine_) {carry_machine_->reset();}
  // Shared policy state survives behavior changes; never leak mimic arm masking
  // into another policy's observations.
  policy_->hangang_mask_pos.clear();
  policy_->hangang_mask_vel.clear();
  policy_->hangang_mask_last.clear();
  if (mimic_gate_) {
    // PolicyRuntime instances share this buffer, but v8 and v16 have different
    // clip counts. Restore this policy's observation width before history reset.
    policy_->mimic_gate_obs.assign(
      static_cast<size_t>(mimic_gate_->observation_size()), 0.0f);
    mimic_gate_->reset();
    mimic_last_state_ = -1;
    std::fill(mimic_walk_last_.begin(), mimic_walk_last_.end(), 0.0f);
    mimic_walk_active_ = false;
  }
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

PolicyUpdateResult PolicyRuntime::update(const rclcpp::Duration & period)
{
  if (!advance_policy_tick(period)) {
    return PolicyUpdateResult::Skipped;
  }

  if (!prepare_observation()) {
    return PolicyUpdateResult::TargetUnavailable;
  }

  if (!update_arm_mode()) {return PolicyUpdateResult::TargetUnavailable;}
  resolve_active_velocity_command();
  update_carry_mode();
  update_mimic_gate();
  compute_observation();

  PolicyUpdateResult result = PolicyUpdateResult::TargetUnavailable;
  if (const auto raw_action = run_policy_inference()) {
    const auto & processed_action = action_pipeline_.process(
      *raw_action, arm_mode_ ? &arm_mode_->offsets() : nullptr);
    result = accept_action(*raw_action, processed_action);
  }

  advance_clocks();
  return result;
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
  if (!arm_mode_) {return true;}
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
  if (!carry_machine_) {return;}
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
    const float v =
      sensors_->joint_vel[static_cast<Eigen::Index>(joint_context_.policy_to_controller[j])];
    leg_sum += v * v;
  }
  const auto requested = mode_->velocity_commands;
  const bool quiet = requested.cwiseAbs().maxCoeff() < 0.1f &&
    std::sqrt(leg_sum / 12.0f) < 0.35f && sensors_->angular_velocity.norm() < 0.35f;
  policy_->carry_mode = carry_machine_->step(sa, se, quiet,
      {requested.x(), requested.y(), requested.z()});
  for (int j = 0; j < 3; ++j) {
    mode_->velocity_commands[j] = carry_machine_->command[j];
  }
}

void PolicyRuntime::configure_mimic_gate(const YAML::Node & term)
{
  const auto & names = joint_context_.policy_joint_names;
  if (names.size() != static_cast<size_t>(k1_mimic_gate::kJoints) ||
    std::abs(step_dt_ - 0.02) > 1e-9 ||
    names[12] != "waist_yaw_joint" || names[0] != "left_hip_pitch_joint" ||
    names[11] != "right_ankle_roll_joint")
  {
    throw std::runtime_error(
        "mimic_gate requires the 23-joint 50 Hz robot joint order");
  }
  const auto cfg = term["params"];
  if (!cfg || !cfg["clips"] || !cfg["clips"].IsSequence() || cfg["clips"].size() == 0) {
    throw std::runtime_error("mimic_gate needs params.clips");
  }
  // trigger switch: trigger_* (2026-10-12: SA CH5, abort_on_release) or the older sd_* keys
  const auto key = [&](const char * a, const char * b) {return cfg[a] ? cfg[a] : cfg[b];};
  mimic_sd_channel_ = key("trigger_channel", "sd_channel").as<int>();
  mimic_sd_min_ = key("trigger_on_min_us", "sd_on_min_us").as<int>();
  mimic_sd_max_ = key("trigger_on_max_us", "sd_on_max_us").as<int>();
  mimic_dial_base_ = cfg["dial_base"].as<int>();
  if (mimic_sd_channel_ < 1 || mimic_sd_channel_ > 16 || mimic_sd_min_ < 500 ||
    mimic_sd_max_ > 2500 ||
    mimic_sd_min_ > mimic_sd_max_)
  {
    throw std::runtime_error("Invalid mimic_gate SD channel / PWM bounds");
  }
  k1_mimic_gate::Parameters p;
  p.dt = static_cast<float>(step_dt_);
  p.abort_on_release = cfg["abort_on_release"] && cfg["abort_on_release"].as<bool>();
  const auto opt = [&](const char * key, float & value) {
      if (cfg[key]) {
        value = cfg[key].as<float>();
      }
    };
  opt("quiet_seconds", p.quiet_seconds);
  opt("pending_timeout_seconds", p.pending_timeout_seconds);
  opt("ramp_seconds", p.ramp_seconds);
  opt("bridge_min_seconds", p.bridge_min_seconds);
  opt("bridge_max_seconds", p.bridge_max_seconds);
  opt("bridge_joint_speed", p.bridge_joint_speed);
  opt("bridge_torso_speed", p.bridge_torso_speed);
  opt("abort_max_reference_tilt", p.abort_max_reference_tilt);
  opt("quiet_command", p.quiet_command);
  opt("quiet_leg_speed", p.quiet_leg_speed);
  opt("quiet_ang_speed", p.quiet_ang_speed);
  if (cfg["acceleration_limits"]) {
    for (int i = 0; i < 3; ++i) {
      p.acceleration_limits[i] = cfg["acceleration_limits"][i].as<float>();
    }
  }
  const auto params_dir = sim2real_config_path_.parent_path();
  std::vector<k1_mimic_gate::Clip> clips;
  for (size_t i = 0; i < cfg["clips"].size(); ++i) {
    const auto path = params_dir / cfg["clips"][i].as<std::string>();
    clips.push_back(k1_mimic_gate::load_clip(path.string(), p.waist_index, p.dt));
    mimic_clip_names_.push_back(
      cfg["clip_names"] &&
        i <
        cfg["clip_names"].size() ? cfg["clip_names"][i].as<std::string>() : path.stem().string());
  }
  // 2026-10-12 walk-greet: clips that may play while walking (names from clip_names)
  const auto flags = [&](const char * key) {
      std::vector<bool> out(mimic_clip_names_.size(), false);
      if (!cfg[key]) {return out;}
      for (const auto & n : cfg[key]) {
        const auto it = std::find(mimic_clip_names_.begin(), mimic_clip_names_.end(),
          n.as<std::string>());
        if (it == mimic_clip_names_.end()) {
          throw std::runtime_error(std::string("mimic_gate ") + key + ": unknown clip " +
            n.as<std::string>());
        }
        out[static_cast<size_t>(it - mimic_clip_names_.begin())] = true;
      }
      return out;
    };
  p.walk_allowed = flags("walk_clips");
  p.arms_only_ref = flags("arms_only_ref_clips");
  p.fast_start = flags("fast_start_clips");   // 2026-10-13 patch4
  p.hold_frames.assign(clips.size(), -1);
  if (const auto hold_clips = cfg["hold_clips"]) {
    if (!hold_clips.IsMap()) {throw std::runtime_error("mimic_gate hold_clips must be a map");}
    for (const auto & item : hold_clips) {
      const auto name = item.first.as<std::string>();
      const auto it = std::find(mimic_clip_names_.begin(), mimic_clip_names_.end(), name);
      if (it == mimic_clip_names_.end()) {
        throw std::runtime_error("mimic_gate hold_clips: unknown clip " + name);
      }
      if (!item.second.IsSequence() || item.second.size() != 1U) {
        throw std::runtime_error("mimic_gate hold_clips needs one frame per clip: " + name);
      }
      const auto index = static_cast<size_t>(it - mimic_clip_names_.begin());
      const int frame = item.second[0].as<int>();
      if (frame < 0 || frame >= clips[index].frames() - 1) {
        throw std::runtime_error("mimic_gate hold_clips frame out of range: " + name);
      }
      p.hold_frames[index] = frame;
    }
  }
  if (cfg["hold_max_seconds"]) {p.hold_max_seconds = cfg["hold_max_seconds"].as<float>();}
  if (!std::isfinite(p.hold_max_seconds) || p.hold_max_seconds < 0.f) {
    throw std::runtime_error("Invalid mimic_gate hold_max_seconds");
  }
  if (cfg["switch_walk_seconds"]) {p.switch_walk_seconds = cfg["switch_walk_seconds"].as<float>();} // patch5
  if (!(p.switch_walk_seconds >= 0.f && p.switch_walk_seconds <= 2.f)) {
    throw std::runtime_error("Invalid switch_walk_seconds");
  }
  RCLCPP_INFO(node_->get_logger(), "[MimicGate] stand-start walk switch: %s (leg handoff %.2f s)",
    p.switch_walk_seconds > 0.f ? "on" : "off", p.switch_walk_seconds);
  if (cfg["fast_start_ramp_seconds"]) {
    p.fast_ramp_seconds = cfg["fast_start_ramp_seconds"].as<float>();
  }
  if (!(p.fast_ramp_seconds >= 0.f && p.fast_ramp_seconds <= p.ramp_seconds)) {
    throw std::runtime_error("Invalid fast_start_ramp_seconds");
  }
  RCLCPP_INFO(node_->get_logger(), "[MimicGate] fast-start clips: %zu, ramp %.2f s",
    static_cast<size_t>(std::count(p.fast_start.begin(), p.fast_start.end(), true)),
    p.fast_ramp_seconds > 0.f ? p.fast_ramp_seconds : p.ramp_seconds);
  if (const auto inter = cfg["interactive"]) {   // 2026-10-13 hold + branch (shake hands / fake)
    const auto index_of = [&](const std::string & n) {
        const auto it = std::find(mimic_clip_names_.begin(), mimic_clip_names_.end(), n);
        if (it == mimic_clip_names_.end()) {
          throw std::runtime_error("mimic_gate interactive: unknown clip " + n);
        }
        return static_cast<int>(it - mimic_clip_names_.begin());
      };
    p.inter_clip = index_of(inter["clip"].as<std::string>());
    p.inter_hold_frame = inter["hold_frame"].as<int>();
    p.inter_low_clip = index_of(inter["low_clip"].as<std::string>());
    p.inter_low_frame = inter["low_frame"].as<int>();
    mimic_select_channel_ = inter["select_channel"].as<int>();
    mimic_select_low_max_ = inter["select_low_max_us"].as<int>();
    mimic_select_high_min_ = inter["select_high_min_us"].as<int>();
    mimic_confirm_channel_ = inter["confirm_channel"].as<int>();
    p.confirm_is_trigger = mimic_confirm_channel_ == mimic_sd_channel_;
    if (mimic_select_channel_ < 1 || mimic_select_channel_ > 16 || mimic_confirm_channel_ < 1 ||
      mimic_confirm_channel_ > 16 || mimic_select_low_max_ >= mimic_select_high_min_ ||
      p.inter_hold_frame < 0 || p.inter_low_frame < 0)
    {
      throw std::runtime_error("Invalid mimic_gate interactive config");
    }
    RCLCPP_INFO(node_->get_logger(),
        "[MimicGate] interactive %s: hold frame %d, CH%d <= %d us -> %s frame %d, >= %d us -> continue, confirm CH%d flip",
      inter["clip"].as<std::string>().c_str(), p.inter_hold_frame, mimic_select_channel_,
        mimic_select_low_max_,
      inter["low_clip"].as<std::string>().c_str(), p.inter_low_frame, mimic_select_high_min_,
        mimic_confirm_channel_);
  }
  // 2026-10-13 patch3: dial slot table (dial_base + i -> clip name); clips left out are reachable only internally
  mimic_dial_clips_.clear();
  if (const auto dial_clips = cfg["dial_clips"]) {
    std::string table;
    for (const auto & n : dial_clips) {
      const auto it = std::find(mimic_clip_names_.begin(), mimic_clip_names_.end(),
          n.as<std::string>());
      if (it == mimic_clip_names_.end()) {
        throw std::runtime_error("mimic_gate dial_clips: unknown clip " + n.as<std::string>());
      }
      mimic_dial_clips_.push_back(static_cast<int>(it - mimic_clip_names_.begin()));
      table += " " +
        std::to_string(mimic_dial_base_ + static_cast<int>(mimic_dial_clips_.size()) - 1) + "=" +
        *it;
    }
    RCLCPP_INFO(node_->get_logger(), "[MimicGate] dial table:%s", table.c_str());
  }
  // 2026-10-15 patch6: loop slots after the dial table (dial_base + len(dial_clips) + k)
  if (const auto dial_loops = cfg["dial_loops"]) {
    if (mimic_dial_clips_.empty()) {
      throw std::runtime_error("mimic_gate dial_loops needs dial_clips");
    }
    for (const auto & seq : dial_loops) {
      std::vector<int> ids;
      std::string text;
      for (const auto & n : seq) {
        const auto it = std::find(mimic_clip_names_.begin(), mimic_clip_names_.end(),
            n.as<std::string>());
        if (it == mimic_clip_names_.end()) {
          throw std::runtime_error("mimic_gate dial_loops: unknown clip " + n.as<std::string>());
        }
        ids.push_back(static_cast<int>(it - mimic_clip_names_.begin()));
        text += (text.empty() ? "" : " > ") + *it;
      }
      if (ids.empty()) {throw std::runtime_error("mimic_gate dial_loops: empty loop");}
      p.loops.push_back(ids);
      RCLCPP_INFO(node_->get_logger(),
          "[MimicGate] dial loop %d = %s (repeat; release = finish the clip)",
        mimic_dial_base_ + static_cast<int>(mimic_dial_clips_.size() + p.loops.size()) - 1,
          text.c_str());
    }
  }
  for (size_t j = 0; j < names.size(); ++j) {
    if (names[j].find("shoulder") != std::string::npos ||
      names[j].find("elbow") != std::string::npos ||
      names[j].find("wrist") != std::string::npos) {mimic_arm_indices_.push_back(j);}
  }
  mimic_walk_last_.assign(names.size(), 0.0f);
  mimic_gate_.emplace(p, std::move(clips));
  policy_->mimic_gate_obs.assign(static_cast<size_t>(mimic_gate_->observation_size()), 0.0f);
  RCLCPP_INFO(node_->get_logger(),
      "[MimicGate] %s: %zu clips, trigger CH%d %d-%d us (%s), dial %d..%zu",
    state_name_.c_str(), mimic_clip_names_.size(), mimic_sd_channel_, mimic_sd_min_, mimic_sd_max_,
    p.abort_on_release ? "release = stop" : "second press = stop", mimic_dial_base_,
    static_cast<size_t>(mimic_dial_base_) +
    (mimic_dial_clips_.empty() ? mimic_clip_names_.size() :
    mimic_dial_clips_.size() + p.loops.size()) - 1U);
}

void PolicyRuntime::update_mimic_gate()
{
  if (!mimic_gate_) {return;}
  const auto & teleop = shared_data_->teleop;
  k1_mimic_gate::Inputs in;
  const bool rc = !teleop.unavailable &&
    teleop.carry_rc_valid[static_cast<size_t>(mimic_sd_channel_)];
  const uint16_t sd_us = teleop.carry_rc_us[static_cast<size_t>(mimic_sd_channel_)];
  in.sd = rc && sd_us >= mimic_sd_min_ && sd_us <= mimic_sd_max_;
  in.dial = teleop.unavailable ? -1 : static_cast<int>(teleop.selector_code) - mimic_dial_base_;
  // 2026-10-15 patch6: dial slots right after the dial table select a loop
  if (!mimic_dial_clips_.empty() && in.dial >= static_cast<int>(mimic_dial_clips_.size())) {
    const int k = in.dial - static_cast<int>(mimic_dial_clips_.size());
    if (k < static_cast<int>(mimic_gate_->params.loops.size())) {in.loop = k;}
  }
  if (!mimic_dial_clips_.empty() && in.dial >= 0) {
    in.dial = in.dial <
      static_cast<int>(mimic_dial_clips_.size()) ? mimic_dial_clips_[static_cast<size_t>(in.dial)] :
      -1;
  }
  in.stick = mode_->velocity_commands;
  k1_mimic_gate::Vec default_q;
  for (int j = 0; j < k1_mimic_gate::kJoints; ++j) {
    const auto c =
      static_cast<Eigen::Index>(joint_context_.policy_to_controller[static_cast<size_t>(j)]);
    in.q[j] = sensors_->joint_pos[c];
    in.dq[j] = sensors_->joint_vel[c];
    default_q[j] = output_->default_joint_pos[c];
  }
  in.gyro = sensors_->angular_velocity;
  in.torso = torso_orientation_in_world(*sensors_, joint_context_);
  if (mimic_select_channel_ > 0 && !teleop.unavailable) {
    const auto ch = static_cast<size_t>(mimic_select_channel_);
    const auto cf = static_cast<size_t>(mimic_confirm_channel_);
    if (teleop.carry_rc_valid[ch]) {
      const int us = teleop.carry_rc_us[ch];
      in.select = us <= mimic_select_low_max_ ? 1 : (us >= mimic_select_high_min_ ? 2 : 0);
    }
    in.confirm = teleop.carry_rc_valid[cf] ? teleop.carry_rc_us[cf] >
      1500 : mimic_gate_->confirm_previous;
  } else {
    in.confirm = mimic_gate_->confirm_previous;
  }
  mimic_gate_->step(in);
  mode_->velocity_commands = mimic_gate_->cmd;
  mimic_gate_->observation(in, default_q, policy_->last_action.data(),
      policy_->mimic_gate_obs.data());
  // Walking inference sees its own previous arm command while an upper-body clip plays.
  mimic_walk_active_ = mimic_gate_->walking_clip_active();
  if (mimic_walk_active_) {
    const size_t n = joint_context_.policy_joint_names.size();
    const auto & ap = action_pipeline_.properties();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    policy_->hangang_mask_pos.assign(n, nan);
    policy_->hangang_mask_vel.assign(n, nan);
    policy_->hangang_mask_last.assign(n, nan);
    for (const size_t j : mimic_arm_indices_) {
      policy_->hangang_mask_pos[j] = ap.offset[j] + ap.scale[j] * mimic_walk_last_[j] -
        default_q[static_cast<Eigen::Index>(j)];
      policy_->hangang_mask_vel[j] = 0.0f;
      policy_->hangang_mask_last[j] = mimic_walk_last_[j];
    }
  } else if (!policy_->hangang_mask_pos.empty()) {
    policy_->hangang_mask_pos.clear();
    policy_->hangang_mask_vel.clear();
    policy_->hangang_mask_last.clear();
  }
  if (mimic_gate_->state != mimic_last_state_) {
    static const char * kNames[] = {"WALK", "PENDING", "CLIP_IN", "CLIP", "CLIP_OUT", "BRIDGE_IN",
      "BRIDGE_OUT"};
    RCLCPP_INFO(node_->get_logger(), "[MimicGate] %s clip %d (%s) w %.2f%s",
        kNames[mimic_gate_->state],
      mimic_gate_->clip + mimic_dial_base_,
        mimic_clip_names_[static_cast<size_t>(mimic_gate_->clip)].c_str(),
      mimic_gate_->w,
        mimic_gate_->loop < 0 ? "" : (mimic_gate_->loop_stop ? " [loop, finishing]" : " [loop]"));
    mimic_last_state_ = mimic_gate_->state;
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
    if (mimic_gate_ && mimic_walk_active_) {
      // The second call with w = 0 runs pure walking on the masked observation.
      auto walk_obs = obs_buffer_;
      walk_obs[onnx_input_name_][static_cast<size_t>(mimic_w_index_)] = 0.0f;
      const auto walk_action = inference_->run(walk_obs);
      std::vector<bool> arm(raw_action.size(), false);
      for (const size_t j : mimic_arm_indices_) {
        arm[j] = true;
      }
      for (size_t j = 0; j < raw_action.size(); ++j) {
        // patch5: after a mid-clip switch the student's leg share (1 - handoff) fades out
        if (!arm[j]) {
          raw_action[j] = walk_action[j] + (1.0f - mimic_gate_->handoff) *
            (raw_action[j] - walk_action[j]);
        }
      }
      std::copy(walk_action.begin(), walk_action.end(), mimic_walk_last_.begin());
    } else if (mimic_gate_) {
      std::copy(raw_action.begin(),
          raw_action.begin() + static_cast<std::ptrdiff_t>(mimic_walk_last_.size()),
        mimic_walk_last_.begin());
    }
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

PolicyUpdateResult PolicyRuntime::accept_action(
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
      return PolicyUpdateResult::TargetUnavailable;
    }
  }

  action_limit_logged_ = false;

  // Only expose fully validated targets to the command output stage.
  std::copy(processed_action.begin(), processed_action.end(), target_command_.position.begin());

  // The buffer is controller-sized; this policy uses the first joint_names.size() slots.
  std::copy(raw_action.begin(), raw_action.end(), policy_->last_action.begin());
  return PolicyUpdateResult::TargetReady;
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
