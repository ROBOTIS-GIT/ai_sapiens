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
// Author: Woojin Wie, Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__SHARED_CONTROL_DATA_HPP_
#define AI_SAPIENS_SIM2REAL__SHARED_CONTROL_DATA_HPP_

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/authority.hpp"
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"

namespace ai_sapiens_sim2real
{

// Glossary: a "mode" is one state of the mode state machine (Damping,
// ReadyPose, Velocity, Mimic*). Each mode runs a behavior of one of these
// kinds. The other control axis, who may command the robot, is Authority
// (see authority.hpp).
enum class BehaviorKind
{
  Damping,
  Posture,
  Policy,
};

// Which stream supplies the active velocity command. Selected by ModeController
// and read by PolicyRuntime to rescale the source command to the active range.
enum class VelocitySource
{
  Zero,
  Api,
  Teleop,
};

// Robot sensor samples. Written by the IMU and joint-state sensor handles.
struct SensorData
{
  static constexpr float GRAVITY_VEC_W[3] = {0.0f, 0.0f, -1.0f};

  Eigen::Vector3f angular_velocity{Eigen::Vector3f::Zero()};
  Eigen::Vector3f projected_gravity{0.0f, 0.0f, -1.0f};
  Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
  Eigen::VectorXf joint_pos;
  Eigen::VectorXf joint_vel;

  // Project world gravity into the body frame using the current orientation.
  void compute_projected_gravity()
  {
    Eigen::Vector3f gravity_w(GRAVITY_VEC_W[0], GRAVITY_VEC_W[1], GRAVITY_VEC_W[2]);
    projected_gravity = orientation.conjugate() * gravity_w;
  }
};

// Operator input stream. Written by the teleop input handle; the mode
// controller additionally zeroes velocity_commands on an API->manual handoff.
struct TeleopInput
{
  Eigen::Vector3f velocity_commands{Eigen::Vector3f::Zero()};
  // Normalized plugin command, rescaled to the active policy range before observation.
  Eigen::Vector3f velocity_command_normalized{Eigen::Vector3f::Zero()};
  std::atomic<bool> received{false};
  std::atomic<bool> unavailable{true};
  std::atomic<bool> api_mode_requested{false};
  uint16_t input_code{0};
  uint16_t selector_code{0};
  std::chrono::steady_clock::time_point update_time{};
};

// API command stream. Written by the twist and heartbeat handles; commands
// are accepted only while the heartbeat remains valid.
struct ApiInput
{
  Eigen::Vector3f velocity_commands{Eigen::Vector3f::Zero()};
  // Raw command, reclamped to the active policy range at inference time.
  Eigen::Vector3f velocity_command_raw{Eigen::Vector3f::Zero()};
  uint32_t heartbeat_sequence{0};
  std::atomic<bool> heartbeat_received{false};
  std::atomic<bool> heartbeat_stale{true};
  std::chrono::steady_clock::time_point heartbeat_update_time{};
};

// Robot/controller joint order. Filled once at startup by PolicyController.
struct JointIndexMap
{
  std::vector<std::string> controller_joint_names;
};

// Immutable per-policy joint order and map. Owned by PolicyRuntime and passed
// directly to that runtime's ObservationManager. Policy joints may be a subset
// of the robot joints; policy_to_controller[i] is where policy joint i lives in
// robot joint order.
struct PolicyJointContext
{
  std::vector<std::string> policy_joint_names;
  std::vector<size_t> policy_to_controller;
};

// Mode arbitration results. Written by ModeController and the
// AuthorityRuntime it drives.
struct ModeDecision
{
  BehaviorKind active_behavior_kind{BehaviorKind::Damping};
  Authority authority{Authority::Manual};
  std::string active_state_name{"Damping"};
  std::string active_policy_name;
  uint64_t transition_count{0};
  // Selected command plus its source for active-range rescaling.
  Eigen::Vector3f velocity_commands{Eigen::Vector3f::Zero()};
  VelocitySource velocity_source{VelocitySource::Zero};
};

// Latched requests produced outside the mode controller and consumed
// (cleared) by it.
struct ModeRequests
{
  std::string state_name;              // motion completion -> next mode
  bool damping{false};                 // teleop loss failsafe latch
  bool action_limit_exceeded{false};   // policy output fault latch
};

// Joint command of the active behavior: action buffers plus the joint
// properties (gains, limits, defaults) that behavior installed. Written by
// ModeController (damping/posture) or PolicyRuntime (policy); the command
// publisher writes back the clipped published_action.
//
// All buffers are in robot joint order and persist across ticks and mode
// transitions. A behavior that controls only a subset of joints (e.g. a
// lower-body policy) overwrites only its joints' slots; the rest keep whatever
// the previous behavior left here. This is what makes partial policies work.
struct BehaviorOutput
{
  std::vector<float> processed_action;
  std::vector<float> published_action;
  std::vector<float> feedforward;
  std::vector<float> stiffness;
  std::vector<float> damping;
  std::vector<float> action_scale;
  std::vector<float> action_offset;
  std::vector<PositionLimit> position_limits;
  Eigen::VectorXf default_joint_pos;
};

// Policy-side working state. Written by PolicyController and PolicyRuntime.
struct PolicyState
{
  std::vector<float> last_action;
  std::vector<float> input;  // scaled/clipped observation vector fed to ONNX
  Eigen::Quaternionf motion_init_quat{Eigen::Quaternionf::Identity()};
  float episode_time{0.0f};
  // Gait-phase clock in [0, 1). Advanced by GaitClock once per policy step and
  // read by the gait_phase observation. Stays 0 for policies without a gait phase.
  float gait_phase{0.0f};
};

/**
 * @brief Shared state exchanged between the control loop components.
 *
 * Grouped by writer: each block names the component(s) that may write it;
 * everyone else only reads. Pre-allocated at startup and shared via raw
 * pointers to avoid memory allocation in the realtime loop.
 */
struct SharedControlData
{
  SensorData sensors;
  TeleopInput teleop;
  ApiInput api;
  JointIndexMap joint_map;
  ModeDecision mode;
  ModeRequests requests;
  BehaviorOutput output;
  PolicyState policy;
  // Command ranges of the active policy. Written by PolicyRuntime, read by the
  // operator-input handles.
  AxisRanges active_velocity_command_ranges;

  /**
   * @brief Pre-allocate vectors based on configuration.
   *
   * Must be called at startup before entering the RT loop.
   *
   * @param num_joints Number of joints
   * @param action_size Size of action vector
   */
  void resize(size_t num_joints, size_t action_size)
  {
    sensors.joint_pos.resize(num_joints);
    sensors.joint_vel.resize(num_joints);
    output.default_joint_pos.resize(num_joints);
    output.processed_action.resize(action_size);
    output.published_action.resize(action_size);
    output.feedforward.resize(action_size);
    output.stiffness.resize(action_size);
    output.damping.resize(action_size);
    output.action_scale.resize(action_size);
    output.action_offset.resize(action_size);
    output.position_limits.assign(action_size, std::nullopt);
    policy.last_action.resize(action_size);

    // Zero initialize
    sensors.joint_pos.setZero();
    sensors.joint_vel.setZero();
    output.default_joint_pos.setZero();
    std::fill(output.processed_action.begin(), output.processed_action.end(), 0.0f);
    std::fill(output.published_action.begin(), output.published_action.end(), 0.0f);
    std::fill(output.feedforward.begin(), output.feedforward.end(), 0.0f);
    std::fill(output.stiffness.begin(), output.stiffness.end(), 0.0f);
    std::fill(output.damping.begin(), output.damping.end(), 0.0f);
    std::fill(output.action_scale.begin(), output.action_scale.end(), 0.0f);
    std::fill(output.action_offset.begin(), output.action_offset.end(), 0.0f);
    std::fill(policy.last_action.begin(), policy.last_action.end(), 0.0f);
  }

  /**
   * @brief Pre-allocate policy input buffer.
   * @param obs_size Total observation size
   */
  void resize_observation(size_t obs_size)
  {
    policy.input.resize(obs_size);
    std::fill(policy.input.begin(), policy.input.end(), 0.0f);
  }
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SHARED_CONTROL_DATA_HPP_
