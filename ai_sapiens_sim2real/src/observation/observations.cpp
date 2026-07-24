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

#include "ai_sapiens_sim2real/observation/observations.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"
#include "ai_sapiens_sim2real/policy/torso_orientation.hpp"

namespace ai_sapiens_sim2real
{

std::map<std::string, ObservationRegistry::ObsFunc> & ObservationRegistry::get_registry()
{
  static std::map<std::string, ObsFunc> registry;
  return registry;
}

bool ObservationRegistry::register_observation(const std::string & name, ObsFunc func)
{
  get_registry()[name] = func;
  return true;
}

namespace detail
{

Eigen::Quaternionf yaw_quaternion(const Eigen::Quaternionf & q)
{
  const float yaw = std::atan2(
        2.0f * (q.w() * q.z() + q.x() * q.y()),
        1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  const float half_yaw = yaw * 0.5f;
  return Eigen::Quaternionf(std::cos(half_yaw), 0.0f, 0.0f, std::sin(half_yaw)).normalized();
}

std::vector<float> reorder_motion_to_policy(
  const Eigen::VectorXf & data,
  const std::vector<std::string> & policy_joint_order,
  const std::vector<std::string> & motion_joint_names)
{
  std::unordered_map<std::string, Eigen::Index> motion_index_by_name;
  for (size_t i = 0; i < motion_joint_names.size(); ++i) {
    motion_index_by_name[motion_joint_names[i]] = static_cast<Eigen::Index>(i);
  }

  std::vector<float> values(policy_joint_order.size(), 0.0f);
  for (size_t i = 0; i < policy_joint_order.size(); ++i) {
    const auto it = motion_index_by_name.find(policy_joint_order[i]);
    if (it == motion_index_by_name.end() || it->second >= data.size()) {
      throw std::runtime_error("Motion CSV is missing policy joint: " + policy_joint_order[i]);
    }

    values[i] = data(it->second);
  }

  return values;
}

std::vector<float> controller_vector_to_active_policy(
  const Eigen::VectorXf & controller_values,
  const std::vector<size_t> & policy_to_controller)
{
  std::vector<float> policy_values(policy_to_controller.size(), 0.0f);
  for (size_t policy_index = 0; policy_index < policy_to_controller.size(); ++policy_index) {
    const size_t controller_index = policy_to_controller[policy_index];
    if (controller_index >= static_cast<size_t>(controller_values.size())) {
      throw std::runtime_error("active policy_to_controller index is out of range");
    }

    policy_values[policy_index] = controller_values[static_cast<Eigen::Index>(controller_index)];
  }

  return policy_values;
}

// base_ang_vel: Angular velocity from IMU (rad/s)
std::vector<float> obs_base_ang_vel(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  return std::vector<float>{
    context.shared.sensors.angular_velocity.x(),
    context.shared.sensors.angular_velocity.y(),
    context.shared.sensors.angular_velocity.z()
  };
}

// projected_gravity: Gravity vector projected into body frame
std::vector<float> obs_projected_gravity(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  return std::vector<float>{
    context.shared.sensors.projected_gravity.x(),
    context.shared.sensors.projected_gravity.y(),
    context.shared.sensors.projected_gravity.z()
  };
}

// velocity_commands: From remote controller (lin_x, lin_y, ang_z)
std::vector<float> obs_velocity_commands(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  return std::vector<float>{
    context.shared.mode.velocity_commands.x(),
    context.shared.mode.velocity_commands.y(),
    context.shared.mode.velocity_commands.z()
  };
}

// joint_pos_rel: Joint positions relative to default
std::vector<float> obs_joint_pos_rel(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  std::vector<float> result(context.joints.policy_to_controller.size(), 0.0f);
  for (size_t policy_index = 0; policy_index < context.joints.policy_to_controller.size();
    ++policy_index)
  {
    const size_t controller_index = context.joints.policy_to_controller[policy_index];
    result[policy_index] =
      context.shared.sensors.joint_pos[static_cast<Eigen::Index>(controller_index)] -
      context.shared.output.default_joint_pos[static_cast<Eigen::Index>(controller_index)];
  }

  return result;
}

// joint_vel_rel: Joint velocities
std::vector<float> obs_joint_vel_rel(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  return controller_vector_to_active_policy(
    context.shared.sensors.joint_vel, context.joints.policy_to_controller);
}

// last_action is controller-sized with the policy-order action in the front
// slots, so return just the active policy joints.
std::vector<float> obs_last_action(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  const auto & last_action = context.shared.policy.last_action;
  return {
    last_action.begin(),
    last_action.begin() + static_cast<std::ptrdiff_t>(context.joints.policy_joint_names.size())};
}

// gait_phase: Periodic locomotion clock produced by GaitClock (policy-side).
// Pure read of the phase in shared state; returns [sin(2*pi*phi), cos(2*pi*phi)].
std::vector<float> obs_gait_phase(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  constexpr float kTwoPi = 2.0f * 3.14159265f;
  const float phase = context.shared.policy.gait_phase;
  return std::vector<float>{std::sin(kTwoPi * phase), std::cos(kTwoPi * phase)};
}

std::vector<float> obs_motion_joint_pos(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  if (context.reference_motion == nullptr) {
    return std::vector<float>(context.joints.policy_joint_names.size(), 0.0f);
  }

  return reorder_motion_to_policy(
        context.reference_motion->joint_pos(),
        context.joints.policy_joint_names,
        context.reference_motion->joint_order());
}

std::vector<float> obs_motion_joint_vel(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  if (context.reference_motion == nullptr) {
    return std::vector<float>(context.joints.policy_joint_names.size(), 0.0f);
  }

  return reorder_motion_to_policy(
        context.reference_motion->joint_vel(),
        context.joints.policy_joint_names,
        context.reference_motion->joint_order());
}

std::vector<float> obs_motion_command(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  auto pos = obs_motion_joint_pos(context, YAML::Node{});
  auto vel = obs_motion_joint_vel(context, YAML::Node{});
  pos.insert(pos.end(), vel.begin(), vel.end());
  return pos;
}

std::vector<float> obs_motion_anchor_ori_b(
  const ObservationContext & context,
  const YAML::Node & /*params*/)
{
  if (context.reference_motion == nullptr) {
    return std::vector<float>(6, 0.0f);
  }

  const auto real_torso_quat_w =
    torso_orientation_in_world(context.shared.sensors, context.joints);
  const auto ref_torso_quat_w =
    context.reference_motion->root_quaternion() *
    Eigen::AngleAxisf(context.reference_motion->joint_pos_for_joint(kWaistYawJointName),
        Eigen::Vector3f::UnitZ());
  const Eigen::Quaternionf rot_q =
    (context.shared.policy.motion_init_quat * ref_torso_quat_w).conjugate() *
    real_torso_quat_w;
  const Eigen::Matrix3f rot = rot_q.toRotationMatrix().transpose();
  return {rot(0, 0), rot(0, 1), rot(1, 0), rot(1, 1), rot(2, 0), rot(2, 1)};
}

// Static registration
struct ObservationRegistrar
{
  ObservationRegistrar()
  {
    ObservationRegistry::register_observation("base_ang_vel", obs_base_ang_vel);
    ObservationRegistry::register_observation("projected_gravity", obs_projected_gravity);
    ObservationRegistry::register_observation("velocity_commands", obs_velocity_commands);
    ObservationRegistry::register_observation("joint_pos_rel", obs_joint_pos_rel);
    ObservationRegistry::register_observation("joint_vel_rel", obs_joint_vel_rel);
    ObservationRegistry::register_observation("last_action", obs_last_action);
    ObservationRegistry::register_observation("gait_phase", obs_gait_phase);
    ObservationRegistry::register_observation("joint_pos", obs_joint_pos_rel);
    ObservationRegistry::register_observation("joint_vel", obs_joint_vel_rel);
    ObservationRegistry::register_observation("actions", obs_last_action);
    ObservationRegistry::register_observation("motion_joint_pos", obs_motion_joint_pos);
    ObservationRegistry::register_observation("motion_joint_vel", obs_motion_joint_vel);
    ObservationRegistry::register_observation("motion_command", obs_motion_command);
    ObservationRegistry::register_observation("motion_anchor_ori_b", obs_motion_anchor_ori_b);
  }
};

// Force registration at static initialization
static ObservationRegistrar observation_registrar_instance;

}  // namespace detail


}  // namespace ai_sapiens_sim2real
