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

#include "ai_sapiens_sim2real/controllers/policy_controller.hpp"

#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_recorder.hpp"

namespace ai_sapiens_sim2real
{

PolicyController::PolicyController(
  rclcpp::Node::SharedPtr node,
  const RootConfig & root_config,
  SharedControlData * shared_data)
: node_(node),
  teleop_(&shared_data->teleop),
  policy_(&shared_data->policy),
  output_(&shared_data->output),
  command_transition_(shared_data->joint_map.controller_joint_names.size())
{
  if (shared_data->joint_map.controller_joint_names.empty()) {
    throw std::runtime_error("PolicyController requires initialized controller joint order");
  }

  initialize_fault_recorder();
  runtimes_ = load_policy_runtimes(node_, shared_data, root_config, fault_recorder_);

  if (runtimes_.behaviors.empty()) {
    throw std::runtime_error("PolicyController requires at least one policy runtime");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[PolicyController] initialized %zu policy runtime(s)",
    runtimes_.instances.size());
}

void PolicyController::initialize_fault_recorder()
{
  try {
    ActionFaultRecorder::Options options;
    std::string directory;
    node_->get_parameter_or("action_fault_directory", directory, std::string{});
    options.directory = directory;
    fault_recorder_ = std::make_shared<ActionFaultRecorder>(options, node_->get_logger());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Action fault file recording unavailable: %s", e.what());
  }
}

void PolicyController::update(
  const ModeDecision & decision,
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  if (decision.active_behavior_kind != BehaviorKind::Policy) {
    active_runtime_ = nullptr;
    // Posture states (including ReadyPose) publish position targets and gains
    // that can seed policy entry. Damping remains an immediate safety mode.
    can_blend_from_previous_ = decision.active_behavior_kind == BehaviorKind::Posture;
    return;
  }

  // Hold the last action while teleop input is unavailable; the mode
  // controller turns the same condition into a damping failsafe.
  if (teleop_->unavailable.load(std::memory_order_acquire)) {
    return;
  }

  auto * entry = active_runtime_entry(decision.active_policy_name);
  if (!entry) {
    active_runtime_ = nullptr;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[PolicyController] active state '%s' requested policy '%s' but no runtime exists",
      decision.active_state_name.c_str(),
      decision.active_policy_name.c_str());
    return;
  }

  auto & runtime = *entry->runtime;

  // The mode controller bumps mode.transition_count on every committed
  // transition. A compatible reference switch may continue the current episode.
  if (entered_transition_count_ != decision.transition_count) {
    if (entry->select_reference) {entry->select_reference();}

    if (!entry->continues_episode(active_runtime_)) {
      enter_policy(runtime, entry->action_transition);
    }

    runtime.set_active_behavior(decision.active_state_name, decision.active_policy_name);
    active_runtime_ = &runtime;
    entered_transition_count_ = decision.transition_count;
  }

  update_policy(runtime, period);
}

void PolicyController::enter_policy(
  PolicyRuntime & runtime, const ActionTransitionConfig & transition)
{
  const double duration = can_blend_from_previous_ && transition.enabled ?
    transition.duration : 0.0;
  const auto & source = output_->has_published_command ?
    output_->last_published : output_->command;
  command_transition_.begin(
    source, runtime.target_command(), runtime.controlled_joints(), duration, output_->command);
  runtime.enter();
  can_blend_from_previous_ = true;
}

void PolicyController::update_policy(PolicyRuntime & runtime, const rclcpp::Duration & period)
{
  const auto result = runtime.update(period);
  command_transition_.update(period.seconds(), result, runtime.target_command(), output_->command);
}

void PolicyController::reset()
{
  active_runtime_ = nullptr;
  for (const auto & runtime : runtimes_.instances) {
    runtime->reset();
  }

  command_transition_.reset();
  entered_transition_count_ = 0;
  can_blend_from_previous_ = false;
  std::fill(policy_->last_action.begin(), policy_->last_action.end(), 0.0f);
}

std::string PolicyController::get_name() const
{
  return "policy_controller";
}

LoadedPolicyRuntimes::Entry * PolicyController::active_runtime_entry(
  const std::string & policy_name)
{
  const auto it = runtimes_.behaviors.find(policy_name);
  if (it != runtimes_.behaviors.end()) {
    return &it->second;
  }

  return nullptr;
}

}  // namespace ai_sapiens_sim2real
