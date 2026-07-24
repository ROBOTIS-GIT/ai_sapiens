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

#include <vector>

#include "ai_sapiens_sim2real/config/root_config.hpp"

namespace ai_sapiens_sim2real
{

PolicyController::PolicyController(
  rclcpp::Node::SharedPtr node,
  const RootConfig & root_config,
  SharedControlData * shared_data)
: node_(node),
  teleop_(&shared_data->teleop),
  policy_(&shared_data->policy)
{
  if (shared_data->joint_map.controller_joint_names.empty()) {
    throw std::runtime_error("PolicyController requires initialized controller joint order");
  }

  load_policies(shared_data, root_config);
  if (runtimes_.empty()) {
    throw std::runtime_error("PolicyController requires at least one policy runtime");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[PolicyController] initialized %zu policy runtime(s)",
    runtimes_.size());
}

void PolicyController::update(
  const ModeDecision & decision,
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  if (decision.active_behavior_kind != BehaviorKind::Policy) {
    return;
  }

  // Hold the last action while teleop input is unavailable; the mode
  // controller turns the same condition into a damping failsafe.
  if (teleop_->unavailable.load(std::memory_order_acquire)) {
    return;
  }

  auto * runtime = active_runtime(decision.active_policy_name);
  if (!runtime) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[PolicyController] active state '%s' requested policy '%s' but no runtime exists",
      decision.active_state_name.c_str(),
      decision.active_policy_name.c_str());
    return;
  }

  // The mode controller bumps mode.transition_count on every committed
  // transition; if one happened since our last enter, the active runtime
  // must start a fresh policy episode before it can update.
  if (entered_transition_count_ != decision.transition_count) {
    runtime->enter();
    entered_transition_count_ = decision.transition_count;
  }

  runtime->update(period);
}

void PolicyController::reset()
{
  for (auto & [_, runtime] : runtimes_) {
    runtime->reset();
  }

  entered_transition_count_ = 0;
  std::fill(policy_->last_action.begin(), policy_->last_action.end(), 0.0f);
}

std::string PolicyController::get_name() const
{
  return "policy_controller";
}

void PolicyController::load_policies(
  SharedControlData * shared_data, const RootConfig & root_config)
{
  const auto & controller_joints = shared_data->joint_map.controller_joint_names;
  const auto behaviors = root_config.policy_behaviors();

  // Resolve every root behavior and parse every nested sim2real.yaml before
  // opening the first ONNX model. A YAML error in a later behavior must not be
  // hidden behind an earlier model load or reported as part of ONNX startup.
  std::vector<Sim2RealConfig> sim2real_configs;
  sim2real_configs.reserve(behaviors.size());
  for (const auto & behavior : behaviors) {
    sim2real_configs.emplace_back(behavior.sim2real_config_path);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[PolicyController] preflighted %zu policy configuration(s)",
    behaviors.size());

  for (std::size_t i = 0; i < behaviors.size(); ++i) {
    const auto & behavior = behaviors[i];
    const auto & sim2real_config = sim2real_configs[i];
    // A mimic behavior drives a MimicPolicyRuntime; otherwise a plain policy.
    // Both drive the act stage through the same PolicyRuntime interface.
    std::unique_ptr<PolicyRuntime> runtime;
    if (behavior.mimic) {
      runtime = MimicPolicyRuntime::create(
        node_,
        behavior.name,
        behavior.policy_path.string(),
        sim2real_config,
        *behavior.mimic,
        controller_joints,
        shared_data);
    } else {
      runtime = std::make_unique<PolicyRuntime>(
        node_,
        behavior.name,
        behavior.policy_path.string(),
        sim2real_config,
        controller_joints,
        shared_data);
    }

    register_runtime(shared_data, behavior.name, std::move(runtime));
  }
}

void PolicyController::register_runtime(
  SharedControlData * shared_data,
  const std::string & state_name,
  std::unique_ptr<PolicyRuntime> runtime)
{
  if (shared_data->policy.input.size() < runtime->observation_size()) {
    shared_data->resize_observation(runtime->observation_size());
  }

  runtimes_.emplace(state_name, std::move(runtime));
}

PolicyRuntime * PolicyController::active_runtime(const std::string & policy_name)
{
  const auto it = runtimes_.find(policy_name);
  if (it != runtimes_.end()) {
    return it->second.get();
  }

  return nullptr;
}

}  // namespace ai_sapiens_sim2real
