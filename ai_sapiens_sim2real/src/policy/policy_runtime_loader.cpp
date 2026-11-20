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

#include "ai_sapiens_sim2real/policy/policy_runtime_loader.hpp"

#include <filesystem>
#include <map>
#include <utility>

#include "ai_sapiens_sim2real/policy/mimic/mimic_policy_runtime.hpp"
#include "ai_sapiens_sim2real/policy/blinkmimic/blinkmimic_policy_runtime.hpp"
#include "ai_sapiens_sim2real/policy/supermimic/supermimic_policy_runtime.hpp"

namespace ai_sapiens_sim2real
{
namespace
{
using PolicyAsset = std::pair<std::filesystem::path, std::filesystem::path>;

PolicyAsset policy_asset(const RootConfig::PolicyBehavior & behavior)
{
  return {std::filesystem::canonical(behavior.policy_path),
    std::filesystem::canonical(behavior.sim2real_config_path)};
}

std::vector<Sim2RealConfig> preflight_policy_configs(
  const std::vector<RootConfig::PolicyBehavior> & behaviors)
{
  // Parse every configuration before opening any ONNX model, so a later YAML
  // error cannot be hidden behind an earlier model load failure.
  std::vector<Sim2RealConfig> configs;
  configs.reserve(behaviors.size());
  for (const auto & behavior : behaviors) {
    configs.emplace_back(behavior.sim2real_config_path);
  }
  return configs;
}

class PolicyRuntimeLoader
{
public:
  PolicyRuntimeLoader(
    rclcpp::Node::SharedPtr node, std::shared_ptr<ActionFaultRecorder> recorder)
  : node_(std::move(node)), fault_recorder_(std::move(recorder)) {}

  LoadedPolicyRuntimes load(SharedControlData * data, const RootConfig & root_config);

private:
  struct RuntimeCache
  {
    std::map<PolicyAsset, BlinkMimicPolicyRuntime *> blinkmimic;
    std::map<PolicyAsset, SuperMimicPolicyRuntime *> supermimic;
  };

  LoadedPolicyRuntimes::Entry load_runtime(
    SharedControlData * data, const RootConfig::PolicyBehavior & behavior,
    const Sim2RealConfig & config, RuntimeCache & cache);
  LoadedPolicyRuntimes::Entry load_blinkmimic(
    SharedControlData * data, const RootConfig::PolicyBehavior & behavior,
    const Sim2RealConfig & config, RuntimeCache & cache);
  LoadedPolicyRuntimes::Entry load_supermimic(
    SharedControlData * data, const RootConfig::PolicyBehavior & behavior,
    const Sim2RealConfig & config, RuntimeCache & cache);
  PolicyRuntime * add_runtime(SharedControlData * data, std::unique_ptr<PolicyRuntime> runtime);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ActionFaultRecorder> fault_recorder_;
  LoadedPolicyRuntimes loaded_;
};

LoadedPolicyRuntimes PolicyRuntimeLoader::load(
  SharedControlData * shared_data, const RootConfig & root_config)
{
  const auto behaviors = root_config.policy_behaviors();
  const auto configs = preflight_policy_configs(behaviors);
  RCLCPP_INFO(node_->get_logger(), "[PolicyController] preflighted %zu policy configuration(s)",
    behaviors.size());

  RuntimeCache cache;
  for (size_t i = 0; i < behaviors.size(); ++i) {
    loaded_.behaviors.emplace(behaviors[i].name,
      load_runtime(shared_data, behaviors[i], configs[i], cache));
  }
  return std::move(loaded_);
}

LoadedPolicyRuntimes::Entry PolicyRuntimeLoader::load_runtime(
  SharedControlData * data, const RootConfig::PolicyBehavior & behavior,
  const Sim2RealConfig & config, RuntimeCache & cache)
{
  if (config.observations()["blinkmimic"]) {
    return load_blinkmimic(data, behavior, config, cache);
  }

  if (config.observations()["supermimic"]) {
    return load_supermimic(data, behavior, config, cache);
  }

  const auto & joints = data->joint_map.controller_joint_names;
  std::unique_ptr<PolicyRuntime> runtime;
  if (behavior.mimic) {
    runtime = MimicPolicyRuntime::create(node_, behavior.name, behavior.policy_path.string(),
      config, *behavior.mimic, joints, data);
  } else {
    runtime = std::make_unique<PolicyRuntime>(node_, behavior.name, behavior.policy_path.string(),
      config, joints, data);
  }

  return {add_runtime(data, std::move(runtime)), behavior.action_transition, {}};
}

LoadedPolicyRuntimes::Entry PolicyRuntimeLoader::load_blinkmimic(
  SharedControlData * data, const RootConfig::PolicyBehavior & behavior,
  const Sim2RealConfig & config, RuntimeCache & cache)
{
  if (!behavior.mimic) {
    throw std::runtime_error("BlinkMimic requires a kind: mimic behavior and a motion CSV");
  }

  const auto asset = policy_asset(behavior);
  auto & runtime = cache.blinkmimic[asset];
  std::string reference_name = behavior.name;
  if (!runtime) {
    reference_name = asset.first.string();
    auto created = BlinkMimicPolicyRuntime::create(node_, reference_name, asset.first.string(),
      config, *behavior.mimic, data->joint_map.controller_joint_names, data);
    runtime = created.get();
    add_runtime(data, std::move(created));
  } else {
    runtime->add_motion(reference_name, *behavior.mimic);
  }

  return {runtime, behavior.action_transition,
    [runtime, reference_name]() {runtime->select_motion(reference_name);}, true};
}

LoadedPolicyRuntimes::Entry PolicyRuntimeLoader::load_supermimic(
  SharedControlData * data, const RootConfig::PolicyBehavior & behavior,
  const Sim2RealConfig & config, RuntimeCache & cache)
{
  const bool is_motion =
    config.observations()["supermimic"]["encoder"].as<std::string>() == "motion";
  if (is_motion != behavior.mimic.has_value()) {
    throw std::runtime_error(
      "SuperMimic motion encoder requires kind: mimic; velocity encoder requires kind: policy");
  }

  const auto asset = policy_asset(behavior);
  auto & runtime = cache.supermimic[asset];
  std::string reference_name = behavior.name;
  if (!runtime) {
    reference_name = asset.first.string();
    auto created = SuperMimicPolicyRuntime::create(node_, reference_name, asset.first.string(),
      config, behavior.mimic, data->joint_map.controller_joint_names, data);
    runtime = created.get();
    add_runtime(data, std::move(created));
  } else if (behavior.mimic) {
    runtime->add_motion(reference_name, *behavior.mimic);
  }

  std::function<void()> select_reference;
  if (behavior.mimic) {
    select_reference = [runtime, reference_name]() {runtime->select_motion(reference_name);};
  }

  return {runtime, behavior.action_transition, std::move(select_reference),
    behavior.mimic.has_value()};
}

PolicyRuntime * PolicyRuntimeLoader::add_runtime(
  SharedControlData * shared_data, std::unique_ptr<PolicyRuntime> runtime)
{
  if (shared_data->policy.input.size() < runtime->observation_size()) {
    shared_data->resize_observation(runtime->observation_size());
  }

  runtime->set_fault_recorder(fault_recorder_);
  loaded_.instances.push_back(std::move(runtime));
  return loaded_.instances.back().get();
}

}  // namespace

LoadedPolicyRuntimes load_policy_runtimes(
  rclcpp::Node::SharedPtr node, SharedControlData * data, const RootConfig & root_config,
  std::shared_ptr<ActionFaultRecorder> fault_recorder)
{
  return PolicyRuntimeLoader(std::move(node), std::move(fault_recorder)).load(data, root_config);
}

}  // namespace ai_sapiens_sim2real
