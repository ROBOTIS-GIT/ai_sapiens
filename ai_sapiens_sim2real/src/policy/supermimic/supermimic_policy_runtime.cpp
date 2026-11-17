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

#include "ai_sapiens_sim2real/policy/supermimic/supermimic_policy_runtime.hpp"

#include <utility>

namespace ai_sapiens_sim2real
{

std::unique_ptr<SuperMimicPolicyRuntime> SuperMimicPolicyRuntime::create(
  rclcpp::Node::SharedPtr node, std::string name, std::string model,
  const Sim2RealConfig & config, const std::optional<MimicBehavior> & mimic,
  const std::vector<std::string> & controller_joints, SharedControlData * data)
{
  auto observations = std::make_unique<SuperMimicObservation>(config, mimic, controller_joints,
      data);
  auto * view = observations.get();
  return std::unique_ptr<SuperMimicPolicyRuntime>(new SuperMimicPolicyRuntime(
      std::move(node), name, std::move(model), config, controller_joints, data,
      std::move(observations), view, mimic ? mimic->on_complete : std::string{},
      mimic ? mimic->motion_file.string() : std::string{}));
}

SuperMimicPolicyRuntime::SuperMimicPolicyRuntime(
  rclcpp::Node::SharedPtr node, std::string name, std::string model,
  const Sim2RealConfig & config, const std::vector<std::string> & controller_joints,
  SharedControlData * data, std::unique_ptr<SuperMimicObservation> observations,
  SuperMimicObservation * observation_view, std::string completion, std::string motion_file)
: PolicyRuntime(std::move(node), name, std::move(model), config,
    controller_joints, data, observation_view->reference(), std::move(observations)),
  observations_(observation_view)
{
  if (observations_->motion()) {
    references_.emplace(name, Reference{
      observations_->motion(), std::move(completion), std::move(motion_file)});
    active_reference_ = &references_.at(name);
  }
}

void SuperMimicPolicyRuntime::add_motion(
  const std::string & name, const MimicBehavior & mimic)
{
  if (!references_.emplace(name, Reference{
      observations_->prepare_motion(mimic), mimic.on_complete, mimic.motion_file.string()}).second)
  {
    throw std::invalid_argument("Duplicate SuperMimic reference: " + name);
  }
}

void SuperMimicPolicyRuntime::select_motion(const std::string & name)
{
  const auto & reference = references_.at(name);
  observations_->select_motion(reference.motion);
  active_reference_ = &reference;
}

bool SuperMimicPolicyRuntime::update_reference_state()
{
  if (active_reference_) {
    set_diagnostic_motion(active_reference_->motion_file, observations_->frame_index());
  }

  if (observations_->has_frame()) {return true;}
  if (active_reference_ && active_reference_->completion != "stay") {
    requests_->state_name = active_reference_->completion;
  }
  return false;
}

void SuperMimicPolicyRuntime::advance_clocks()
{
  PolicyRuntime::advance_clocks();
  observations_->advance();
}

}  // namespace ai_sapiens_sim2real
