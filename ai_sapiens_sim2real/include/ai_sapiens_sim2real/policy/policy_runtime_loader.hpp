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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__POLICY_RUNTIME_LOADER_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__POLICY_RUNTIME_LOADER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"

namespace ai_sapiens_sim2real
{

// Owns each runtime once; multiple behaviors can select references on the same runtime.
struct LoadedPolicyRuntimes
{
  struct Entry
  {
    PolicyRuntime * runtime;
    ActionTransitionConfig action_transition;
    std::function<void()> select_reference;
    bool preserves_episode_on_reference_switch{false};

    bool continues_episode(const PolicyRuntime * active) const
    {
      return preserves_episode_on_reference_switch && runtime == active;
    }
  };

  std::vector<std::unique_ptr<PolicyRuntime>> instances;
  std::unordered_map<std::string, Entry> behaviors;
};

LoadedPolicyRuntimes load_policy_runtimes(
  rclcpp::Node::SharedPtr node, SharedControlData * data, const RootConfig & root_config,
  std::shared_ptr<ActionFaultRecorder> fault_recorder);

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__POLICY_RUNTIME_LOADER_HPP_
