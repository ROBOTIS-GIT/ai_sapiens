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
// Author: Woojin Wie

#ifndef AI_SAPIENS_SIM2REAL__POLICY__SUPERMIMIC__SUPERMIMIC_POLICY_RUNTIME_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__SUPERMIMIC__SUPERMIMIC_POLICY_RUNTIME_HPP_

#include "ai_sapiens_sim2real/policy/supermimic/supermimic_observation.hpp"
#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"

namespace ai_sapiens_sim2real
{

// One inference runtime and history serve all references of this policy.
class SuperMimicPolicyRuntime : public PolicyRuntime
{
public:
  static std::unique_ptr<SuperMimicPolicyRuntime> create(
    rclcpp::Node::SharedPtr node, std::string name, std::string model,
    const Sim2RealConfig & config, const std::optional<MimicBehavior> & mimic,
    const std::vector<std::string> & controller_joints, SharedControlData * data);

  // Prepare all reference data at startup; selection does no file IO or policy reset.
  void add_motion(const std::string & name, const MimicBehavior & mimic);
  void select_motion(const std::string & name);

protected:
  bool update_reference_state() override;
  void advance_clocks() override;

private:
  SuperMimicPolicyRuntime(
    rclcpp::Node::SharedPtr node, std::string name, std::string model,
    const Sim2RealConfig & config, const std::vector<std::string> & controller_joints,
    SharedControlData * data, std::unique_ptr<SuperMimicObservation> observations,
    SuperMimicObservation * observation_view, std::string completion, std::string motion_file);

  SuperMimicObservation * observations_;  // Owned by the base runtime throughout this lifetime.
  struct Reference
  {
    std::shared_ptr<const SuperMimicMotion> motion;
    std::string completion;
    std::string motion_file;
  };
  std::unordered_map<std::string, Reference> references_;
  const Reference * active_reference_{nullptr};
};

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__SUPERMIMIC__SUPERMIMIC_POLICY_RUNTIME_HPP_
