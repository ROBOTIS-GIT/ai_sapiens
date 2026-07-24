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

#ifndef AI_SAPIENS_SIM2REAL__CONTROLLERS__POLICY_CONTROLLER_HPP_
#define AI_SAPIENS_SIM2REAL__CONTROLLERS__POLICY_CONTROLLER_HPP_

#include <algorithm>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/interfaces/controller_base.hpp"
#include "ai_sapiens_sim2real/policy/mimic_policy_runtime.hpp"
#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

// Defined in root_config.hpp; only referenced here, so forward-declared to keep
// the bootstrap header (pulled in by root_config.hpp) out of this include cycle.
class RootConfig;

class PolicyController : public ControllerBase
{
public:
  PolicyController(
    rclcpp::Node::SharedPtr node,
    const RootConfig & root_config,
    SharedControlData * shared_data);

  // The act stage consumes the decide stage's ModeDecision as a const argument,
  // so this controller can read the active mode but cannot rewrite it.
  void update(
    const ModeDecision & decision,
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period);
  void reset() override;
  std::string get_name() const override;

private:
  // Startup loading: create one runtime per resolved policy behavior.
  void load_policies(SharedControlData * shared_data, const RootConfig & root_config);
  void register_runtime(
    SharedControlData * shared_data,
    const std::string & state_name,
    std::unique_ptr<PolicyRuntime> runtime);

  // Runtime lookup follows the active mode's policy name.
  PolicyRuntime * active_runtime(const std::string & policy_name);

  rclcpp::Node::SharedPtr node_;
  // Reads the operator availability flag; owns the policy output buffers. Holds
  // no path to the mode block, which arrives per tick as a const argument.
  const TeleopInput * teleop_;
  PolicyState * policy_;

  // Stored by base pointer so mimic policies keep their derived behavior; the
  // active state's policy name selects which runtime drives the act stage.
  std::unordered_map<std::string, std::unique_ptr<PolicyRuntime>> runtimes_;
  // SharedControlData::mode.transition_count value at our last runtime enter.
  uint64_t entered_transition_count_{0};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONTROLLERS__POLICY_CONTROLLER_HPP_
