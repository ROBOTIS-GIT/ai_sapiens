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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__MIMIC_POLICY_RUNTIME_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MIMIC_POLICY_RUNTIME_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/config/mimic_behavior.hpp"
#include "ai_sapiens_sim2real/policy/motion_playback.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief A policy state that also plays a reference motion.
 *
 * On entry it seeks the motion to its start and aligns its heading to the
 * robot's current torso yaw; each step it seeks the motion in lockstep with the
 * episode and, once the window ends, hands off to the configured completion
 * state. The policy mechanics (observe/infer/act) are inherited unchanged.
 */
class MimicPolicyRuntime : public PolicyRuntime
{
public:
  // Loads the reference motion described by mimic, then builds the runtime.
  static std::unique_ptr<MimicPolicyRuntime> create(
    rclcpp::Node::SharedPtr node,
    std::string state_name,
    std::string model_path,
    const Sim2RealConfig & sim2real_config,
    const MimicBehavior & mimic,
    const std::vector<std::string> & controller_joint_names,
    SharedControlData * shared_data);

  MimicPolicyRuntime(
    rclcpp::Node::SharedPtr node,
    std::string state_name,
    std::string model_path,
    const Sim2RealConfig & sim2real_config,
    const std::vector<std::string> & controller_joint_names,
    SharedControlData * shared_data,
    MotionPlayback playback,
    std::string completion_state);

protected:
  void on_enter() override;
  bool prepare_observation() override;

private:
  // Loads the reference motion and clamps the playback window to its duration.
  static MotionPlayback load_playback(
    const MimicBehavior & mimic,
    const std::vector<std::string> & controller_joint_names);
  static Eigen::Quaternionf yaw_quaternion(const Eigen::Quaternionf & q);

  // Outlives the runtime, so the reference-motion pointer the base handed to
  // its observation manager stays valid for the runtime's lifetime.
  MotionPlayback playback_;
  // Which state to hand off to once the playback window ends ("stay" = none).
  std::string completion_state_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__MIMIC_POLICY_RUNTIME_HPP_
