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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__SIM2REAL_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__SIM2REAL_CONFIG_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/axis_range.hpp"
#include "ai_sapiens_sim2real/config/config_utils.hpp"

namespace ai_sapiens_sim2real
{

using PositionLimit = std::optional<std::pair<float, float>>;

struct JointProperties
{
  std::vector<float> default_position;
  std::vector<float> stiffness;
  std::vector<float> damping;
  std::vector<PositionLimit> position_limits;
};

struct ActionProperties
{
  std::vector<float> scale;
  std::vector<float> offset;
  std::vector<PositionLimit> clip;
};

// Maps a target joint order onto positions in a source order. Used to scatter a
// policy's (possibly partial) joints onto the controller's joint slots.
std::vector<size_t> make_source_indices_for_target(
  const std::vector<std::string> & source,
  const std::vector<std::string> & target,
  const std::string & source_label,
  const std::string & target_label);

/**
 * @brief A single policy's sim2real.yaml: the joint set it controls, its control
 *        rate, gains/defaults, action pipeline, and (optionally) command ranges.
 *
 * The whole file is parsed and validated in the constructor; the accessors just
 * return the stored values. Observation terms are not parsed here; the raw
 * observations() node is handed to ObservationManager, which owns that schema.
 */
class Sim2RealConfig
{
public:
  explicit Sim2RealConfig(const std::filesystem::path & path);

  const std::filesystem::path & path() const
  {
    return path_;
  }

  // The joints this policy controls, in policy order. The properties below are
  // aligned to this order.
  const std::vector<std::string> & policy_joints() const
  {
    return policy_joints_;
  }
  double step_dt() const
  {
    return step_dt_;
  }
  const JointProperties & joint_properties() const
  {
    return joint_properties_;
  }
  const ActionProperties & action_properties() const
  {
    return action_properties_;
  }
  const std::optional<AxisRanges> & velocity_command_ranges() const
  {
    return velocity_command_ranges_;
  }

  const YAML::Node & observations() const
  {
    return observations_;
  }

private:
  std::filesystem::path path_;
  std::vector<std::string> policy_joints_;
  double step_dt_{0.0};
  JointProperties joint_properties_;
  ActionProperties action_properties_;
  std::optional<AxisRanges> velocity_command_ranges_;
  YAML::Node observations_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__SIM2REAL_CONFIG_HPP_
