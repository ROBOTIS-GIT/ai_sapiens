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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__BLINKMIMIC__BLINKMIMIC_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__BLINKMIMIC__BLINKMIMIC_CONFIG_HPP_

#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
#include "ai_sapiens_sim2real/observation/configured_observation_group.hpp"
#include "ai_sapiens_sim2real/config/mimic_behavior.hpp"

namespace ai_sapiens_sim2real
{
// Validates deployment configuration before inference or history allocation.
struct BlinkMimicConfig
{
  explicit BlinkMimicConfig(const Sim2RealConfig & config);
  void validate_motion(const MimicBehavior & mimic) const;

  std::vector<std::string> joints;
  std::vector<float> defaults;
  ConfiguredObservationGroup history;
  std::vector<ConfiguredObservationGroup> goal_groups;
  size_t start_offset{1};
  size_t history_length;
  size_t future_frames;
  size_t future_stride;

private:
  void read_history(const YAML::Node & node);
  void read_motion_goal(const YAML::Node & goal);
};

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__BLINKMIMIC__BLINKMIMIC_CONFIG_HPP_
