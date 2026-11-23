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

#ifndef AI_SAPIENS_SIM2REAL__OBSERVATION__CONFIGURED_OBSERVATION_GROUP_HPP_
#define AI_SAPIENS_SIM2REAL__OBSERVATION__CONFIGURED_OBSERVATION_GROUP_HPP_

#include <yaml-cpp/yaml.h>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ai_sapiens_sim2real/observation/policy_observation.hpp"

namespace ai_sapiens_sim2real
{
struct ConfiguredObservationTerm
{
  std::string name;
  size_t width;
  std::vector<float> scale;
  std::optional<std::pair<float, float>> clip;
  void apply(std::vector<float> & values, size_t begin, size_t end) const;
};

// Owns packing and preprocessing only; each policy supplies its term calculations.
class ConfiguredObservationGroup
{
public:
  ConfiguredObservationGroup() = default;
  ConfiguredObservationGroup(
    const YAML::Node & node, const std::map<std::string, size_t> & supported_terms);
  void append_frames(std::vector<float> & output, const std::vector<float> & frames) const;
  std::vector<ObservationSlice> layout(size_t offset, size_t frames) const;
  const std::vector<ConfiguredObservationTerm> & terms() const {return terms_;}
  size_t width() const {return width_;}
  size_t history_length() const {return history_length_;}

private:
  std::vector<ConfiguredObservationTerm> terms_;
  size_t width_{0};
  size_t history_length_{1};
  bool is_term_major_{false};
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__OBSERVATION__CONFIGURED_OBSERVATION_GROUP_HPP_
