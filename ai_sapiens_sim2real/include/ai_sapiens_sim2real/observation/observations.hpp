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

#ifndef AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATIONS_HPP_
#define AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATIONS_HPP_

#include <map>
#include <string>

#include "ai_sapiens_sim2real/observation/observation_term_cfg.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief Registry for observation functions.
 *
 * Observation functions read from SharedControlData and return raw observation values.
 * They are registered by name and looked up when parsing sim2real.yaml.
 */
class ObservationRegistry
{
public:
  using ObsFunc = ObservationTermCfg::ObsFunc;

  static std::map<std::string, ObsFunc> & get_registry();

  static bool register_observation(const std::string & name, ObsFunc func);
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATIONS_HPP_
