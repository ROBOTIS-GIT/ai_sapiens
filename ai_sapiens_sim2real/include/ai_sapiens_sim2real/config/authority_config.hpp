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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__AUTHORITY_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__AUTHORITY_CONFIG_HPP_

#include <optional>
#include <string>
#include <vector>

namespace ai_sapiens_sim2real
{

struct AuthorityConfig
{
  std::optional<double> api_entry_warmup_duration;
  std::optional<float> api_entry_velocity_neutral_threshold;
  std::vector<std::string> api_entry_allowed_from_states;
  std::string default_velocity_state;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__AUTHORITY_CONFIG_HPP_
