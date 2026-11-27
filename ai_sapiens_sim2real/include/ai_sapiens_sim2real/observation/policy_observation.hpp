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

#ifndef AI_SAPIENS_SIM2REAL__OBSERVATION__POLICY_OBSERVATION_HPP_
#define AI_SAPIENS_SIM2REAL__OBSERVATION__POLICY_OBSERVATION_HPP_

#include <string>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_layout.hpp"

namespace ai_sapiens_sim2real
{

// Owns model input history. Action acceptance, retry budgets and delivery stay in PolicyRuntime.
class PolicyObservation
{
public:
  virtual ~PolicyObservation() = default;
  virtual void reset() = 0;
  virtual std::vector<float> compute() = 0;
  virtual void reject_latest() = 0;
  virtual std::vector<ObservationSlice> layout() const = 0;
  virtual size_t get_observation_size() const = 0;
  virtual std::string input_name() const {return {};}
};

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__OBSERVATION__POLICY_OBSERVATION_HPP_
