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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__ACTION_PIPELINE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__ACTION_PIPELINE_HPP_

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ai_sapiens_sim2real/config/sim2real_config.hpp"

namespace ai_sapiens_sim2real
{

class ActionPipeline
{
public:
  ActionPipeline() = default;
  explicit ActionPipeline(ActionProperties properties);

  const std::vector<float> & process(const std::vector<float> & raw_action);
  size_t size() const
  {
    return properties_.scale.size();
  }

  const ActionProperties & properties() const
  {
    return properties_;
  }

private:
  // Static action transform from config and reusable output buffer.
  ActionProperties properties_;
  std::vector<float> processed_action_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__ACTION_PIPELINE_HPP_
