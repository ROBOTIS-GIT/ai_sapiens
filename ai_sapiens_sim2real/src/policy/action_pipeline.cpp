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

#include "ai_sapiens_sim2real/policy/action_pipeline.hpp"

namespace ai_sapiens_sim2real
{

ActionPipeline::ActionPipeline(ActionProperties properties)
: properties_(std::move(properties))
{
  const size_t action_size = properties_.scale.size();
  if (action_size == 0U) {
    throw std::runtime_error("ActionPipeline requires non-empty action scale");
  }

  if (properties_.offset.size() != action_size) {
    throw std::runtime_error("ActionPipeline offset size does not match scale size");
  }

  if (properties_.clip.size() != action_size) {
    throw std::runtime_error("ActionPipeline clip size does not match scale size");
  }

  processed_action_.resize(action_size, 0.0f);
}

const std::vector<float> & ActionPipeline::process(const std::vector<float> & raw_action)
{
  if (raw_action.size() != properties_.scale.size()) {
    throw std::runtime_error(
      "Raw action size " + std::to_string(raw_action.size()) +
      " does not match action pipeline size " + std::to_string(properties_.scale.size()));
  }

  for (size_t i = 0; i < raw_action.size(); ++i) {
    float value = raw_action[i] * properties_.scale[i] + properties_.offset[i];
    if (properties_.clip[i]) {
      value = std::clamp(value, properties_.clip[i]->first, properties_.clip[i]->second);
    }

    processed_action_[i] = value;
  }

  return processed_action_;
}


}  // namespace ai_sapiens_sim2real
