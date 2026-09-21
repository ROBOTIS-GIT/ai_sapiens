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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__PLANAR_STEERING_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__PLANAR_STEERING_CONFIG_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include "ai_sapiens_sim2real/axis_range.hpp"

namespace ai_sapiens_sim2real
{

// Deployment uses external commands only; random command sampling belongs to training.
struct PlanarSteeringConfig
{
  AxisRanges ranges{{-0.3, 0.3}, {-0.3, 0.3}, {-0.3, 0.3}};
  float smoothing_time_constant{0.5f};
  std::string tracking_mode{"trajectory"};
  float velocity_estimator_time_constant{0.1f};

  void validate() const
  {
    if (tracking_mode != "trajectory" && tracking_mode != "velocity") {
      throw std::runtime_error("mimic steering tracking_mode must be trajectory or velocity");
    }
    if (!std::isfinite(velocity_estimator_time_constant) || velocity_estimator_time_constant < 0.0f) {
      throw std::runtime_error("mimic velocity estimator time constant must be finite and >= 0");
    }
    for (const auto & range : {ranges.linear_x, ranges.linear_y, ranges.angular_z}) {
      if (!std::isfinite(range.min) || !std::isfinite(range.max) ||
        !(range.min <= 0.0 && range.max >= 0.0))
      {
        throw std::runtime_error("mimic steering ranges must be finite and include zero");
      }
    }
    if (!std::isfinite(smoothing_time_constant) || smoothing_time_constant < 0.0f) {
      throw std::runtime_error("mimic steering smoothing_time_constant must be finite and >= 0");
    }
  }
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__PLANAR_STEERING_CONFIG_HPP_
