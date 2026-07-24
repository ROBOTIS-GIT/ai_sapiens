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

#ifndef AI_SAPIENS_SIM2REAL__AXIS_RANGE_HPP_
#define AI_SAPIENS_SIM2REAL__AXIS_RANGE_HPP_

#include <algorithm>

namespace ai_sapiens_sim2real
{

struct AxisRange
{
  double min{-1.0};
  double max{1.0};
};

struct AxisRanges
{
  AxisRange linear_x;
  AxisRange linear_y;
  AxisRange angular_z;
};

// Zero-preserving normalize of a declared-range value onto [-1, 1]: the positive
// and negative sides scale by their own bound so neutral stays neutral.
inline float normalize_axis_to_unit(double value, const AxisRange & declared)
{
  if (value >= 0.0) {
    if (declared.max <= 0.0) {
      return 0.0f;
    }

    return static_cast<float>(value / declared.max);
  }

  if (declared.min >= 0.0) {
    return 0.0f;
  }

  return static_cast<float>(-(value / declared.min));
}

// Inverse of normalize_axis_to_unit onto an active range; each side scales by its
// own active bound, so normalize then scale reproduces a direct declared->active map.
inline float scale_unit_to_axis(float normalized, const AxisRange & active)
{
  if (normalized >= 0.0f) {
    return static_cast<float>(normalized * active.max);
  }

  return static_cast<float>(-normalized * active.min);
}

inline float clamp_to_axis(double value, const AxisRange & active)
{
  return static_cast<float>(std::clamp(value, active.min, active.max));
}

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__AXIS_RANGE_HPP_
