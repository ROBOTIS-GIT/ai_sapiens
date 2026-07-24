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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__MOTION_PLAYBACK_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MOTION_PLAYBACK_HPP_

#include <limits>
#include <memory>
#include <optional>

namespace ai_sapiens_sim2real
{

class MotionReference;

// A reference motion plus the absolute-motion-time window [time_start, time_end]
// to play; the motion and its window travel as one value.
struct MotionPlayback
{
  std::shared_ptr<MotionReference> reference;
  float time_start{0.0f};
  float time_end{std::numeric_limits<float>::infinity()};

  // Absolute motion time to seek at this episode time, or nullopt once that
  // motion time passes time_end.
  std::optional<float> seek_time(float episode_time) const
  {
    const float motion_time = time_start + episode_time;
    if (motion_time > time_end) {
      return std::nullopt;
    }

    return motion_time;
  }
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__MOTION_PLAYBACK_HPP_
