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

#include <cmath>
#include <limits>
#include <stdexcept>
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
  double time_start{0.0};
  double time_end{std::numeric_limits<double>::infinity()};
  double step_dt{0.02};

  void reset() {has_sampled_endpoint_ = false;}

  // Consume an episode-time sample during forward playback. Clamp the first
  // sample reaching/passing the endpoint to time_end, then complete on the
  // next call. reset() rearms the endpoint when the policy is entered again.
  std::optional<double> seek_time(double episode_time)
  {
    if (!std::isfinite(step_dt) || step_dt <= 0.0 ||
      !std::isfinite(time_start) || std::isnan(time_end) || time_end < time_start ||
      !std::isfinite(episode_time) || episode_time < 0.0)
    {
      throw std::runtime_error("Invalid motion playback time, window or policy period");
    }
    if (has_sampled_endpoint_) {
      return std::nullopt;
    }
    const double motion_time = time_start + episode_time;
    // Match the small tick-relative tolerance used for motion frame snapping.
    // This absorbs float-configured endpoints and accumulated-time roundoff.
    const double endpoint_tolerance = step_dt * 1.0e-3;
    if (std::isfinite(time_end) && motion_time >= time_end - endpoint_tolerance) {
      has_sampled_endpoint_ = true;
      return time_end;
    }
    return motion_time;
  }

private:
  bool has_sampled_endpoint_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__MOTION_PLAYBACK_HPP_
