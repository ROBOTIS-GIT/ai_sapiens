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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__ACTION_TRANSITION_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__ACTION_TRANSITION_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ai_sapiens_sim2real
{

struct ActionTransitionConfig
{
  bool enabled{false};
  double duration{0.2};
};

// Blend final joint positions, never raw policy actions. The start is fixed at
// entry; the target may change on every inference. No allocation while blending.
class ActionTransition
{
public:
  void resize(size_t size)
  {
    start_.resize(size);
    start_stiffness_.resize(size);
    start_damping_.resize(size);
  }

  void capture_gains(
    const std::vector<float> & stiffness,
    const std::vector<float> & damping,
    const std::vector<size_t> & policy_to_controller)
  {
    if (start_.size() != policy_to_controller.size()) {
      throw std::runtime_error("Gain transition joint count mismatch");
    }
    for (size_t j = 0; j < start_.size(); ++j) {
      start_stiffness_[j] = stiffness.at(policy_to_controller[j]);
      start_damping_[j] = damping.at(policy_to_controller[j]);
    }
  }

  void begin(
    const std::vector<float> & published,
    const std::vector<size_t> & policy_to_controller,
    double duration)
  {
    if (!std::isfinite(duration) || duration < 0.0) {
      throw std::runtime_error("Action transition duration must be finite and non-negative");
    }
    if (start_.size() != policy_to_controller.size()) {
      throw std::runtime_error("Action transition joint count mismatch");
    }
    for (size_t j = 0; j < start_.size(); ++j) {
      start_[j] = published.at(policy_to_controller[j]);
    }
    duration_ = duration;
    elapsed_ = 0.0;
  }

  void reset() {duration_ = 0.0; elapsed_ = 0.0;}

  void advance(double period)
  {
    if (std::isfinite(period) && period > 0.0) {
      elapsed_ = std::min(duration_, elapsed_ + period);
    }
  }

  float position(size_t joint, float target) const
  {
    return blend(start_[joint], target);
  }

  float stiffness(size_t joint, float target) const
  {
    return blend(start_stiffness_[joint], target);
  }

  float damping(size_t joint, float target) const
  {
    return blend(start_damping_[joint], target);
  }

private:
  float blend(float start, float target) const
  {
    if (duration_ <= 0.0 || elapsed_ >= duration_) {
      return target;
    }
    const double s = elapsed_ / duration_;
    const double alpha = s * s * (3.0 - 2.0 * s);
    return static_cast<float>((1.0 - alpha) * start + alpha * target);
  }

  std::vector<float> start_;
  std::vector<float> start_stiffness_;
  std::vector<float> start_damping_;
  double duration_{0.0};
  double elapsed_{0.0};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__ACTION_TRANSITION_HPP_
