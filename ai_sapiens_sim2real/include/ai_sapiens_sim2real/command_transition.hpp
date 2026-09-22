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

#ifndef AI_SAPIENS_SIM2REAL__COMMAND_TRANSITION_HPP_
#define AI_SAPIENS_SIM2REAL__COMMAND_TRANSITION_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "ai_sapiens_sim2real/joint_command.hpp"

namespace ai_sapiens_sim2real
{

// One output-stage transition, shared by all policies. No inference, behavior
// selection or publishing here. All storage is allocated at construction.
class CommandTransition
{
public:
  explicit CommandTransition(size_t robot_joint_count)
  {
    start_.resize(robot_joint_count);
    joints_.reserve(robot_joint_count);
  }

  // Snapshot a complete source command before modifying output. Targets and
  // joints are in policy order; source/output are in robot order. As before,
  // entry installs gains immediately, but position waits for valid inference.
  void begin(
    const JointCommand & source, const JointCommand & target,
    const std::vector<size_t> & joints, double duration, JointCommand & output)
  {
    if (!std::isfinite(duration) || duration < 0.0) {
      throw std::runtime_error("Command transition duration must be finite and non-negative");
    }
    if (joints.size() > start_.position.size() ||
      target.position.size() != joints.size() ||
      target.stiffness.size() != joints.size() || target.damping.size() != joints.size())
    {
      throw std::runtime_error("Command transition joint count mismatch");
    }
    for (size_t j = 0; j < joints.size(); ++j) {
      start_.position[j] = source.position.at(joints[j]);
      start_.stiffness[j] = source.stiffness.at(joints[j]);
      start_.damping[j] = source.damping.at(joints[j]);
    }
    joints_.assign(joints.begin(), joints.end());
    duration_ = duration;
    elapsed_ = 0.0;
    target_ready_ = false;
    for (size_t j = 0; j < joints_.size(); ++j) {
      output.stiffness[joints_[j]] = blend(start_.stiffness[j], target.stiffness[j]);
      output.damping[joints_[j]] = blend(start_.damping[j], target.damping[j]);
    }
  }

  void reset()
  {
    duration_ = 0.0;
    elapsed_ = 0.0;
    target_ready_ = false;
    joints_.clear();
  }

  void update(
    double period, TargetUpdate result, const JointCommand & target, JointCommand & output)
  {
    // Use readiness from the preceding tick, before consuming this result.
    // This preserves time advancement on a failing inference tick and starts
    // the first accepted target at alpha=0, even after a delayed first inference.
    if (target_ready_ && std::isfinite(period) && period > 0.0) {
      elapsed_ = std::min(duration_, elapsed_ + period);
    }
    if (result == TargetUpdate::HoldOutput) {
      target_ready_ = false;
    } else if (result == TargetUpdate::TargetReady) {
      target_ready_ = true;
    }
    if (!target_ready_) {
      return;
    }
    for (size_t j = 0; j < joints_.size(); ++j) {
      const size_t index = joints_[j];
      output.position[index] = blend(start_.position[j], target.position[j]);
      output.stiffness[index] = blend(start_.stiffness[j], target.stiffness[j]);
      output.damping[index] = blend(start_.damping[j], target.damping[j]);
    }
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

  JointCommand start_;
  std::vector<size_t> joints_;
  double duration_{0.0};
  double elapsed_{0.0};
  bool target_ready_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__COMMAND_TRANSITION_HPP_
