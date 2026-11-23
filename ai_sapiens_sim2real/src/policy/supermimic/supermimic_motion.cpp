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
// Author: Woojin Wie

#include "ai_sapiens_sim2real/policy/supermimic/supermimic_motion.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"

namespace ai_sapiens_sim2real
{

SuperMimicMotion::SuperMimicMotion(
  std::shared_ptr<MotionReference> reference, double fps,
  const std::vector<std::string> & joints)
: reference_(std::move(reference))
{
  if (!reference_ || reference_->num_frames() < 2 || !std::isfinite(fps) || fps <= 0) {
    throw std::runtime_error("SuperMimic requires at least two finite-rate motion frames");
  }
  const auto indices = make_source_indices_for_target(
    reference_->joint_order(), joints, "motion joints", "policy joints");
  MotionReference cursor = *reference_;
  frames_.resize(cursor.num_frames());

  for (size_t i = 0; i < frames_.size(); ++i) {
    cursor.seek(static_cast<double>(i) / fps);
    auto & frame = frames_[i];
    frame.rotation = cursor.root_quaternion().toRotationMatrix();
    const auto position = cursor.joint_pos();
    const auto velocity = cursor.joint_vel();
    frame.joint_position.resize(joints.size());
    frame.joint_velocity.resize(joints.size());
    for (size_t j = 0; j < joints.size(); ++j) {
      frame.joint_position[j] = position[indices[j]];
      frame.joint_velocity[j] = velocity[indices[j]];
    }
    if (!frame.rotation.allFinite() || !frame.joint_position.allFinite() ||
      !frame.joint_velocity.allFinite())
    {
      throw std::runtime_error("Nonfinite SuperMimic reference frame");
    }
  }
}

}  // namespace ai_sapiens_sim2real
