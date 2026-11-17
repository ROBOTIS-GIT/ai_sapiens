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

// Author: Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__POLICY__MIMIC_HEADING_ALIGNMENT_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MIMIC_HEADING_ALIGNMENT_HPP_

#include <cmath>
#include <Eigen/Geometry>

namespace ai_sapiens_sim2real
{
// Best world-Z rotation aligning two torso orientations. Unlike subtracting
// their Euler yaws, this remains defined when both forward axes are vertical
// (supine/prone). Keep roll/pitch tracking error; do not align full attitude.
inline Eigen::Quaternionf mimic_heading_alignment(
  const Eigen::Quaternionf & reference, const Eigen::Quaternionf & actual)
{
  const Eigen::Quaternionf delta = actual.normalized() * reference.normalized().conjugate();
  const float norm = std::hypot(delta.w(), delta.z());
  // Opposing tilt makes yaw unobservable. Deterministic identity is preferable
  // to amplifying noise; such a pose is not a valid matching entry posture.
  if (norm < 1.0e-6f) {
    return Eigen::Quaternionf::Identity();
  }
  return Eigen::Quaternionf(delta.w() / norm, 0.0f, 0.0f, delta.z() / norm);
}
}  // namespace ai_sapiens_sim2real

#endif
