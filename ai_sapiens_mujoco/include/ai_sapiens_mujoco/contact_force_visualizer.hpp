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

#ifndef AI_SAPIENS_MUJOCO__CONTACT_FORCE_VISUALIZER_HPP_
#define AI_SAPIENS_MUJOCO__CONTACT_FORCE_VISUALIZER_HPP_

#include <mujoco/mujoco.h>

#include <array>

namespace ai_sapiens_mujoco
{

struct ContactNormalForceLine
{
  std::array<mjtNum, 3> start{};
  std::array<mjtNum, 3> end{};
  mjtNum magnitude{0.0};
};

/// Compute one contact's normal-force line in world coordinates.
///
/// The direction follows the force acting on a dynamic body when it contacts
/// the world. For other contact pairs, the normal is oriented toward the
/// positive world-Z hemisphere so floor-like contacts remain visually upright.
bool compute_contact_normal_force_line(
  const mjModel * model, const mjData * data, int contact_id,
  ContactNormalForceLine * line);

/// Append one lime line geom per active contact to an already-updated scene.
/// Returns the number of lines appended.
int append_contact_normal_force_lines(
  const mjModel * model, const mjData * data, mjvScene * scene);

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__CONTACT_FORCE_VISUALIZER_HPP_
