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

#ifndef AI_SAPIENS_MUJOCO__VIEWER_SCENE_STYLING_HPP_
#define AI_SAPIENS_MUJOCO__VIEWER_SCENE_STYLING_HPP_

#include <mujoco/mujoco.h>

namespace ai_sapiens_mujoco
{

inline constexpr int kVisualGeomGroup = 2;
inline constexpr int kCollisionGeomGroup = 3;
inline constexpr float kCenterOfMassVisualAlpha = 0.25F;

/// Makes robot visual meshes translucent while center-of-mass display is active.
///
/// Only model geoms in the K1 visual group are changed. Collision geoms, the
/// floor, gantry, and MuJoCo decoration geoms retain their original opacity.
void apply_center_of_mass_visual_style(
  const mjModel * model, const mjvOption & option, mjvScene * scene);

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__VIEWER_SCENE_STYLING_HPP_
