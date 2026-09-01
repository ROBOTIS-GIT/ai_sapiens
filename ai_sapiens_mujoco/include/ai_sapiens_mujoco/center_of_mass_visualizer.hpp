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

#ifndef AI_SAPIENS_MUJOCO__CENTER_OF_MASS_VISUALIZER_HPP_
#define AI_SAPIENS_MUJOCO__CENTER_OF_MASS_VISUALIZER_HPP_

#include <mujoco/mujoco.h>

#include <array>

namespace ai_sapiens_mujoco
{

inline constexpr std::array<float, 4> kRobotCenterOfMassColor{
  1.0F, 0.0F, 0.0F, 1.0F};
inline constexpr float kRobotCenterOfMassRadius = 0.055F;
inline constexpr float kLinkCenterOfMassMinimumRadius = 0.014F;
inline constexpr float kLinkCenterOfMassMaximumRadius = 0.048F;
inline constexpr std::array<float, 4> kLinkCenterOfMassColor{
  0.35F, 0.80F, 1.0F, 0.50F};

/// Maps link mass to a visually distinguishable radius within a robot.
float link_center_of_mass_radius(
  mjtNum body_mass, mjtNum minimum_body_mass, mjtNum maximum_body_mass);

/// Appends one whole-robot CoM marker and one marker for each link CoM.
///
/// The whole-robot marker uses the free-floating root body's subtree CoM.
/// Link markers use each descendant body's inertial-frame position. Their
/// radii span the configured visual range according to relative cube-root mass
/// within the robot subtree. Bodies outside that subtree, such as the gantry,
/// are excluded.
int append_center_of_mass_markers(
  const mjModel * model, const mjData * data,
  const mjvOption & option, mjvScene * scene);

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__CENTER_OF_MASS_VISUALIZER_HPP_
