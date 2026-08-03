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

#include "ai_sapiens_mujoco/viewer_scene_styling.hpp"

#include <algorithm>

namespace ai_sapiens_mujoco
{

void apply_center_of_mass_visual_style(
  const mjModel * model, const mjvOption & option, mjvScene * scene)
{
  if (!model || !scene || !option.flags[mjVIS_COM]) {
    return;
  }

  for (int index = 0; index < scene->ngeom; ++index) {
    mjvGeom & scene_geom = scene->geoms[index];
    if (scene_geom.objtype != mjOBJ_GEOM ||
      scene_geom.objid < 0 || scene_geom.objid >= model->ngeom ||
      model->geom_group[scene_geom.objid] != kVisualGeomGroup)
    {
      continue;
    }
    scene_geom.rgba[3] =
      std::min(scene_geom.rgba[3], kCenterOfMassVisualAlpha);
  }
}

}  // namespace ai_sapiens_mujoco
