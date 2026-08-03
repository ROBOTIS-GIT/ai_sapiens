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

#include "ai_sapiens_mujoco/center_of_mass_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ai_sapiens_mujoco
{
namespace
{

int find_free_root_body(const mjModel * model)
{
  for (int body_id = 1; body_id < model->nbody; ++body_id) {
    const int joint_end =
      model->body_jntadr[body_id] + model->body_jntnum[body_id];
    for (int joint_id = model->body_jntadr[body_id];
      joint_id < joint_end; ++joint_id)
    {
      if (model->jnt_type[joint_id] == mjJNT_FREE) {
        return body_id;
      }
    }
  }
  return -1;
}

bool is_descendant_of(
  const mjModel * model, int body_id, int ancestor_body_id)
{
  for (int current = body_id; current > 0;
    current = model->body_parentid[current])
  {
    if (current == ancestor_body_id) {
      return true;
    }
  }
  return false;
}

bool append_sphere(
  const mjtNum * position, float radius,
  const float * color,
  int object_type, int object_id, mjvScene * scene)
{
  if (scene->ngeom >= scene->maxgeom) {
    return false;
  }

  const mjtNum size[3]{radius, radius, radius};
  mjvGeom & marker = scene->geoms[scene->ngeom++];
  mjv_initGeom(
    &marker, mjGEOM_SPHERE, size, position, nullptr, color);
  marker.category = mjCAT_DECOR;
  marker.objtype = object_type;
  marker.objid = object_id;
  marker.transparent = color[3] < 1.0F;
  return true;
}

}  // namespace

float link_center_of_mass_radius(
  mjtNum body_mass, mjtNum minimum_body_mass, mjtNum maximum_body_mass)
{
  if (body_mass <= 0.0 || minimum_body_mass <= 0.0 ||
    maximum_body_mass < minimum_body_mass)
  {
    return kLinkCenterOfMassMinimumRadius;
  }

  const float minimum_mass_scale =
    std::cbrt(static_cast<float>(minimum_body_mass));
  const float maximum_mass_scale =
    std::cbrt(static_cast<float>(maximum_body_mass));
  const float mass_scale = std::cbrt(static_cast<float>(body_mass));
  const float scale_range = maximum_mass_scale - minimum_mass_scale;
  const float normalized_mass =
    scale_range > std::numeric_limits<float>::epsilon() ?
    std::clamp(
      (mass_scale - minimum_mass_scale) / scale_range, 0.0F, 1.0F) :
    0.5F;
  return
    kLinkCenterOfMassMinimumRadius +
    normalized_mass *
    (kLinkCenterOfMassMaximumRadius - kLinkCenterOfMassMinimumRadius);
}

int append_center_of_mass_markers(
  const mjModel * model, const mjData * data,
  const mjvOption & option, mjvScene * scene)
{
  if (!model || !data || !scene || !option.flags[mjVIS_COM]) {
    return 0;
  }

  const int root_body_id = find_free_root_body(model);
  if (root_body_id < 0 || model->body_mass[root_body_id] <= 0.0) {
    return 0;
  }

  mjtNum minimum_body_mass = std::numeric_limits<mjtNum>::max();
  mjtNum maximum_body_mass = 0.0;
  for (int body_id = root_body_id; body_id < model->nbody; ++body_id) {
    if (model->body_mass[body_id] > 0.0 &&
      is_descendant_of(model, body_id, root_body_id))
    {
      minimum_body_mass =
        std::min(minimum_body_mass, model->body_mass[body_id]);
      maximum_body_mass =
        std::max(maximum_body_mass, model->body_mass[body_id]);
    }
  }

  int appended = 0;
  if (append_sphere(
      data->subtree_com + 3 * root_body_id,
      kRobotCenterOfMassRadius, model->vis.rgba.com,
      mjOBJ_UNKNOWN, -1, scene))
  {
    ++appended;
  } else {
    return appended;
  }

  for (int body_id = root_body_id;
    body_id < model->nbody && scene->ngeom < scene->maxgeom; ++body_id)
  {
    if (model->body_mass[body_id] <= 0.0 ||
      !is_descendant_of(model, body_id, root_body_id))
    {
      continue;
    }
    if (append_sphere(
        data->xipos + 3 * body_id,
        link_center_of_mass_radius(
          model->body_mass[body_id], minimum_body_mass, maximum_body_mass),
        kLinkCenterOfMassColor.data(),
        mjOBJ_BODY, body_id, scene))
    {
      ++appended;
    }
  }
  return appended;
}

}  // namespace ai_sapiens_mujoco
