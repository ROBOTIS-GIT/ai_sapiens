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

#include "ai_sapiens_mujoco/contact_force_visualizer.hpp"

#include <algorithm>

namespace ai_sapiens_mujoco
{

namespace
{

constexpr mjtNum kMinimumVisibleForce = 1.0e-6;
constexpr mjtNum kMinimumArrowWidth = 1.0e-3;
constexpr mjtNum kMaximumArrowExtentFraction = 0.5;

bool geom_belongs_to_world(const mjModel * model, int geom_id)
{
  return geom_id >= 0 && geom_id < model->ngeom &&
         model->geom_bodyid[geom_id] == 0;
}

bool geom_belongs_to_dynamic_body(const mjModel * model, int geom_id)
{
  return geom_id >= 0 && geom_id < model->ngeom &&
         model->geom_bodyid[geom_id] > 0;
}

}  // namespace

bool compute_contact_normal_force_arrow(
  const mjModel * model, const mjData * data, int contact_id,
  ContactNormalForceArrow * arrow)
{
  if (!model || !data || !arrow || contact_id < 0 || contact_id >= data->ncon) {
    return false;
  }

  const mjContact & contact = data->contact[contact_id];
  if (contact.efc_address < 0) {
    return false;
  }

  mjtNum contact_force[6]{};
  mj_contactForce(model, data, contact_id, contact_force);
  const mjtNum normal_force = contact_force[0];
  if (normal_force <= kMinimumVisibleForce) {
    return false;
  }

  mjtNum direction[3]{
    contact.frame[0],
    contact.frame[1],
    contact.frame[2],
  };

  // MuJoCo's contact normal points from geom[0] to geom[1]. Point the arrow
  // toward the dynamic body for world contacts; otherwise prefer upward.
  const bool flip_for_world_contact =
    geom_belongs_to_dynamic_body(model, contact.geom[0]) &&
    geom_belongs_to_world(model, contact.geom[1]);
  const bool neither_side_is_world =
    !geom_belongs_to_world(model, contact.geom[0]) &&
    !geom_belongs_to_world(model, contact.geom[1]);
  if (flip_for_world_contact || (neither_side_is_world && direction[2] < 0.0)) {
    mju_scl3(direction, direction, -1.0);
  }

  const mjtNum maximum_length = std::max(
    static_cast<mjtNum>(model->stat.meansize),
    kMaximumArrowExtentFraction * model->stat.extent);
  const mjtNum arrow_length = std::min(
    normal_force * static_cast<mjtNum>(model->vis.map.force),
    maximum_length);
  if (arrow_length <= 0.0) {
    return false;
  }

  arrow->magnitude = normal_force;
  for (int axis = 0; axis < 3; ++axis) {
    arrow->start[axis] = contact.pos[axis];
    arrow->end[axis] = contact.pos[axis] + arrow_length * direction[axis];
  }
  return true;
}

int append_contact_normal_force_arrows(
  const mjModel * model, const mjData * data, mjvScene * scene)
{
  if (!model || !data || !scene) {
    return 0;
  }

  const mjtNum arrow_width = std::max(
    kMinimumArrowWidth,
    static_cast<mjtNum>(model->vis.scale.forcewidth) * model->stat.meansize);
  int appended = 0;
  for (int contact_id = 0;
    contact_id < data->ncon && scene->ngeom < scene->maxgeom;
    ++contact_id)
  {
    ContactNormalForceArrow arrow;
    if (!compute_contact_normal_force_arrow(model, data, contact_id, &arrow)) {
      continue;
    }

    mjvGeom & geom = scene->geoms[scene->ngeom++];
    mjv_initGeom(
      &geom, mjGEOM_ARROW, nullptr, nullptr, nullptr,
      model->vis.rgba.contactforce);
    mjv_connector(
      &geom, mjGEOM_ARROW, arrow_width, arrow.start.data(), arrow.end.data());
    geom.category = mjCAT_DECOR;
    geom.emission = 0.25f;
    ++appended;
  }
  return appended;
}

}  // namespace ai_sapiens_mujoco
