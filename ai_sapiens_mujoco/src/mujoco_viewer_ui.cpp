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

#include "ai_sapiens_mujoco/mujoco_viewer_ui.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "ai_sapiens_mujoco/viewer_scene_styling.hpp"

namespace ai_sapiens_mujoco
{

namespace
{

constexpr int kRaiseGantryUserId =
  static_cast<int>(ViewerUiAction::kRaiseGantry);
constexpr int kLowerGantryUserId =
  static_cast<int>(ViewerUiAction::kLowerGantry);
constexpr int kAttachGantryUserId =
  static_cast<int>(ViewerUiAction::kAttachGantry);
constexpr int kReleaseGantryUserId =
  static_cast<int>(ViewerUiAction::kReleaseGantry);

void copy_ui_text(char * destination, const char * source)
{
  std::snprintf(destination, mjMAXUINAME, "%s", source ? source : "");
}

}  // namespace

void MujocoViewerUi::initialize(mjvOption * option, bool gantry_present)
{
  if (initialized_ || !option) {
    return;
  }

  gantry_present_ = gantry_present;
  ui_.spacing = mjui_themeSpacing(0);
  ui_.color = mjui_themeColor(3);
  ui_.rectid = kPanelRectId;
  ui_.auxid = 0;

  // Keep the neutral native theme, with a restrained teal accent for active
  // controls. This remains legible on both the default and bright scenes.
  const float panel_background[3]{0.08f, 0.09f, 0.10f};
  const float section_top[3]{0.10f, 0.26f, 0.28f};
  const float section_bottom[3]{0.07f, 0.15f, 0.17f};
  const float section_foreground[3]{0.90f, 0.94f, 0.95f};
  const float active_control[3]{0.12f, 0.46f, 0.40f};
  const float button_color[3]{0.18f, 0.21f, 0.23f};
  std::copy(panel_background, panel_background + 3, ui_.color.master);
  std::copy(panel_background, panel_background + 3, ui_.color.sectpane);
  std::copy(section_top, section_top + 3, ui_.color.secttitle);
  std::copy(section_bottom, section_bottom + 3, ui_.color.secttitle2);
  std::copy(section_foreground, section_foreground + 3, ui_.color.sectfont);
  std::copy(section_foreground, section_foreground + 3, ui_.color.sectsymbol);
  std::copy(active_control, active_control + 3, ui_.color.check);
  std::copy(button_color, button_color + 3, ui_.color.button);

  const mjuiDef common_definitions[] = {
    {mjITEM_SECTION, "Diagnostics", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_STATIC, "Render FPS", 1, nullptr, "0.0", 0},
    {mjITEM_STATIC, "Contacts", 1, nullptr, "0", 0},
    {mjITEM_STATIC, "Force tool", 1, nullptr, "Ctrl+RMB drag", 0},

    {mjITEM_SECTION, "Visualization", mjSECT_OPEN, nullptr, "", 0},
    {
      mjITEM_CHECKBYTE, "Visual meshes", 1,
      option->geomgroup + kVisualGeomGroup, " V", 0
    },
    {
      mjITEM_CHECKBYTE, "Collision", 1,
      option->geomgroup + kCollisionGeomGroup, " G", 0
    },
    {
      mjITEM_CHECKBYTE, "Contact points", 1,
      option->flags + mjVIS_CONTACTPOINT, " C", 0
    },
    {
      mjITEM_CHECKBYTE, "Contact forces", 1,
      option->flags + mjVIS_CONTACTFORCE, " F", 0
    },
    {
      mjITEM_CHECKBYTE, "Inertia boxes", 1,
      option->flags + mjVIS_INERTIA, " I", 0
    },
    {
      mjITEM_CHECKBYTE, "Robot + link CoM", 1,
      option->flags + mjVIS_COM, " M", 0
    },
    {mjITEM_END, "", 0, nullptr, "", 0}
  };
  mjui_add(&ui_, common_definitions);

  if (gantry_present_) {
    const mjuiDef gantry_definitions[] = {
      {mjITEM_SECTION, "Gantry", mjSECT_OPEN, nullptr, "", 0},
      {mjITEM_STATIC, "State", 1, nullptr, "Attached", 0},
      {mjITEM_BUTTON, "Raise  +2 cm", 1, nullptr, "", kRaiseGantryUserId},
      {mjITEM_BUTTON, "Lower  -2 cm", 1, nullptr, "", kLowerGantryUserId},
      {mjITEM_BUTTON, "Attach robot", 1, nullptr, " A", kAttachGantryUserId},
      {mjITEM_BUTTON, "Release robot", 1, nullptr, " R", kReleaseGantryUserId},
      {mjITEM_END, "", 0, nullptr, "", 0}
    };
    mjui_add(&ui_, gantry_definitions);
  }

  state_.nrect = 3;
  initialized_ = true;
}

void MujocoViewerUi::resize(
  int framebuffer_width, int framebuffer_height, mjrContext * context)
{
  if (!initialized_ || !context || framebuffer_width <= 0 || framebuffer_height <= 0) {
    return;
  }
  if (framebuffer_width_ == framebuffer_width &&
    framebuffer_height_ == framebuffer_height &&
    context->auxFBO[ui_.auxid] != 0)
  {
    return;
  }

  framebuffer_width_ = framebuffer_width;
  framebuffer_height_ = framebuffer_height;
  state_.rect[0] = {0, 0, framebuffer_width, framebuffer_height};

  mjui_resize(&ui_, context);
  const int aux_id = ui_.auxid;
  if (context->auxFBO[aux_id] == 0 ||
    context->auxFBO_r[aux_id] == 0 ||
    context->auxColor[aux_id] == 0 ||
    context->auxColor_r[aux_id] == 0 ||
    context->auxWidth[aux_id] != ui_.width ||
    context->auxHeight[aux_id] != ui_.maxheight ||
    context->auxSamples[aux_id] != ui_.spacing.samples)
  {
    mjr_addAux(
      aux_id, ui_.width, ui_.maxheight, ui_.spacing.samples, context);
  }

  state_.rect[kPanelRectId] = {
    0, 0, std::min(ui_.width, framebuffer_width), framebuffer_height};
  state_.rect[kSceneRectId] = {
    state_.rect[kPanelRectId].width, 0,
    std::max(0, framebuffer_width - state_.rect[kPanelRectId].width),
    framebuffer_height};
  state_.dragbutton = mjBUTTON_NONE;
  state_.dragrect = 0;
  mjui_update(-1, -1, &ui_, &state_, context);
}

ViewerUiResult MujocoViewerUi::handle_event(
  const ViewerUiEvent & event, const mjrContext * context)
{
  ViewerUiResult result;
  if (!initialized_ || !context) {
    return result;
  }

  update_pointer_state(event);
  state_.type = event.type;
  state_.sx = event.sx;
  state_.sy = event.sy;
  state_.key = event.key;
  state_.keytime = event.time;

  if (event.type == mjEVENT_PRESS) {
    state_.doubleclick =
      event.button == state_.button && event.time - state_.buttontime < 0.25;
    state_.button = event.button;
    state_.buttontime = event.time;
    if (state_.mouserect != 0) {
      state_.dragbutton = state_.button;
      state_.dragrect = state_.mouserect;
    }
  } else if (event.type == mjEVENT_RELEASE) {
    state_.button = event.button;
  }

  const bool directed_to_panel =
    state_.dragrect == ui_.rectid ||
    (state_.dragrect == 0 && state_.mouserect == ui_.rectid) ||
    event.type == mjEVENT_KEY;
  if (directed_to_panel) {
    mjuiItem * item = mjui_event(&ui_, &state_, context);
    if (event.type == mjEVENT_KEY) {
      result.handled = item != nullptr || state_.key == 0;
    } else {
      result.handled =
        state_.dragrect == ui_.rectid ||
        state_.mouserect == ui_.rectid;
    }
    result.action = action_from_item(item);
  }

  if (event.type == mjEVENT_RELEASE) {
    state_.dragrect = 0;
    state_.dragbutton = mjBUTTON_NONE;
  }
  return result;
}

void MujocoViewerUi::update_status(
  double frames_per_second, int contact_count,
  bool external_force_active, const char * external_force_body_name,
  bool gantry_attached, double gantry_height,
  const mjrContext * context)
{
  if (!initialized_ || !context) {
    return;
  }

  char value[mjMAXUINAME]{};
  std::snprintf(value, sizeof(value), "%.1f", frames_per_second);
  set_static_value(kDiagnosticsSection, 0, value, context);

  std::snprintf(value, sizeof(value), "%d", contact_count);
  set_static_value(kDiagnosticsSection, 1, value, context);

  if (external_force_active) {
    std::snprintf(
      value, sizeof(value), "Pushing: %.28s",
      external_force_body_name ? external_force_body_name : "unnamed");
  } else {
    std::snprintf(value, sizeof(value), "Ctrl+RMB drag");
  }
  set_static_value(kDiagnosticsSection, 2, value, context);

  if (!gantry_present_) {
    return;
  }

  if (gantry_attached) {
    std::snprintf(value, sizeof(value), "ON  %.3f m", gantry_height);
  } else {
    std::snprintf(value, sizeof(value), "OFF  Released");
  }
  set_static_value(kGantrySection, 0, value, context);
  update_gantry_button_states(gantry_attached, context);
}

void MujocoViewerUi::render(const mjrContext * context)
{
  if (initialized_ && context && framebuffer_width_ > 0 && framebuffer_height_ > 0) {
    mjui_render(&ui_, &state_, context);
  }
}

mjrRect MujocoViewerUi::scene_viewport() const
{
  return state_.rect[kSceneRectId];
}

bool MujocoViewerUi::cursor_is_in_scene() const
{
  return state_.mouserect == kSceneRectId;
}

void MujocoViewerUi::update_pointer_state(const ViewerUiEvent & event)
{
  state_.left = event.left;
  state_.right = event.right;
  state_.middle = event.middle;
  state_.control = event.control;
  state_.shift = event.shift;
  state_.alt = event.alt;
  state_.dx = event.dx;
  state_.dy = event.dy;
  state_.x = event.x;
  state_.y = event.y;
  state_.mouserect =
    mjr_findRect(
    static_cast<int>(std::lround(event.x)),
    static_cast<int>(std::lround(event.y)),
    state_.nrect - 1, state_.rect + 1) + 1;
}

void MujocoViewerUi::set_static_value(
  int section, int item, const char * value, const mjrContext * context)
{
  mjuiItem & ui_item = ui_.sect[section].item[item];
  if (ui_item.multi.nelem == 1 &&
    std::strncmp(ui_item.multi.name[0], value, mjMAXUINAME) == 0)
  {
    return;
  }

  ui_item.multi.nelem = 1;
  copy_ui_text(ui_item.multi.name[0], value);
  mjui_update(section, item, &ui_, &state_, context);
}

void MujocoViewerUi::update_gantry_button_states(
  bool gantry_attached, const mjrContext * context)
{
  if (gantry_state_initialized_ && gantry_attached_ == gantry_attached) {
    return;
  }
  gantry_state_initialized_ = true;
  gantry_attached_ = gantry_attached;

  ui_.sect[kGantrySection].item[1].state = gantry_attached ? 1 : 0;
  ui_.sect[kGantrySection].item[2].state = gantry_attached ? 1 : 0;
  ui_.sect[kGantrySection].item[3].state = gantry_attached ? 0 : 1;
  ui_.sect[kGantrySection].item[4].state = gantry_attached ? 1 : 0;
  mjui_update(kGantrySection, -1, &ui_, &state_, context);
}

ViewerUiAction MujocoViewerUi::action_from_item(const mjuiItem * item)
{
  if (!item || item->userid < kRaiseGantryUserId ||
    item->userid > kReleaseGantryUserId)
  {
    return ViewerUiAction::kNone;
  }
  return static_cast<ViewerUiAction>(item->userid);
}

}  // namespace ai_sapiens_mujoco
