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

#ifndef AI_SAPIENS_MUJOCO__MUJOCO_VIEWER_UI_HPP_
#define AI_SAPIENS_MUJOCO__MUJOCO_VIEWER_UI_HPP_

#include <mujoco/mujoco.h>

namespace ai_sapiens_mujoco
{

enum class ViewerUiAction
{
  kNone = 0,
  kRaiseGantry,
  kLowerGantry,
  kAttachGantry,
  kReleaseGantry,
};

struct ViewerUiEvent
{
  int type{mjEVENT_NONE};
  int button{mjBUTTON_NONE};
  double x{0.0};
  double y{0.0};
  double dx{0.0};
  double dy{0.0};
  double sx{0.0};
  double sy{0.0};
  bool left{false};
  bool right{false};
  bool middle{false};
  bool control{false};
  bool shift{false};
  bool alt{false};
  int key{0};
  double time{0.0};
};

struct ViewerUiResult
{
  bool handled{false};
  ViewerUiAction action{ViewerUiAction::kNone};
};

/// MuJoCo-native side panel for viewer diagnostics and controls.
///
/// This class owns all mjUI state so the viewer's camera and perturbation
/// handlers only receive events from the 3D viewport.
class MujocoViewerUi
{
public:
  void initialize(mjvOption * option, bool gantry_present);
  void resize(int framebuffer_width, int framebuffer_height, mjrContext * context);

  ViewerUiResult handle_event(
    const ViewerUiEvent & event, const mjrContext * context);

  void update_status(
    double frames_per_second, int contact_count,
    bool external_force_active, const char * external_force_body_name,
    bool gantry_attached, double gantry_height,
    const mjrContext * context);

  void render(const mjrContext * context);

  [[nodiscard]] mjrRect scene_viewport() const;
  [[nodiscard]] bool cursor_is_in_scene() const;

private:
  static constexpr int kPanelRectId = 1;
  static constexpr int kSceneRectId = 2;
  static constexpr int kDiagnosticsSection = 0;
  static constexpr int kVisualizationSection = 1;
  static constexpr int kGantrySection = 2;

  void update_pointer_state(const ViewerUiEvent & event);
  void set_static_value(
    int section, int item, const char * value, const mjrContext * context);
  void update_gantry_button_states(
    bool gantry_attached, const mjrContext * context);
  static ViewerUiAction action_from_item(const mjuiItem * item);

  mjUI ui_{};
  mjuiState state_{};
  bool initialized_{false};
  bool gantry_present_{false};
  bool gantry_state_initialized_{false};
  bool gantry_attached_{false};
  int framebuffer_width_{0};
  int framebuffer_height_{0};
};

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__MUJOCO_VIEWER_UI_HPP_
