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
#include "ai_sapiens_mujoco/mujoco_simulation.hpp"
#include <string>
#include <vector>
#include <cstdint>

namespace ai_sapiens_mujoco
{

enum class ViewerUiAction
{
  kNone = 0,
  kRaiseGantry,
  kLowerGantry,
  kAttachGantry,
  kReleaseGantry,
  kReset, kPause, kAlign, kReadyPose, kDamping, kVelocity, kMimic,
  kStopVelocity,
  kSetFriction, kResetFriction, kSetPayload, kResetPayload,
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

struct ViewerPolicyOption { uint16_t code; std::string name; };
struct ViewerControlState {
  bool connected{false}, paused{false}, busy{false};
  std::string mode{"Disconnected"}, note{"Start with --sim --gui"};
  std::string velocity_asset;
  std::vector<ViewerPolicyOption> motions;
  int selected_motion{0};
  float velocity[3]{0, 0, 0};  // Normalized commands, as with keyboard teleop.
};

/// Dear ImGui shell matching the robotis_mujoco viewer.
class MujocoViewerUi
{
public:
  void initialize(mjvOption * option, bool gantry_present);
  void shutdown();
  void update_physics(const PhysicsSettings & settings) { physics_ = settings; friction_ = settings.friction; }
  float requested_friction() const { return friction_; }
  float requested_payload() const { return payload_; }
  void set_controls(ViewerControlState * controls) { controls_ = controls; }
  void resize(int width, int height, mjrContext * context);
  ViewerUiResult handle_event(const ViewerUiEvent & event, const mjrContext * context);
  void update_status(double fps, int contacts, bool force, const char * body,
    bool attached, double height, const mjrContext * context);
  void render(const mjrContext * context);
  ViewerUiAction take_action();
  mjrRect scene_viewport() const;
  bool cursor_is_in_scene() const;
private:
  mjvOption * option_{nullptr};
  bool initialized_{false}, gantry_{false}, attached_{false};
  bool left_{true}, right_{true}, help_{false}, info_{true};
  int width_{0}, height_{0}, contacts_{0};
  double fps_{0}, gantry_height_{0};
  ViewerUiAction pending_{ViewerUiAction::kNone};
  void * bold_{nullptr};
  ViewerControlState * controls_{nullptr};
  PhysicsSettings physics_;
  float friction_{0}, payload_{0};
  bool velocity_panel_{true}, mimic_panel_{true};
};
}  // namespace ai_sapiens_mujoco
#endif
