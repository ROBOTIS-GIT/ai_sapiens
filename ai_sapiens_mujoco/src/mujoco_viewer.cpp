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

#include "ai_sapiens_mujoco/mujoco_viewer.hpp"

#include <GLFW/glfw3.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <utility>

#include "ai_sapiens_mujoco/contact_force_visualizer.hpp"
#include "ai_sapiens_mujoco/viewer_scene_styling.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ai_sapiens_mujoco
{

namespace
{

constexpr double kGantryNudgeMeters = 0.02;
constexpr double kGantrySpeedMetersPerSecond = 0.2;

rclcpp::Logger viewer_logger()
{
  return rclcpp::get_logger("mujoco_viewer");
}

int to_mujoco_button(int glfw_button)
{
  switch (glfw_button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      return mjBUTTON_LEFT;
    case GLFW_MOUSE_BUTTON_RIGHT:
      return mjBUTTON_RIGHT;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      return mjBUTTON_MIDDLE;
    default:
      return mjBUTTON_NONE;
  }
}

bool modifier_pressed(GLFWwindow * window, int left_key, int right_key)
{
  return
    glfwGetKey(window, left_key) == GLFW_PRESS ||
    glfwGetKey(window, right_key) == GLFW_PRESS;
}

}  // namespace

MujocoViewer::MujocoViewer(std::shared_ptr<MujocoSimulation> sim)
: sim_(std::move(sim))
{
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);

  // Always draw the selected point and mouse-spring force while perturbing.
  opt_.flags[mjVIS_PERTFORCE] = 1;
  opt_.flags[mjVIS_PERTOBJ] = 1;
  opt_.flags[mjVIS_SELECT] = 1;
}

MujocoViewer::~MujocoViewer()
{
  stop();
}

void MujocoViewer::start()
{
  if (thread_.joinable()) {
    return;
  }
  running_ = true;
  thread_ = std::thread([this] {run();});
}

void MujocoViewer::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void MujocoViewer::run()
{
  if (!glfwInit()) {
    RCLCPP_ERROR(viewer_logger(), "Failed to initialize GLFW; viewer disabled");
    return;
  }

  GLFWwindow * window =
    glfwCreateWindow(1200, 900, "AI Sapiens K1 - MuJoCo", nullptr, nullptr);
  if (!window) {
    RCLCPP_ERROR(viewer_logger(), "Failed to create GLFW window; viewer disabled");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  cam_.lookat[0] = 0.0;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 0.8;
  cam_.distance = 2.5;
  cam_.elevation = -20.0;

  mjv_makeScene(sim_->model(), &scn_, 2000);
  mjr_makeContext(sim_->model(), &con_, mjFONTSCALE_150);
  ui_.initialize(&opt_, sim_->gantry_present());

  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  ui_.resize(framebuffer_width, framebuffer_height, &con_);

  glfwSetWindowUserPointer(window, this);
  glfwSetKeyCallback(window, &MujocoViewer::key_callback);
  glfwSetMouseButtonCallback(window, &MujocoViewer::mouse_button_callback);
  glfwSetCursorPosCallback(window, &MujocoViewer::cursor_pos_callback);
  glfwSetScrollCallback(window, &MujocoViewer::scroll_callback);

  fps_sample_start_ = std::chrono::steady_clock::now();
  while (!glfwWindowShouldClose(window) && running_) {
    {
      std::lock_guard<std::mutex> lock(sim_->mutex());
      apply_external_force_locked();
      contact_count_ = sim_->data()->ncon;
      mjvOption scene_options = opt_;
      const bool show_contact_forces =
        scene_options.flags[mjVIS_CONTACTFORCE] != 0;
      scene_options.flags[mjVIS_CONTACTFORCE] = 0;
      mjv_updateScene(
        sim_->model(), sim_->data(), &scene_options, &pert_, &cam_,
        mjCAT_ALL, &scn_);
      apply_center_of_mass_visual_style(
        sim_->model(), scene_options, &scn_);
      if (show_contact_forces) {
        append_contact_normal_force_lines(
          sim_->model(), sim_->data(), &scn_);
      }
    }
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
    ui_.resize(framebuffer_width, framebuffer_height, &con_);
    const mjrRect viewport = ui_.scene_viewport();
    if (viewport.width > 0 && viewport.height > 0) {
      mjr_render(viewport, &scn_, &con_);
    }

    const char * external_force_body_name = nullptr;
    if (external_force_dragging_ && external_force_body_id_ > 0) {
      external_force_body_name =
        mj_id2name(sim_->model(), mjOBJ_BODY, external_force_body_id_);
    }
    const bool gantry_attached =
      sim_->gantry_present() && sim_->gantry_attached();
    const double gantry_height =
      sim_->gantry_present() ? sim_->gantry_height() : 0.0;
    ui_.update_status(
      frames_per_second_, contact_count_,
      external_force_dragging_, external_force_body_name,
      gantry_attached, gantry_height, &con_);
    ui_.render(&con_);
    glfwSwapBuffers(window);
    glfwPollEvents();
    update_frame_rate();
  }

  end_external_force_drag();
  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);
  glfwDestroyWindow(window);
  glfwTerminate();
}

void MujocoViewer::key_callback(
  GLFWwindow * window, int key, int /*scancode*/, int action, int /*mods*/)
{
  auto * viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (viewer) {
    viewer->handle_key(window, key, action);
  }
}

void MujocoViewer::mouse_button_callback(
  GLFWwindow * window, int button, int action, int /*mods*/)
{
  auto * viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (viewer) {
    viewer->handle_mouse_button(window, button, action);
  }
}

void MujocoViewer::cursor_pos_callback(GLFWwindow * window, double xpos, double ypos)
{
  auto * viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (viewer) {
    viewer->handle_cursor_pos(window, xpos, ypos);
  }
}

void MujocoViewer::scroll_callback(
  GLFWwindow * window, double xoffset, double yoffset)
{
  auto * viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (viewer) {
    viewer->handle_scroll(window, xoffset, yoffset);
  }
}

void MujocoViewer::handle_key(GLFWwindow * window, int key, int action)
{
  if (action == GLFW_PRESS) {
    double cursor_x = 0.0;
    double cursor_y = 0.0;
    glfwGetCursorPos(window, &cursor_x, &cursor_y);
    ViewerUiEvent event = make_pointer_event(
      window, mjEVENT_KEY, mjBUTTON_NONE, cursor_x, cursor_y);
    event.key = key;
    const ViewerUiResult ui_result = ui_.handle_event(event, &con_);
    apply_ui_action(ui_result.action);
    if (ui_result.handled) {
      return;
    }
  }

  if (action != GLFW_PRESS && action != GLFW_REPEAT) {
    return;
  }
  switch (key) {
    case GLFW_KEY_UP:
      nudge_gantry(kGantryNudgeMeters);
      break;
    case GLFW_KEY_DOWN:
      nudge_gantry(-kGantryNudgeMeters);
      break;
    default:
      break;
  }
}

// Canonical mouse handlers from MuJoCo's basic.cc sample.
void MujocoViewer::handle_mouse_button(GLFWwindow * window, int button, int action)
{
  double cursor_x = 0.0;
  double cursor_y = 0.0;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  const int event_type =
    action == GLFW_PRESS ? mjEVENT_PRESS : mjEVENT_RELEASE;
  const ViewerUiResult ui_result = ui_.handle_event(
    make_pointer_event(
      window, event_type, to_mujoco_button(button), cursor_x, cursor_y),
    &con_);
  lastx_ = cursor_x;
  lasty_ = cursor_y;
  apply_ui_action(ui_result.action);
  if (ui_result.handled) {
    button_left_ = false;
    button_middle_ = false;
    button_right_ = false;
    return;
  }

  if (handle_external_force_button(window, button, action)) {
    button_right_ = false;
    return;
  }

  // Update button state.
  button_left_ =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  button_middle_ =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  button_right_ =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

  // Update mouse position.
  glfwGetCursorPos(window, &lastx_, &lasty_);
}

void MujocoViewer::handle_cursor_pos(GLFWwindow * window, double xpos, double ypos)
{
  const bool any_button_down =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ||
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS ||
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  if (!any_button_down && !external_force_dragging_)
  {
    lastx_ = xpos;
    lasty_ = ypos;
    return;
  }

  // Compute mouse displacement, save.
  const double dx = xpos - lastx_;
  const double dy = ypos - lasty_;
  const ViewerUiResult ui_result = ui_.handle_event(
    make_pointer_event(
      window, mjEVENT_MOVE, mjBUTTON_NONE, xpos, ypos),
    &con_);
  lastx_ = xpos;
  lasty_ = ypos;
  apply_ui_action(ui_result.action);
  if (ui_result.handled) {
    return;
  }

  if (!button_left_ && !button_middle_ && !button_right_ &&
    !external_force_dragging_)
  {
    return;
  }

  // Get current window size.
  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);
  if (height <= 0) {
    return;
  }

  if (external_force_dragging_) {
    update_external_force_drag(window, dx, dy, height);
    return;
  }

  // Get shift key state.
  const bool mod_shift =
    glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

  // Determine action based on mouse button.
  mjtMouse mouse_action;
  if (button_right_) {
    mouse_action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left_) {
    mouse_action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    mouse_action = mjMOUSE_ZOOM;
  }

  // Move camera.
  mjv_moveCamera(
    sim_->model(), mouse_action, dx / height, dy / height, &scn_, &cam_);
}

void MujocoViewer::handle_scroll(
  GLFWwindow * window, double xoffset, double yoffset)
{
  double cursor_x = 0.0;
  double cursor_y = 0.0;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  const ViewerUiResult ui_result = ui_.handle_event(
    make_pointer_event(
      window, mjEVENT_SCROLL, mjBUTTON_NONE,
      cursor_x, cursor_y, xoffset, yoffset),
    &con_);
  apply_ui_action(ui_result.action);
  if (ui_result.handled) {
    return;
  }

  // Emulate vertical mouse motion = 5% of window height.
  mjv_moveCamera(sim_->model(), mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn_, &cam_);
}

ViewerUiEvent MujocoViewer::make_pointer_event(
  GLFWwindow * window, int event_type, int button,
  double xpos, double ypos, double xoffset, double yoffset) const
{
  int window_width = 0;
  int window_height = 0;
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

  ViewerUiEvent event;
  event.type = event_type;
  event.button = button;
  if (window_width > 0 && window_height > 0) {
    const double scale_x =
      static_cast<double>(framebuffer_width) / window_width;
    const double scale_y =
      static_cast<double>(framebuffer_height) / window_height;
    event.x = xpos * scale_x;
    event.y = (window_height - ypos) * scale_y;
    event.dx = (xpos - lastx_) * scale_x;
    event.dy = (lasty_ - ypos) * scale_y;
    event.sx = xoffset * scale_x;
    event.sy = yoffset * scale_y;
  }

  event.left =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  event.middle =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  event.right =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  event.control = modifier_pressed(
    window, GLFW_KEY_LEFT_CONTROL, GLFW_KEY_RIGHT_CONTROL);
  event.shift = modifier_pressed(
    window, GLFW_KEY_LEFT_SHIFT, GLFW_KEY_RIGHT_SHIFT);
  event.alt = modifier_pressed(
    window, GLFW_KEY_LEFT_ALT, GLFW_KEY_RIGHT_ALT);
  event.time = glfwGetTime();
  return event;
}

void MujocoViewer::apply_ui_action(ViewerUiAction action)
{
  switch (action) {
    case ViewerUiAction::kRaiseGantry:
      nudge_gantry(kGantryNudgeMeters);
      break;
    case ViewerUiAction::kLowerGantry:
      nudge_gantry(-kGantryNudgeMeters);
      break;
    case ViewerUiAction::kAttachGantry:
      sim_->gantry_attach();
      break;
    case ViewerUiAction::kReleaseGantry:
      sim_->gantry_release();
      break;
    case ViewerUiAction::kNone:
      break;
  }
}

bool MujocoViewer::handle_external_force_button(
  GLFWwindow * window, int button, int action)
{
  if (button != GLFW_MOUSE_BUTTON_RIGHT) {
    return false;
  }

  if (action == GLFW_RELEASE && external_force_dragging_) {
    end_external_force_drag();
    return true;
  }

  const bool control_pressed =
    glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
    glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
  if (action == GLFW_PRESS && control_pressed) {
    return begin_external_force_drag(window);
  }
  return false;
}

bool MujocoViewer::begin_external_force_drag(GLFWwindow * window)
{
  double cursor_x = 0.0;
  double cursor_y = 0.0;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  const ViewerUiEvent pointer = make_pointer_event(
    window, mjEVENT_NONE, mjBUTTON_NONE, cursor_x, cursor_y);
  const mjrRect viewport = ui_.scene_viewport();
  if (viewport.width <= 0 || viewport.height <= 0 ||
    pointer.x < viewport.left ||
    pointer.x >= viewport.left + viewport.width ||
    pointer.y < viewport.bottom ||
    pointer.y >= viewport.bottom + viewport.height)
  {
    return false;
  }

  const mjtNum relative_x =
    (pointer.x - viewport.left) / static_cast<mjtNum>(viewport.width);
  const mjtNum relative_y =
    (pointer.y - viewport.bottom) / static_cast<mjtNum>(viewport.height);
  const mjtNum aspect_ratio =
    static_cast<mjtNum>(viewport.width) / viewport.height;
  mjtNum selection_point[3]{};
  int geom_id = -1;
  int flex_id = -1;
  int skin_id = -1;

  std::lock_guard<std::mutex> lock(sim_->mutex());
  const int body_id = mjv_select(
    sim_->model(), sim_->data(), &opt_, aspect_ratio, relative_x, relative_y,
    &scn_, selection_point, &geom_id, &flex_id, &skin_id);
  if (body_id <= 0) {
    return false;
  }

  pert_.select = body_id;
  pert_.flexselect = flex_id;
  pert_.skinselect = skin_id;
  mjtNum offset[3]{};
  mju_sub3(offset, selection_point, sim_->data()->xpos + 3 * body_id);
  mju_mulMatTVec(
    pert_.localpos, sim_->data()->xmat + 9 * body_id, offset, 3, 3);
  pert_.active = mjPERT_TRANSLATE;
  mjv_initPerturb(sim_->model(), sim_->data(), &scn_, &pert_);

  external_force_body_id_ = body_id;
  external_force_dragging_ = true;
  lastx_ = cursor_x;
  lasty_ = cursor_y;
  const char * body_name = mj_id2name(sim_->model(), mjOBJ_BODY, body_id);
  RCLCPP_INFO(
    viewer_logger(), "External force selected body: %s",
    body_name ? body_name : "(unnamed body)");
  return true;
}

void MujocoViewer::update_external_force_drag(
  GLFWwindow * window, double dx, double dy, int height)
{
  const bool shift_pressed =
    glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  const mjtMouse mouse_action =
    shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;

  std::lock_guard<std::mutex> lock(sim_->mutex());
  mjv_movePerturb(
    sim_->model(), sim_->data(), mouse_action, dx / height, dy / height,
    &scn_, &pert_);
}

void MujocoViewer::end_external_force_drag()
{
  if (!external_force_dragging_) {
    return;
  }

  std::lock_guard<std::mutex> lock(sim_->mutex());
  if (external_force_body_id_ > 0 && external_force_body_id_ < sim_->model()->nbody) {
    mju_zero(sim_->data()->xfrc_applied + 6 * external_force_body_id_, 6);
  }
  pert_.active = 0;
  pert_.select = 0;
  pert_.flexselect = -1;
  pert_.skinselect = -1;
  external_force_dragging_ = false;
  external_force_body_id_ = 0;
}

void MujocoViewer::apply_external_force_locked()
{
  if (external_force_dragging_) {
    mjv_applyPerturbForce(sim_->model(), sim_->data(), &pert_);
  }
}

void MujocoViewer::update_frame_rate()
{
  ++fps_sample_frames_;
  const auto now = std::chrono::steady_clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - fps_sample_start_).count();
  if (elapsed >= 0.5) {
    frames_per_second_ = fps_sample_frames_ / elapsed;
    fps_sample_frames_ = 0;
    fps_sample_start_ = now;
  }
}

void MujocoViewer::nudge_gantry(double delta_m)
{
  sim_->gantry_set_target(
    sim_->gantry_height() + delta_m, kGantrySpeedMetersPerSecond);
}

}  // namespace ai_sapiens_mujoco
