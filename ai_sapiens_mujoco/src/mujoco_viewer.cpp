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

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <utility>

#include "ai_sapiens_mujoco/contact_force_visualizer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ai_sapiens_mujoco
{

namespace
{

constexpr double kGantryNudgeMeters = 0.02;
constexpr double kGantrySpeedMetersPerSecond = 0.2;
constexpr int kGantryControlWidth = 170;
constexpr int kGantryControlHeight = 36;
constexpr int kGantryControlMargin = 12;
constexpr int kGantryControlGap = 6;
constexpr int kViewerControlWidth = 210;
constexpr int kViewerControlHeight = 32;
constexpr int kViewerControlMargin = 12;
constexpr int kViewerControlGap = 6;

struct VisualizationControl
{
  int flag;
  const char * label;
};

constexpr std::array<VisualizationControl, 4> kVisualizationControls{{
  {mjVIS_CONTACTPOINT, "Contact points  [C]"},
  {mjVIS_CONTACTFORCE, "Contact forces  [F]"},
  {mjVIS_INERTIA, "Inertia boxes  [I]"},
  {mjVIS_COM, "Center of mass  [M]"},
}};

rclcpp::Logger viewer_logger()
{
  return rclcpp::get_logger("mujoco_viewer");
}

bool contains(const mjrRect & rect, int x, int y)
{
  return x >= rect.left && x < rect.left + rect.width &&
         y >= rect.bottom && y < rect.bottom + rect.height;
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
      if (show_contact_forces) {
        append_contact_normal_force_lines(
          sim_->model(), sim_->data(), &scn_);
      }
    }
    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn_, &con_);
    render_visualization_controls(viewport.width, viewport.height);
    render_gantry_controls(viewport.width, viewport.height);
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
    viewer->handle_key(key, action);
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
  GLFWwindow * window, double /*xoffset*/, double yoffset)
{
  auto * viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (viewer) {
    viewer->handle_scroll(yoffset);
  }
}

void MujocoViewer::handle_key(int key, int action)
{
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
    case GLFW_KEY_A:
      if (action == GLFW_PRESS) {
        sim_->gantry_attach();
      }
      break;
    case GLFW_KEY_R:
      if (action == GLFW_PRESS) {
        sim_->gantry_release();
      }
      break;
    case GLFW_KEY_C:
      if (action == GLFW_PRESS) {
        toggle_visualization_flag(mjVIS_CONTACTPOINT);
      }
      break;
    case GLFW_KEY_F:
      if (action == GLFW_PRESS) {
        toggle_visualization_flag(mjVIS_CONTACTFORCE);
      }
      break;
    case GLFW_KEY_I:
      if (action == GLFW_PRESS) {
        toggle_visualization_flag(mjVIS_INERTIA);
      }
      break;
    case GLFW_KEY_M:
      if (action == GLFW_PRESS) {
        toggle_visualization_flag(mjVIS_COM);
      }
      break;
    default:
      break;
  }
}

// Canonical mouse handlers from MuJoCo's basic.cc sample.
void MujocoViewer::handle_mouse_button(GLFWwindow * window, int button, int action)
{
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
    handle_overlay_click(window))
  {
    button_left_ = false;
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
  // No buttons down: nothing to do.
  if (!button_left_ && !button_middle_ && !button_right_ &&
    !external_force_dragging_)
  {
    return;
  }

  // Compute mouse displacement, save.
  const double dx = xpos - lastx_;
  const double dy = ypos - lasty_;
  lastx_ = xpos;
  lasty_ = ypos;

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

void MujocoViewer::handle_scroll(double yoffset)
{
  // Emulate vertical mouse motion = 5% of window height.
  mjv_moveCamera(sim_->model(), mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn_, &cam_);
}

bool MujocoViewer::handle_overlay_click(GLFWwindow * window)
{
  double cursor_x = 0.0;
  double cursor_y = 0.0;
  int window_width = 0;
  int window_height = 0;
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

  if (window_width <= 0 || window_height <= 0) {
    return false;
  }

  const int x = static_cast<int>(
    cursor_x * static_cast<double>(framebuffer_width) / window_width);
  const int y = static_cast<int>(
    (window_height - cursor_y) * static_cast<double>(framebuffer_height) / window_height);

  update_visualization_control_layout(framebuffer_width, framebuffer_height);
  if (handle_visualization_control_click(x, y)) {
    return true;
  }

  if (!sim_->gantry_present()) {
    return false;
  }
  update_gantry_control_layout(framebuffer_width, framebuffer_height);
  return handle_gantry_control_click(x, y);
}

bool MujocoViewer::handle_visualization_control_click(int x, int y)
{
  for (std::size_t i = 0; i < kVisualizationControls.size(); ++i) {
    if (contains(visualization_control_rects_[i], x, y)) {
      toggle_visualization_flag(kVisualizationControls[i].flag);
      return true;
    }
  }
  return false;
}

bool MujocoViewer::handle_gantry_control_click(int x, int y)
{
  if (contains(gantry_raise_rect_, x, y)) {
    nudge_gantry(kGantryNudgeMeters);
    return true;
  }
  if (contains(gantry_lower_rect_, x, y)) {
    nudge_gantry(-kGantryNudgeMeters);
    return true;
  }
  if (contains(gantry_attach_rect_, x, y)) {
    sim_->gantry_attach();
    return true;
  }
  if (contains(gantry_release_rect_, x, y)) {
    sim_->gantry_release();
    return true;
  }
  return false;
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
  int width = 0;
  int height = 0;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  glfwGetWindowSize(window, &width, &height);
  if (width <= 0 || height <= 0) {
    return false;
  }

  const mjtNum relative_x = cursor_x / static_cast<mjtNum>(width);
  const mjtNum relative_y = (height - cursor_y) / static_cast<mjtNum>(height);
  const mjtNum aspect_ratio = static_cast<mjtNum>(width) / height;
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

void MujocoViewer::toggle_visualization_flag(int flag)
{
  if (flag >= 0 && flag < mjNVISFLAG) {
    opt_.flags[flag] = !opt_.flags[flag];
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

void MujocoViewer::update_visualization_control_layout(
  int /*framebuffer_width*/, int framebuffer_height)
{
  const int left = kViewerControlMargin;
  int bottom = std::max(
    0, framebuffer_height - kViewerControlMargin - kViewerControlHeight);

  viewer_status_rect_ = {left, bottom, kViewerControlWidth, kViewerControlHeight};
  bottom = std::max(0, bottom - kViewerControlGap - kViewerControlHeight);
  external_force_help_rect_ = {left, bottom, kViewerControlWidth, kViewerControlHeight};
  for (auto & rect : visualization_control_rects_) {
    bottom = std::max(0, bottom - kViewerControlGap - kViewerControlHeight);
    rect = {left, bottom, kViewerControlWidth, kViewerControlHeight};
  }
}

void MujocoViewer::render_visualization_controls(
  int framebuffer_width, int framebuffer_height)
{
  update_visualization_control_layout(framebuffer_width, framebuffer_height);

  char status[80];
  std::snprintf(
    status, sizeof(status), "FPS  %5.1f    Contacts  %d",
    frames_per_second_, contact_count_);
  mjr_label(
    viewer_status_rect_, mjFONT_NORMAL, status,
    0.12f, 0.12f, 0.12f, 0.90f, 0.95f, 0.95f, 0.95f, &con_);

  char force_help[96];
  if (external_force_dragging_ && external_force_body_id_ > 0) {
    const char * body_name =
      mj_id2name(sim_->model(), mjOBJ_BODY, external_force_body_id_);
    std::snprintf(
      force_help, sizeof(force_help), "Pushing: %s",
      body_name ? body_name : "(unnamed body)");
  } else {
    std::snprintf(force_help, sizeof(force_help), "Ctrl + right drag: push body");
  }
  mjr_label(
    external_force_help_rect_, mjFONT_NORMAL, force_help,
    0.10f, 0.24f, 0.44f, 0.90f, 0.95f, 0.95f, 0.95f, &con_);

  for (std::size_t i = 0; i < kVisualizationControls.size(); ++i) {
    const bool active = opt_.flags[kVisualizationControls[i].flag] != 0;
    char label[64];
    std::snprintf(
      label, sizeof(label), "[%c]  %s",
      active ? 'x' : ' ', kVisualizationControls[i].label);
    const float brightness = active ? 1.0f : 0.55f;
    mjr_label(
      visualization_control_rects_[i], mjFONT_NORMAL, label,
      0.10f * brightness, 0.34f * brightness, 0.20f * brightness, 0.90f,
      0.95f * brightness, 0.95f * brightness, 0.95f * brightness, &con_);
  }
}

void MujocoViewer::update_gantry_control_layout(
  int framebuffer_width, int framebuffer_height)
{
  const int left = std::max(0, framebuffer_width - kGantryControlWidth - kGantryControlMargin);
  int bottom = std::max(
    0, framebuffer_height - kGantryControlMargin - kGantryControlHeight);

  gantry_status_rect_ = {left, bottom, kGantryControlWidth, kGantryControlHeight};
  bottom = std::max(0, bottom - kGantryControlGap - kGantryControlHeight);
  gantry_raise_rect_ = {left, bottom, kGantryControlWidth, kGantryControlHeight};
  bottom = std::max(0, bottom - kGantryControlGap - kGantryControlHeight);
  gantry_lower_rect_ = {left, bottom, kGantryControlWidth, kGantryControlHeight};
  bottom = std::max(0, bottom - kGantryControlGap - kGantryControlHeight);
  gantry_attach_rect_ = {left, bottom, kGantryControlWidth, kGantryControlHeight};
  bottom = std::max(0, bottom - kGantryControlGap - kGantryControlHeight);
  gantry_release_rect_ = {left, bottom, kGantryControlWidth, kGantryControlHeight};
}

void MujocoViewer::render_gantry_controls(
  int framebuffer_width, int framebuffer_height)
{
  if (!sim_->gantry_present()) {
    return;
  }

  update_gantry_control_layout(framebuffer_width, framebuffer_height);
  const bool attached = sim_->gantry_attached();
  char status[64];
  if (attached) {
    std::snprintf(status, sizeof(status), "Gantry  %.3f m", sim_->gantry_height());
  } else {
    std::snprintf(status, sizeof(status), "Gantry  released");
  }

  mjr_label(
    gantry_status_rect_, mjFONT_NORMAL, status,
    0.12f, 0.12f, 0.12f, 0.90f, 0.95f, 0.95f, 0.95f, &con_);

  const float motion_active = attached ? 1.0f : 0.35f;
  mjr_label(
    gantry_raise_rect_, mjFONT_NORMAL, "Raise  +2 cm",
    0.10f * motion_active, 0.38f * motion_active, 0.18f * motion_active, 0.90f,
    0.95f * motion_active, 0.95f * motion_active, 0.95f * motion_active, &con_);
  mjr_label(
    gantry_lower_rect_, mjFONT_NORMAL, "Lower  -2 cm",
    0.38f * motion_active, 0.20f * motion_active, 0.10f * motion_active, 0.90f,
    0.95f * motion_active, 0.95f * motion_active, 0.95f * motion_active, &con_);

  const float attach_active = attached ? 0.35f : 1.0f;
  mjr_label(
    gantry_attach_rect_, mjFONT_NORMAL, "Attach robot  [A]",
    0.10f * attach_active, 0.26f * attach_active, 0.52f * attach_active, 0.90f,
    0.95f * attach_active, 0.95f * attach_active, 0.95f * attach_active, &con_);

  const float release_active = attached ? 1.0f : 0.35f;
  mjr_label(
    gantry_release_rect_, mjFONT_NORMAL, "Release robot  [R]",
    0.52f * release_active, 0.12f * release_active, 0.12f * release_active, 0.90f,
    0.95f * release_active, 0.95f * release_active, 0.95f * release_active, &con_);
}

void MujocoViewer::nudge_gantry(double delta_m)
{
  sim_->gantry_set_target(
    sim_->gantry_height() + delta_m, kGantrySpeedMetersPerSecond);
}

}  // namespace ai_sapiens_mujoco
