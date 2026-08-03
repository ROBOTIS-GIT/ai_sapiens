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
#include <cstdio>
#include <memory>
#include <mutex>
#include <utility>

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
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
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

  while (!glfwWindowShouldClose(window) && running_) {
    {
      std::lock_guard<std::mutex> lock(sim_->mutex());
      mjv_updateScene(
        sim_->model(), sim_->data(), &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    }
    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn_, &con_);
    render_gantry_controls(viewport.width, viewport.height);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

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
    default:
      break;
  }
}

// Canonical mouse handlers from MuJoCo's basic.cc sample.
void MujocoViewer::handle_mouse_button(GLFWwindow * window, int button, int action)
{
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && sim_->gantry_present()) {
    double cursor_x = 0.0;
    double cursor_y = 0.0;
    int window_width = 0;
    int window_height = 0;
    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetCursorPos(window, &cursor_x, &cursor_y);
    glfwGetWindowSize(window, &window_width, &window_height);
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

    if (window_width > 0 && window_height > 0) {
      update_gantry_control_layout(framebuffer_width, framebuffer_height);
      const int x = static_cast<int>(
        cursor_x * static_cast<double>(framebuffer_width) / window_width);
      const int y = static_cast<int>(
        (window_height - cursor_y) * static_cast<double>(framebuffer_height) / window_height);

      if (contains(gantry_raise_rect_, x, y)) {
        nudge_gantry(kGantryNudgeMeters);
        button_left_ = false;
        return;
      }
      if (contains(gantry_lower_rect_, x, y)) {
        nudge_gantry(-kGantryNudgeMeters);
        button_left_ = false;
        return;
      }
      if (contains(gantry_attach_rect_, x, y)) {
        sim_->gantry_attach();
        button_left_ = false;
        return;
      }
      if (contains(gantry_release_rect_, x, y)) {
        sim_->gantry_release();
        button_left_ = false;
        return;
      }
    }
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
  if (!button_left_ && !button_middle_ && !button_right_) {
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
