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

#ifndef AI_SAPIENS_MUJOCO__MUJOCO_VIEWER_HPP_
#define AI_SAPIENS_MUJOCO__MUJOCO_VIEWER_HPP_

#include <mujoco/mujoco.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "ai_sapiens_mujoco/mujoco_simulation.hpp"

struct GLFWwindow;

namespace ai_sapiens_mujoco
{

/// Interactive GLFW viewer running in its own thread.
///
/// The on-screen Raise/Lower buttons and Up/Down keys nudge the gantry target
/// height by +/-0.02 m at 0.2 m/s. Attach/Release buttons and A/R keys toggle
/// the weld. The diagnostics overlay exposes contact points/forces, inertia
/// boxes, center of mass, and FPS. Mouse: left drag rotates, right drag pans,
/// Ctrl+right drag applies an external force, and scroll zooms.
class MujocoViewer
{
public:
  explicit MujocoViewer(std::shared_ptr<MujocoSimulation> sim);
  ~MujocoViewer();

  /// Launch the render thread. No-op if already started.
  void start();

  /// Ask the render thread to exit and join it.
  void stop();

private:
  void run();

  static void key_callback(GLFWwindow * window, int key, int scancode, int action, int mods);
  static void mouse_button_callback(GLFWwindow * window, int button, int action, int mods);
  static void cursor_pos_callback(GLFWwindow * window, double xpos, double ypos);
  static void scroll_callback(GLFWwindow * window, double xoffset, double yoffset);

  void handle_key(int key, int action);
  void handle_mouse_button(GLFWwindow * window, int button, int action);
  void handle_cursor_pos(GLFWwindow * window, double xpos, double ypos);
  void handle_scroll(double yoffset);
  bool handle_overlay_click(GLFWwindow * window);
  bool handle_visualization_control_click(int x, int y);
  bool handle_gantry_control_click(int x, int y);
  bool handle_external_force_button(GLFWwindow * window, int button, int action);
  bool begin_external_force_drag(GLFWwindow * window);
  void update_external_force_drag(GLFWwindow * window, double dx, double dy, int height);
  void end_external_force_drag();
  void apply_external_force_locked();
  void toggle_visualization_flag(int flag);
  void update_frame_rate();
  void update_visualization_control_layout(
    int framebuffer_width, int framebuffer_height);
  void render_visualization_controls(
    int framebuffer_width, int framebuffer_height);
  void update_gantry_control_layout(int framebuffer_width, int framebuffer_height);
  void render_gantry_controls(int framebuffer_width, int framebuffer_height);
  void nudge_gantry(double delta_m);

  std::shared_ptr<MujocoSimulation> sim_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;
  mjvScene scn_;
  mjrContext con_;
  mjrRect viewer_status_rect_{};
  mjrRect external_force_help_rect_{};
  std::array<mjrRect, 4> visualization_control_rects_{};
  mjrRect gantry_status_rect_{};
  mjrRect gantry_raise_rect_{};
  mjrRect gantry_lower_rect_{};
  mjrRect gantry_attach_rect_{};
  mjrRect gantry_release_rect_{};

  // Mouse interaction state (viewer thread only).
  bool button_left_{false};
  bool button_middle_{false};
  bool button_right_{false};
  bool external_force_dragging_{false};
  int external_force_body_id_{0};
  double lastx_{0.0};
  double lasty_{0.0};

  // Render diagnostics (viewer thread only).
  std::chrono::steady_clock::time_point fps_sample_start_{};
  int fps_sample_frames_{0};
  double frames_per_second_{0.0};
  int contact_count_{0};
};

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__MUJOCO_VIEWER_HPP_
