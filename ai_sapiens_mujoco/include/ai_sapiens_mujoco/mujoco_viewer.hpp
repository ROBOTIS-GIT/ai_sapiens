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

#include <atomic>
#include <memory>
#include <thread>

#include "ai_sapiens_mujoco/mujoco_simulation.hpp"

struct GLFWwindow;

namespace ai_sapiens_mujoco
{

/// Interactive GLFW viewer running in its own thread.
///
/// Key bindings: Up/Down nudge the gantry target height by +/-0.02 m at
/// 0.2 m/s, R releases the gantry. Mouse: left drag rotates, right drag
/// pans, scroll zooms (canonical MuJoCo sample handlers).
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
  void handle_mouse_button(GLFWwindow * window);
  void handle_cursor_pos(GLFWwindow * window, double xpos, double ypos);
  void handle_scroll(double yoffset);

  std::shared_ptr<MujocoSimulation> sim_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;

  // Mouse interaction state (viewer thread only).
  bool button_left_{false};
  bool button_middle_{false};
  bool button_right_{false};
  double lastx_{0.0};
  double lasty_{0.0};
};

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__MUJOCO_VIEWER_HPP_
