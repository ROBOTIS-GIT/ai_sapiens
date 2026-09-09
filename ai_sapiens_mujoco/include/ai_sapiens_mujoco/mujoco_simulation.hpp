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

#ifndef AI_SAPIENS_MUJOCO__MUJOCO_SIMULATION_HPP_
#define AI_SAPIENS_MUJOCO__MUJOCO_SIMULATION_HPP_

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace ai_sapiens_mujoco
{

struct JointCommand
{
  double position{0.0};
  double feedforward{0.0};
  double kp{0.0};
  double kd{0.0};
};

struct JointState
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct ImuState
{
  std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};  // w, x, y, z
  std::array<double, 3> gyro{};
  std::array<double, 3> accel{};
};

struct PhysicsSettings {
  bool floor_available{false}, payload_available{false};
  double friction{0}, nominal_friction{0}, nominal_mass{0}, current_mass{0};
  std::string base_body;
};

class MujocoSimulation
{
public:
  ~MujocoSimulation();

  /// Load one MuJoCo scene and index the given joints.
  ///
  /// Throws std::runtime_error when loading or validation fails, and
  /// std::logic_error if a scene is already loaded. A failed first load leaves
  /// the simulation reusable.
  void load(
    const std::string & scene_path,
    const std::vector<std::string> & joint_names);

  /// Call once before stepping; shifts the floating base and gantry mocap.
  /// Throws std::invalid_argument when pelvis_z is not finite.
  void set_hang_height(double pelvis_z);

  void set_command(std::size_t joint_index, const JointCommand & cmd);
  JointState joint_state(std::size_t joint_index) const;
  ImuState imu_state() const;

  /// Steps the simulation with an accumulator; thread-safe.
  void advance(double dt_seconds);

  double sim_time() const;
  /// Restore the initial scene (including configured hang height), clearing forces/commands.
  void reset();
  void set_paused(bool paused);
  bool paused() const;
  PhysicsSettings physics_settings() const;
  void set_floor_friction(double coefficient);
  void restore_floor_friction();
  void set_payload(double additional_mass);
  void restore_payload();

  /// Viewer render lock.
  std::mutex & mutex();

  const mjModel * model() const;
  mjData * data();

  // Gantry control (only meaningful when the scene contains a gantry).
  bool gantry_present() const;
  bool gantry_attached() const;
  bool gantry_set_target(double height_m, double speed_mps);
  bool gantry_attach();
  bool gantry_release();
  double gantry_height() const;

private:
  void cache_physics_settings();  // caller holds mutex_
  int floor_id_{-1}, payload_body_{-1};
  PhysicsSettings physics_;
  std::array<mjtNum, 3> nominal_floor_{};
  std::vector<int> floor_pairs_, floor_feet_;
  std::vector<std::array<mjtNum, 5>> nominal_pairs_;
  std::vector<std::array<mjtNum, 3>> nominal_feet_;
  void apply_control();   // caller holds mutex_
  void update_gantry();   // caller holds mutex_

  mjModel * model_{nullptr};
  mjData * data_{nullptr};
  mjData * initial_data_{nullptr};
  bool paused_{false};
  std::vector<mjtNum> initial_eq_data_;
  mutable std::mutex mutex_;
  double accumulator_{0.0};
  std::vector<int> qpos_adr_, qvel_adr_, act_id_;
  std::vector<JointCommand> commands_;
  int imu_quat_adr_{-1}, imu_gyro_adr_{-1}, imu_acc_adr_{-1};
  int gantry_body_{-1}, gantry_mocap_{-1}, gantry_eq_{-1}, gantry_robot_body_{-1};
  int robot_free_qpos_adr_{-1};
  bool gantry_released_{false};
  double gantry_target_z_{0.0}, gantry_speed_{0.0};
  std::array<mjtNum, 3> gantry_to_base_pos_{};
  std::array<mjtNum, 3> gantry_attach_pos_{};
  std::array<mjtNum, 4> gantry_to_base_quat_{1.0, 0.0, 0.0, 0.0};
  std::array<mjtNum, 4> gantry_upright_quat_{1.0, 0.0, 0.0, 0.0};
};

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__MUJOCO_SIMULATION_HPP_
