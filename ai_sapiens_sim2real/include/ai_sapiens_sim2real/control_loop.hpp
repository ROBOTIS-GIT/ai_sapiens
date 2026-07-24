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
// Author: Woojin Wie, Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__CONTROL_LOOP_HPP_
#define AI_SAPIENS_SIM2REAL__CONTROL_LOOP_HPP_

#include <cinttypes>
#include <cstdio>
#include <cstdint>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ai_sapiens_sim2real/controllers/mode_controller.hpp"
#include "ai_sapiens_sim2real/controllers/policy_controller.hpp"
#include "ai_sapiens_sim2real/interfaces/command_publisher_base.hpp"
#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief Manages sensor handles and controllers with realtime update loop.
 *
 * Similar to ros2_control's ControllerManager, this class:
 * - Owns the shared SharedControlData
 * - Manages registration of sensor handles and controllers
 * - Waits for all required sensors to be ready before starting inference
 * - Runs the realtime control loop with ordered updates
 *
 * Update order: sensors first (in registration order), then controllers.
 */
class ControlLoop
{
public:
  /**
   * @brief Construct control loop.
   * @param node ROS node for clock and logging
   * @param command_publish_rate Command publish frequency in Hz
   */
  ControlLoop(rclcpp::Node::SharedPtr node, float command_publish_rate);
  ~ControlLoop();

  /**
   * @brief Register a sensor handle.
   *
   * Sensor handles are updated in registration order before controllers.
   * Must be called before start().
   *
   * @param handle Sensor handle to register
   * @param required If true, control loop waits for this sensor before running inference
   */
  void add_sensor_handle(std::shared_ptr<SensorHandleBase> handle, bool required = true);

  /**
   * @brief Register the two controllers that form the decide/act stages.
   *
   * The control step runs ModeController (decide) then PolicyController (act)
   * on every tick. Both must be set before start().
   */
  void set_mode_controller(std::shared_ptr<ModeController> controller);
  void set_policy_controller(std::shared_ptr<PolicyController> controller);

  /**
   * @brief Register a command publisher.
   *
   * Command publishers are called in registration order after all controllers.
   * Must be called before start().
   *
   * @param publisher Command publisher to register
   */
  void add_command_publisher(std::shared_ptr<CommandPublisherBase> publisher);

  /**
   * @brief Get pointer to shared state data.
   *
   * Use this to pass shared_data to sensor handles and controllers.
   *
   * @return Pointer to SharedControlData (owned by ControlLoop)
   */
  SharedControlData * get_shared_data()
  {
    return &shared_data_;
  }

  /**
   * @brief Set RT thread priority.
   * @param priority SCHED_FIFO priority (0-99)
   */
  void set_thread_priority(int priority);

  /**
   * @brief Enable/disable memory locking.
   * @param lock True to lock memory
   */
  void set_lock_memory(bool lock);

  /**
   * @brief Set timeout for waiting for sensors to be ready.
   * @param timeout_seconds Timeout in seconds (0 = no timeout)
   */
  void set_wait_for_ready_timeout(double timeout_seconds);

  void set_detailed_status_log(bool enabled);
  void set_status_log_enabled(bool enabled);
  void set_debug_publish_enabled(bool enabled);
  void set_debug_publish_rate(double rate_hz);

  /**
   * @brief Start the realtime control loop.
   */
  void start();

  /**
   * @brief Stop the realtime control loop.
   */
  void stop();

  /**
   * @brief Check if control loop is running.
   * @return True if running
   */
  bool is_running() const
  {
    return running_;
  }

  /**
   * @brief Whether the RT thread aborted (uncaught exception or sensor-init
   *        failure). Lets the caller exit with a failure status instead of 0.
   */
  bool faulted() const
  {
    return faulted_;
  }

  /**
   * @brief Check if all required sensors are ready.
   * @return True if all required sensors have received data
   */
  bool all_sensors_ready() const;

private:
  /**
   * @brief Wait for all required sensors to be ready.
   * @return True if all sensors ready, false if timeout
  */
  bool wait_for_sensors_ready();
  std::vector<std::string> not_ready_required_sensors() const;
  static std::string make_waiting_for_message(const std::vector<std::string> & names);

  // RT thread lifecycle.
  void rt_thread_func();
  void prepare_realtime_thread();
  bool initialize_controllers_after_sensor_ready();

  // One control-loop tick. Each stage names what it does to SharedControlData:
  //   sense   -> sensor handles fill sensors/teleop/api inputs
  //   decide  -> ModeController picks the mode and selects the velocity source
  //   act     -> PolicyController computes the joint command (output)
  //   command -> publisher sends output to the hardware
  void run_control_step(
    const rclcpp::Time & current_time,
    const rclcpp::Duration & measured_period,
    int iteration,
    int observation_publish_interval);
  void sense(const rclcpp::Time & current_time);
  // decide returns the mode decision it produced; act consumes it as a const
  // reference (to the pre-allocated mode block, so no allocation), which is what
  // keeps the act stage from rewriting the decision.
  const ModeDecision & decide(
    const rclcpp::Time & current_time, const rclcpp::Duration & measured_period);
  void act(
    const ModeDecision & decision,
    const rclcpp::Time & current_time,
    const rclcpp::Duration & measured_period);
  void command(const rclcpp::Time & current_time);
  void publish_emergency_damping();
  void publish_debug_topics_if_needed(
    int iteration,
    int observation_publish_interval);
  void warn_sensors_not_ready();
  void log_status_if_needed(int iteration);
  void sleep_until_next_tick(
    std::chrono::steady_clock::time_point & next_time,
    const std::chrono::nanoseconds & command_period);
  const char * behavior_kind_name(BehaviorKind mode) const;

  // Low-rate status/debug output kept outside the core update sequence.
  struct StatusSnapshot
  {
    BehaviorKind active_behavior_kind{BehaviorKind::Damping};
    std::string active_state_name;
    std::string requested_mode_name;
    uint64_t transition_count{0};
    bool inference_enabled{false};
    bool damping_requested{false};
    bool action_limit_exceeded{false};

    bool operator!=(const StatusSnapshot & other) const
    {
      return active_behavior_kind != other.active_behavior_kind ||
             active_state_name != other.active_state_name ||
             requested_mode_name != other.requested_mode_name ||
             transition_count != other.transition_count ||
             inference_enabled != other.inference_enabled ||
             damping_requested != other.damping_requested ||
             action_limit_exceeded != other.action_limit_exceeded;
    }
  };

  StatusSnapshot make_status_snapshot() const;
  void log_state(int iteration, const char * reason);
  void log_basic_status(int iteration, const char * reason);
  void log_detailed_status();
  static std::string format_vec3(float x, float y, float z);
  template<typename VectorT>
  static std::string format_vector_preview(const VectorT & vec, size_t n)
  {
    std::string s = "[";
    const size_t count = std::min(n, static_cast<size_t>(vec.size()));
    for (size_t i = 0; i < count; ++i) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%.3f", static_cast<float>(vec[i]));
      s += buf;
      if (i + 1 < count) {
        s += ", ";
      }
    }
    if (static_cast<size_t>(vec.size()) > count) {
      char buf[32];
      snprintf(buf, sizeof(buf), ", ... +%zu", static_cast<size_t>(vec.size()) - count);
      s += buf;
    }

    s += "]";
    return s;
  }

  void publish_raw_debug_topics();

  struct SensorEntry
  {
    std::shared_ptr<SensorHandleBase> handle;
    bool required;
  };

  rclcpp::Node::SharedPtr node_;

  // Loop timing and RT setup.
  float command_publish_rate_;
  double debug_publish_rate_;
  int thread_priority_;
  bool lock_memory_;
  double wait_for_ready_timeout_;

  // Logging/debug controls.
  bool detailed_status_log_{false};
  bool status_log_enabled_{false};
  bool debug_publish_enabled_{false};

  // Sensor handles and publishers are homogeneous, so they stay as lists run in
  // insertion order; the two controllers are named because they are the
  // distinct decide/act stages of the control step.
  std::vector<SensorEntry> sensor_handles_;
  std::shared_ptr<ModeController> mode_controller_;
  std::shared_ptr<PolicyController> policy_controller_;
  std::vector<std::shared_ptr<CommandPublisherBase>> command_publishers_;

  // Shared state and status cache.
  SharedControlData shared_data_;
  StatusSnapshot last_status_snapshot_;
  bool last_status_snapshot_valid_{false};

  // Optional debug publishers.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr raw_observation_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr processed_action_publisher_;

  // Worker thread control.
  std::thread rt_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> faulted_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONTROL_LOOP_HPP_
