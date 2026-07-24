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

#include "ai_sapiens_sim2real/control_loop.hpp"

namespace ai_sapiens_sim2real
{

ControlLoop::ControlLoop(
  rclcpp::Node::SharedPtr node,
  float command_publish_rate)
: node_(node)
  , command_publish_rate_(command_publish_rate)
  , debug_publish_rate_(50.0)
  , thread_priority_(50)
  , lock_memory_(true)
  , wait_for_ready_timeout_(30.0)  // 30 second default timeout
{
  if (command_publish_rate_ <= 0.0f) {
    throw std::runtime_error("ControlLoop command publish rate must be positive");
  }
}

ControlLoop::~ControlLoop()
{
  stop();
}

void ControlLoop::add_sensor_handle(std::shared_ptr<SensorHandleBase> handle, bool required)
{
  sensor_handles_.push_back({handle, required});
  RCLCPP_INFO(
    node_->get_logger(),
    "Registered sensor handle: %s (%s)",
    handle->get_name().c_str(),
    required ? "required" : "optional");
}

void ControlLoop::set_mode_controller(std::shared_ptr<ModeController> controller)
{
  mode_controller_ = std::move(controller);
  RCLCPP_INFO(
    node_->get_logger(),
    "Registered controller: %s",
    mode_controller_->get_name().c_str());
}

void ControlLoop::set_policy_controller(std::shared_ptr<PolicyController> controller)
{
  policy_controller_ = std::move(controller);
  RCLCPP_INFO(
    node_->get_logger(),
    "Registered controller: %s",
    policy_controller_->get_name().c_str());
}

void ControlLoop::add_command_publisher(std::shared_ptr<CommandPublisherBase> publisher)
{
  command_publishers_.push_back(publisher);
  RCLCPP_INFO(
    node_->get_logger(),
    "Registered command publisher: %s (%s)",
    publisher->get_name().c_str(),
    publisher->is_enabled() ? "enabled" : "disabled");
}

void ControlLoop::set_thread_priority(int priority)
{
  thread_priority_ = priority;
}

void ControlLoop::set_lock_memory(bool lock)
{
  lock_memory_ = lock;
}

void ControlLoop::set_wait_for_ready_timeout(double timeout_seconds)
{
  wait_for_ready_timeout_ = timeout_seconds;
}

void ControlLoop::set_detailed_status_log(bool enabled)
{
  detailed_status_log_ = enabled;
}

void ControlLoop::set_status_log_enabled(bool enabled)
{
  status_log_enabled_ = enabled;
}

void ControlLoop::set_debug_publish_enabled(bool enabled)
{
  debug_publish_enabled_ = enabled;
  if (!enabled) {
    raw_observation_publisher_.reset();
    processed_action_publisher_.reset();
    return;
  }

  if (!raw_observation_publisher_) {
    raw_observation_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/policy_input/raw_observation", 10);
    processed_action_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/policy_output/processed_action", 10);
    RCLCPP_INFO(
      node_->get_logger(),
      "Debug publishers enabled: /policy_input/raw_observation, "
      "/policy_output/processed_action");
  }
}

void ControlLoop::set_debug_publish_rate(double rate_hz)
{
  if (rate_hz <= 0.0) {
    throw std::runtime_error("Debug publish rate must be positive");
  }

  debug_publish_rate_ = rate_hz;
}

void ControlLoop::start()
{
  if (running_) {
    RCLCPP_WARN(node_->get_logger(), "Control loop already running");
    return;
  }

  running_ = true;
  rt_thread_ = std::thread(&ControlLoop::rt_thread_func, this);
  RCLCPP_INFO(node_->get_logger(), "Control loop started at %.1f Hz", command_publish_rate_);
}

void ControlLoop::stop()
{
  running_ = false;
  if (rt_thread_.joinable()) {
    rt_thread_.join();
    RCLCPP_INFO(node_->get_logger(), "Control loop stopped");
  }
}

bool ControlLoop::all_sensors_ready() const
{
  for (const auto & [handle, required] : sensor_handles_) {
    if (required && !handle->is_ready()) {
      return false;
    }
  }

  return true;
}

bool ControlLoop::wait_for_sensors_ready()
{
  RCLCPP_INFO(node_->get_logger(), "Waiting for sensors to be ready...");

  auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(wait_for_ready_timeout_);

  while (running_ && rclcpp::ok()) {
    const auto not_ready = not_ready_required_sensors();
    if (not_ready.empty()) {
      RCLCPP_INFO(node_->get_logger(), "All required sensors are ready!");
      return true;
    }

    const auto waiting_for = make_waiting_for_message(not_ready);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "Waiting for: %s",
      waiting_for.c_str());

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (wait_for_ready_timeout_ > 0 && elapsed > timeout) {
      RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for sensors: %s", waiting_for.c_str());
      return false;
    }

    // Sleep briefly before checking again
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;
}

std::vector<std::string> ControlLoop::not_ready_required_sensors() const
{
  std::vector<std::string> not_ready;
  for (const auto & [handle, required] : sensor_handles_) {
    if (required && !handle->is_ready()) {
      not_ready.push_back(handle->get_name());
    }
  }

  return not_ready;
}

std::string ControlLoop::make_waiting_for_message(const std::vector<std::string> & names)
{
  std::string waiting_for;
  for (size_t i = 0; i < names.size(); ++i) {
    waiting_for += names[i];
    if (i < names.size() - 1) {
      waiting_for += ", ";
    }
  }

  return waiting_for;
}

void ControlLoop::rt_thread_func()
{
  // Fatal boundary for unexpected RT-thread failures.
  try {
    prepare_realtime_thread();
    if (!initialize_controllers_after_sensor_ready()) {
      faulted_ = true;
      rclcpp::shutdown();
      return;
    }

    const auto command_period = std::chrono::nanoseconds(
      static_cast<int64_t>(1'000'000'000.0 / command_publish_rate_));
    auto next_time = std::chrono::steady_clock::now();
    auto previous_time = node_->get_clock()->now();
    int iteration = 0;
    const int observation_publish_interval =
      std::max(1, static_cast<int>(command_publish_rate_ / debug_publish_rate_));

    // Keep the order: sensors, controllers, commands, sleep.
    RCLCPP_INFO(node_->get_logger(), "Starting RT loop at %.1f Hz", command_publish_rate_);

    while (running_ && rclcpp::ok()) {
      auto current_time = node_->get_clock()->now();
      auto measured_period = current_time - previous_time;
      previous_time = current_time;

      run_control_step(current_time, measured_period, iteration, observation_publish_interval);
      log_status_if_needed(iteration);
      iteration++;
      sleep_until_next_tick(next_time, command_period);
    }

    RCLCPP_INFO(node_->get_logger(), "RT thread exiting after %d iterations", iteration);
  } catch (const std::exception & e) {
    running_ = false;
    faulted_ = true;
    RCLCPP_FATAL(node_->get_logger(), "RT loop aborted: %s; stopping control", e.what());
    publish_emergency_damping();
    rclcpp::shutdown();
  } catch (...) {
    running_ = false;
    faulted_ = true;
    RCLCPP_FATAL(node_->get_logger(), "RT loop aborted by unknown exception; stopping control");
    publish_emergency_damping();
    rclcpp::shutdown();
  }
}

// Last-gasp for the fatal path: without this, consumers keep applying the
// aborted loop's final command (stale pose at full stiffness) indefinitely.
void ControlLoop::publish_emergency_damping()
{
  try {
    if (mode_controller_) {
      mode_controller_->apply_emergency_damping();
    }
    command(node_->get_clock()->now());
  } catch (...) {
  }
}

void ControlLoop::prepare_realtime_thread()
{
  // Lock memory to prevent page faults.
  if (lock_memory_) {
    const auto lock_result = realtime_tools::lock_memory();
    if (!lock_result.first) {
      RCLCPP_WARN(node_->get_logger(), "Unable to lock memory: %s", lock_result.second.c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "Memory locked successfully");
    }
  }

  // Configure SCHED_FIFO for realtime scheduling.
  if (!realtime_tools::configure_sched_fifo(thread_priority_)) {
    RCLCPP_WARN(node_->get_logger(), "Could not enable FIFO RT scheduling policy");
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "Enabled FIFO RT scheduling with priority %d",
      thread_priority_);
  }
}

bool ControlLoop::initialize_controllers_after_sensor_ready()
{
  // Wait for all required sensors to be ready before starting inference.
  if (!wait_for_sensors_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize sensors, exiting");
    running_ = false;
    return false;
  }

  // Reset both controllers now that sensors are ready.
  const auto current_time = node_->get_clock()->now();
  sense(current_time);
  for (ControllerBase * controller : {
      static_cast<ControllerBase *>(mode_controller_.get()),
      static_cast<ControllerBase *>(policy_controller_.get())})
  {
    controller->reset();
    RCLCPP_INFO(node_->get_logger(), "Reset controller: %s", controller->get_name().c_str());
  }

  return true;
}

void ControlLoop::run_control_step(
  const rclcpp::Time & current_time,
  const rclcpp::Duration & measured_period,
  int iteration,
  int observation_publish_interval)
{
  // Readiness comes from sensor callbacks, not sense(), so gate before sensing.
  if (!all_sensors_ready()) {
    warn_sensors_not_ready();
    return;
  }

  sense(current_time);

  // decide() runs before act() so a transition into a policy enters and
  // infers on the same tick.
  const auto & decision = decide(current_time, measured_period);
  act(decision, current_time, measured_period);
  command(current_time);
  publish_debug_topics_if_needed(iteration, observation_publish_interval);
}

void ControlLoop::sense(const rclcpp::Time & current_time)
{
  for (auto & [handle, required] : sensor_handles_) {
    handle->update(current_time);
  }
}

const ModeDecision & ControlLoop::decide(
  const rclcpp::Time & current_time, const rclcpp::Duration & measured_period)
{
  mode_controller_->update(current_time, measured_period);
  return shared_data_.mode;
}

void ControlLoop::act(
  const ModeDecision & decision,
  const rclcpp::Time & current_time,
  const rclcpp::Duration & measured_period)
{
  policy_controller_->update(decision, current_time, measured_period);
}

void ControlLoop::command(const rclcpp::Time & current_time)
{
  for (auto & publisher : command_publishers_) {
    if (publisher->is_enabled()) {
      publisher->publish(current_time);
    }
  }
}

void ControlLoop::publish_debug_topics_if_needed(
  int iteration,
  int observation_publish_interval)
{
  const bool should_publish_debug_topics =
    debug_publish_enabled_ && iteration % observation_publish_interval == 0;
  if (should_publish_debug_topics) {
    publish_raw_debug_topics();
  }
}

void ControlLoop::warn_sensors_not_ready()
{
  if (!status_log_enabled_) {
    return;
  }

  RCLCPP_WARN_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "Skipping controller update - sensors not ready");
}

void ControlLoop::log_status_if_needed(int iteration)
{
  if (!status_log_enabled_) {
    return;
  }

  const auto status = make_status_snapshot();
  const bool has_status_changed =
    !last_status_snapshot_valid_ || status != last_status_snapshot_;
  const int status_log_interval = std::max(1, static_cast<int>(command_publish_rate_));
  const bool should_log_periodically = iteration % status_log_interval == 0;
  if (has_status_changed || should_log_periodically) {
    const char * reason = "periodic";
    if (has_status_changed) {
      reason = last_status_snapshot_valid_ ? "change" : "startup";
    }

    log_state(iteration, reason);
    last_status_snapshot_ = status;
    last_status_snapshot_valid_ = true;
  }
}

void ControlLoop::sleep_until_next_tick(
  std::chrono::steady_clock::time_point & next_time,
  const std::chrono::nanoseconds & command_period)
{
  // Sleep until next iteration, and report overruns without blocking the loop.
  next_time += command_period;
  const auto time_now = std::chrono::steady_clock::now();
  if (next_time < time_now) {
    if (status_log_enabled_) {
      const double overrun_ms = std::chrono::duration<double, std::milli>(
        time_now - next_time).count();
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[ControlLoop] overrun(%.2f ms)",
        overrun_ms);
    }

    next_time = time_now;
  }

  std::this_thread::sleep_until(next_time);
}

const char * ControlLoop::behavior_kind_name(BehaviorKind mode) const
{
  switch (mode) {
    case BehaviorKind::Damping:
      return "Damping";
    case BehaviorKind::Posture:
      return "Posture";
    case BehaviorKind::Policy:
      return "Policy";
  }

  return "Unknown";
}

ControlLoop::StatusSnapshot ControlLoop::make_status_snapshot() const
{
  const bool is_policy_mode = shared_data_.mode.active_behavior_kind == BehaviorKind::Policy;
  return StatusSnapshot{
    shared_data_.mode.active_behavior_kind,
    shared_data_.mode.active_state_name,
    shared_data_.requests.state_name,
    shared_data_.mode.transition_count,
    is_policy_mode,
    shared_data_.requests.damping,
    shared_data_.requests.action_limit_exceeded
  };
}

void ControlLoop::log_state(int iteration, const char * reason)
{
  log_basic_status(iteration, reason);
  if (detailed_status_log_) {
    log_detailed_status();
  }
}

void ControlLoop::log_basic_status(int iteration, const char * reason)
{
  const char * requested_state =
    shared_data_.requests.state_name.empty() ? "-" : shared_data_.requests.state_name.c_str();
  const char * inference_status =
    shared_data_.mode.active_behavior_kind == BehaviorKind::Policy ? "on" : "off";
  const char * damping_request_status = shared_data_.requests.damping ? "true" : "false";
  const char * action_limit_status =
    shared_data_.requests.action_limit_exceeded ? "true" : "false";
  const auto velocity_command = format_vec3(
    shared_data_.mode.velocity_commands.x(),
    shared_data_.mode.velocity_commands.y(),
    shared_data_.mode.velocity_commands.z());

  RCLCPP_INFO(
    node_->get_logger(),
    "[status:%s iter=%d] behavior=%s mode=%s requested=%s transitions=%" PRIu64 " "
    "inference=%s damping_req=%s limit=%s",
    reason,
    iteration,
    behavior_kind_name(shared_data_.mode.active_behavior_kind),
    shared_data_.mode.active_state_name.c_str(),
    requested_state,
    shared_data_.mode.transition_count,
    inference_status,
    damping_request_status,
    action_limit_status);
  RCLCPP_INFO(
    node_->get_logger(),
    "  cmd=%s episode=%.3f obs=%zu raw_action=%zu processed=%zu published=%zu",
    velocity_command.c_str(),
    shared_data_.policy.episode_time,
    shared_data_.policy.input.size(),
    shared_data_.policy.last_action.size(),
    shared_data_.output.processed_action.size(),
    shared_data_.output.published_action.size());
}

void ControlLoop::log_detailed_status()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "  imu_ang_vel:       %s",
    format_vec3(
      shared_data_.sensors.angular_velocity.x(),
      shared_data_.sensors.angular_velocity.y(),
      shared_data_.sensors.angular_velocity.z()).c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "  projected_gravity: %s",
    format_vec3(
      shared_data_.sensors.projected_gravity.x(),
      shared_data_.sensors.projected_gravity.y(),
      shared_data_.sensors.projected_gravity.z()).c_str());

  std::vector<float> joint_pos_rel;
  joint_pos_rel.reserve(static_cast<size_t>(shared_data_.sensors.joint_pos.size()));
  for (Eigen::Index i = 0; i < shared_data_.sensors.joint_pos.size(); ++i) {
    joint_pos_rel.push_back(
      shared_data_.sensors.joint_pos[i] - shared_data_.output.default_joint_pos[i]);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "  joint_pos_rel:     %s",
    format_vector_preview(joint_pos_rel, 6).c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "  joint_vel:         %s",
    format_vector_preview(shared_data_.sensors.joint_vel, 6).c_str());
  // Raw NN output stays in policy joint order; the other previews use robot joint order.
  RCLCPP_INFO(
    node_->get_logger(),
    "  raw_action(policy_order): %s",
    format_vector_preview(shared_data_.policy.last_action, 6).c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "  processed_action:  %s",
    format_vector_preview(shared_data_.output.processed_action, 6).c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "  published_action:  %s",
    format_vector_preview(shared_data_.output.published_action, 6).c_str());
}

std::string ControlLoop::format_vec3(float x, float y, float z)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "[%.3f, %.3f, %.3f]", x, y, z);
  return std::string(buf);
}

void ControlLoop::publish_raw_debug_topics()
{
  if (!raw_observation_publisher_ || !processed_action_publisher_) {
    return;
  }

  std_msgs::msg::Float64MultiArray obs_msg;
  obs_msg.data.reserve(shared_data_.policy.input.size());
  for (const auto value : shared_data_.policy.input) {
    obs_msg.data.push_back(static_cast<double>(value));
  }

  raw_observation_publisher_->publish(obs_msg);

  std_msgs::msg::Float64MultiArray action_msg;
  action_msg.data.reserve(shared_data_.output.processed_action.size());
  for (const auto value : shared_data_.output.processed_action) {
    action_msg.data.push_back(static_cast<double>(value));
  }

  processed_action_publisher_->publish(action_msg);
}


}  // namespace ai_sapiens_sim2real
