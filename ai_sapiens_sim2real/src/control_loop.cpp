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

#include <rcutils/error_handling.h>

#include <array>
#include <cstring>

#include <realtime_tools/lock_free_queue.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ai_sapiens_sim2real
{

class ControlLoop::DebugTopicPublisher
{
public:
  DebugTopicPublisher(rclcpp::Node & node, const char * topic)
  : publisher_(node.create_publisher<Message>(topic, 10)) {}

  void publish(const std::vector<float> & values)
  {
    try {
      message_.data.assign(values.begin(), values.end());
      publisher_->publish(message_);
    } catch (const std::exception & error) {
      record_failure(error.what());
    } catch (...) {
      record_failure("unknown exception");
    }
  }

  void flush(const rclcpp::Logger & logger)
  {
    Failure failure;
    if (failures_.pop(failure)) {
      RCLCPP_WARN(logger, "Dropped monitoring message (topic=%s): %s",
        publisher_->get_topic_name(), failure.data());
    }
  }

private:
  using Message = std_msgs::msg::Float64MultiArray;
  using Failure = std::array<char, RCUTILS_ERROR_MESSAGE_MAX_LENGTH>;

  void record_failure(const char * reason)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_report_) {
      return;
    }
    next_report_ = now + std::chrono::seconds(1);
    Failure failure{};
    std::strncpy(failure.data(), reason, failure.size() - 1);
    // Keep the pending warning when the consumer has not drained it yet.
    (void)failures_.push(failure);
  }

  rclcpp::Publisher<Message>::SharedPtr publisher_;
  Message message_;
  realtime_tools::LockFreeSPSCQueue<Failure, 1> failures_;
  std::chrono::steady_clock::time_point next_report_{std::chrono::steady_clock::time_point::min()};
};

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
  if (!enabled) {
    debug_publish_enabled_ = false;
    raw_observation_publisher_.reset();
    processed_action_publisher_.reset();
    return;
  }

  if (!debug_publish_enabled_) {
    raw_observation_publisher_ = std::make_unique<DebugTopicPublisher>(
      *node_, "/policy_input/raw_observation");
    processed_action_publisher_ = std::make_unique<DebugTopicPublisher>(
      *node_, "/policy_output/processed_action");
    debug_publish_enabled_ = true;
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

  faulted_ = false;
  rt_exception_ = nullptr;
  emergency_exception_ = nullptr;
  running_ = true;
  RCLCPP_INFO(node_->get_logger(), "Starting control loop at %.1f Hz", command_publish_rate_);
  rt_thread_ = std::thread(&ControlLoop::rt_thread_func, this);
}

void ControlLoop::stop()
{
  running_ = false;
  if (rt_thread_.joinable()) {
    wait();
    log_exit();
  }
  flush_diagnostics();
}

void ControlLoop::wait()
{
  if (rt_thread_.joinable()) {
    rt_thread_.join();
  }
}

void ControlLoop::log_exit() const
{
  if (rt_exception_) {
    log_abort();
  }
  RCLCPP_INFO(node_->get_logger(), "Control loop stopped");
}

void ControlLoop::flush_diagnostics()
{
  if (policy_controller_) {
    policy_controller_->flush_diagnostics();
  }

  for (const auto & publisher : command_publishers_) {
    publisher->flush_diagnostics();
  }
  if (debug_publish_enabled_) {
    raw_observation_publisher_->flush(node_->get_logger());
    processed_action_publisher_->flush(node_->get_logger());
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

rclcpp::Time ControlLoop::read_control_time()
{
  rt_stage_ = "clock";
  return node_->now();
}

void ControlLoop::rt_thread_func()
{
  // Fatal boundary for unexpected RT-thread failures.
  int iteration = 0;
  // A zero stamp identifies an emergency command issued before the first clock read.
  rclcpp::Time last_control_time(0, 0, RCL_ROS_TIME);
  try {
    prepare_realtime_thread();
    if (!initialize_controllers_after_sensor_ready(last_control_time)) {
      faulted_ = true;
      return;
    }

    const auto command_period = std::chrono::nanoseconds(
      static_cast<int64_t>(1'000'000'000.0 / command_publish_rate_));
    auto next_time = std::chrono::steady_clock::now();
    last_control_time = read_control_time();
    const int observation_publish_interval =
      std::max(1, static_cast<int>(command_publish_rate_ / debug_publish_rate_));

    // Keep the order: sensors, controllers, commands, sleep.
    RCLCPP_INFO(node_->get_logger(), "Starting RT loop at %.1f Hz", command_publish_rate_);

    while (running_ && rclcpp::ok()) {
      const auto previous_time = last_control_time;
      last_control_time = read_control_time();
      const auto measured_period = last_control_time - previous_time;

      run_control_step(last_control_time, measured_period, iteration, observation_publish_interval);
      log_status_if_needed(iteration);
      iteration++;
      sleep_until_next_tick(next_time, command_period);
    }

    RCLCPP_INFO(node_->get_logger(), "RT thread exiting after %d iterations", iteration);
  } catch (...) {
    rt_exception_ = std::current_exception();
    abort_stage_ = rt_stage_;
    abort_iteration_ = iteration;
    running_ = false;
    try {
      publish_emergency_damping(last_control_time);
    } catch (...) {
      emergency_exception_ = std::current_exception();
    }
    faulted_ = true;
  }
}

void ControlLoop::log_abort() const
{
  const auto describe = [](std::exception_ptr failure) -> std::string {
      try {
        std::rethrow_exception(failure);
      } catch (const std::exception & error) {
        return error.what();
      } catch (...) {
        return "unknown exception";
      }
    };
  const auto & mode = shared_data_.mode;
  RCLCPP_FATAL(node_->get_logger(), "RT loop aborted (stage=%s, tick=%d, mode=%s, policy=%s): %s",
    abort_stage_, abort_iteration_, mode.active_state_name.c_str(),
    mode.active_policy_name.empty() ? "none" : mode.active_policy_name.c_str(),
    describe(rt_exception_).c_str());
  if (emergency_exception_) {
    RCLCPP_ERROR(node_->get_logger(), "Emergency Damping failed: %s",
      describe(emergency_exception_).c_str());
  }
}

void ControlLoop::publish_emergency_damping(const rclcpp::Time & stamp)
{
  if (!mode_controller_) {
    throw std::logic_error("no mode controller configured");
  }
  mode_controller_->apply_emergency_damping();
  command(stamp, CommandPublishMode::EmergencyDamping);
}

void ControlLoop::prepare_realtime_thread()
{
  rt_stage_ = "realtime setup";
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

bool ControlLoop::initialize_controllers_after_sensor_ready(rclcpp::Time & last_control_time)
{
  rt_stage_ = "sensor and controller initialization";
  // Wait for all required sensors to be ready before starting inference.
  if (!wait_for_sensors_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize sensors, exiting");
    running_ = false;
    return false;
  }

  // Reset both controllers now that sensors are ready.
  last_control_time = read_control_time();
  sense(last_control_time);
  rt_stage_ = "controller reset";
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
  rt_stage_ = "sensor readiness";
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
  rt_stage_ = "sensor update";
  for (auto & [handle, required] : sensor_handles_) {
    handle->update(current_time);
  }
}

const ModeDecision & ControlLoop::decide(
  const rclcpp::Time & current_time, const rclcpp::Duration & measured_period)
{
  rt_stage_ = "mode decision";
  mode_controller_->update(current_time, measured_period);
  return shared_data_.mode;
}

void ControlLoop::act(
  const ModeDecision & decision,
  const rclcpp::Time & current_time,
  const rclcpp::Duration & measured_period)
{
  rt_stage_ = "policy update";
  policy_controller_->update(decision, current_time, measured_period);
}

void ControlLoop::command(const rclcpp::Time & current_time, CommandPublishMode mode)
{
  rt_stage_ = "command publish";
  for (auto & publisher : command_publishers_) {
    if (publisher->is_enabled()) {
      publisher->publish(current_time, mode);
    }
  }
}

void ControlLoop::publish_debug_topics_if_needed(
  int iteration,
  int observation_publish_interval)
{
  rt_stage_ = "debug publish";
  const bool should_publish_debug_topics =
    debug_publish_enabled_ && iteration % observation_publish_interval == 0;
  if (should_publish_debug_topics) {
    raw_observation_publisher_->publish(shared_data_.policy.input);
    processed_action_publisher_->publish(shared_data_.output.command.position);
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
  rt_stage_ = "status log";
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
  rt_stage_ = "wait for next tick";
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
    shared_data_.output.command.position.size(),
    shared_data_.output.last_published.position.size());
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
    format_vector_preview(shared_data_.output.command.position, 6).c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "  published_action:  %s",
    format_vector_preview(shared_data_.output.last_published.position, 6).c_str());
}

std::string ControlLoop::format_vec3(float x, float y, float z)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "[%.3f, %.3f, %.3f]", x, y, z);
  return std::string(buf);
}

}  // namespace ai_sapiens_sim2real
