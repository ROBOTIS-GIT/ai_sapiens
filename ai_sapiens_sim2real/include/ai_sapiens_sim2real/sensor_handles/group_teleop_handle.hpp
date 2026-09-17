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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__GROUP_TELEOP_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__GROUP_TELEOP_HANDLE_HPP_

#include <atomic>
#include <memory>
#include <realtime_tools/lock_free_queue.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include "ai_sapiens_sim2real/mode_runtime/group_arbiter.hpp"
#include "ai_sapiens_sim2real/mode_runtime/operator_command_input_options.hpp"
#include "ai_sapiens_sim2real/sensor_handles/teleop_input_handle.hpp"
#include "ai_sapiens_sim2real/sensor_handles/group_input_reader.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"

namespace ai_sapiens_sim2real
{
class GroupTeleopHandle : public SensorHandleBase
{
public:
  GroupTeleopHandle(
    const rclcpp::Node::SharedPtr & node, SharedControlData * state,
    const OperatorCommandInputOptions & options,
    const std::shared_ptr<TeleopInputPluginBase> & individual,
    const std::shared_ptr<TeleopInputPluginBase> & group)
  : state_(state), options_(options),
    individual_handle_(node, &individual_, &state->requests,
      &state->active_velocity_command_ranges, individual,
      options.teleop_input_timeout, options.teleop_vel_command_timeout),
    group_reader_(group, options.group.value())
  {
    state_->group.frame.route = GroupControlFrame::Route::Individual;
    status_pub_ = node->create_publisher<std_msgs::msg::String>("/ai_sapiens/group_status", 10);
    status_timer_ = node->create_wall_timer(std::chrono::milliseconds(200),
        [this, logger = node->get_logger()]() {
          flush_logs(logger);
          std_msgs::msg::String msg;
          msg.data = state_->group.participating.load() ? "GROUP" : "INDIVIDUAL";
          msg.data +=
          state_->group.available.load() ? " group=available" : " group=lost";
          status_pub_->publish(msg);
      });
  }

  void update(const rclcpp::Time & time) override
  {
    individual_handle_.update(time);
    const auto group_sample = group_reader_.read();
    const auto result = arbiter_.update(
      input(individual_), group_sample,
      state_->mode.active_state_name == options_.group->commands.locomotion_state,
      state_->mode.active_behavior_kind == BehaviorKind::Posture);
    enqueue_log(result.event);
    update_group_state(result, group_sample.valid);
    copy_selected_command(result);
    copy_velocity(result);
  }

  bool is_ready() const override {return individual_handle_.is_ready();}
  std::string get_name() const override {return "group_teleop";}

private:
  using Command = GroupArbiter::Command;
  using Event = GroupArbiter::Event;
  void enqueue_log(Event event)
  {
    // The control thread only enqueues fixed-size events; the ROS timer logs them.
    if (event != Event::None && !log_events_.push(event)) {
      dropped_logs_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void update_group_state(const GroupArbiter::Output & result, bool group_available)
  {
    using Route = GroupControlFrame::Route;
    const bool override_individual = result.participating || result.exited;
    const auto route = !override_individual ? Route::Individual :
      (result.request ? Route::Command : Route::Hold);
    state_->group.frame = {route, individual_.input_code, individual_.selector_code};
    state_->group.participating.store(result.participating, std::memory_order_release);
    state_->group.available.store(group_available, std::memory_order_release);
  }

  void copy_selected_command(const GroupArbiter::Output & result)
  {
    auto & output = state_->teleop;
    output.received.store(individual_.received.load(), std::memory_order_release);
    output.unavailable.store(individual_.unavailable.load(), std::memory_order_release);
    output.api_mode_requested = !state_->group.frame.overrides_individual() &&
      individual_.api_mode_requested;
    output.update_time = individual_.update_time;
    output.input_code =
      state_->group.frame.overrides_individual() ? code(result.command) : individual_.input_code;
    output.selector_code =
      state_->group.frame.overrides_individual() ? result.selector : individual_.selector_code;
    output.velocity_fresh = individual_.velocity_fresh;
  }

  void copy_velocity(const GroupArbiter::Output & result)
  {
    auto & output = state_->teleop;
    for (size_t axis = 0; axis < 3; ++axis) {
      output.velocity_command_normalized[axis] = result.velocity[axis];
    }
    const auto & ranges = state_->active_velocity_command_ranges;
    output.velocity_commands.x() = scale_unit_to_axis(result.velocity[0], ranges.linear_x);
    output.velocity_commands.y() = scale_unit_to_axis(result.velocity[1], ranges.linear_y);
    output.velocity_commands.z() = scale_unit_to_axis(result.velocity[2], ranges.angular_z);
  }

  void flush_logs(const rclcpp::Logger & logger)
  {
    Event event;
    while (log_events_.pop(event)) {
      const char * reason = "unknown";
      switch (event) {
        case Event::Joined:
          RCLCPP_INFO(logger, "[GroupControl] GROUP reason(group_requested)");
          continue;
        case Event::Left:
          RCLCPP_INFO(logger, "[GroupControl] INDIVIDUAL reason(individual_requested)");
          continue;
        case Event::IndividualLost:
          RCLCPP_WARN(logger, "[GroupControl] INDIVIDUAL reason(individual_input_unavailable)");
          continue;
        case Event::GroupLost:
          RCLCPP_WARN(logger, "[GroupControl] INDIVIDUAL reason(group_input_unavailable)");
          continue;
        case Event::JoinGroupUnavailable: reason = "group_input_unavailable"; break;
        case Event::JoinRobotNotLocomotion: reason = "robot_not_locomotion"; break;
        case Event::JoinIndividualNotLocomotion: reason = "individual_not_locomotion"; break;
        case Event::JoinGroupNotLocomotion: reason = "group_not_locomotion"; break;
        case Event::JoinIndividualVelocityStale: reason = "individual_velocity_stale"; break;
        case Event::JoinGroupVelocityStale: reason = "group_velocity_stale"; break;
        case Event::JoinIndividualMoving: reason = "individual_velocity_nonzero"; break;
        case Event::JoinGroupMoving: reason = "group_velocity_nonzero"; break;
        case Event::None: continue;
      }
      RCLCPP_WARN(logger, "[GroupControl] JOIN_REJECTED reason(%s)", reason);
    }
    const auto dropped = dropped_logs_.exchange(0, std::memory_order_relaxed);
    if (dropped != 0) {
      RCLCPP_WARN(logger, "[GroupControl] log_queue_overflow dropped(%u)", dropped);
    }
  }
  uint16_t code(Command command) const
  {
    switch (command) {
      case Command::Damping: return options_.group->commands.damping_code;
      case Command::ReadyPose: return options_.group->commands.ready_code;
      case Command::Locomotion: return options_.group->commands.locomotion_code;
      case Command::Mimic: return options_.group->commands.mimic_code;
      default: return 0;
    }
  }
  GroupArbiter::Input input(const TeleopInput & teleop) const
  {
    GroupArbiter::Input result;
    result.valid = teleop.received && !teleop.unavailable;
    result.velocity_fresh = teleop.velocity_fresh;
    result.join = teleop.group_requested;
    result.selector = teleop.selector_code;
    result.command = GroupInputReader::decode(teleop.input_code, options_.group->commands);
    for (size_t axis = 0; axis < 3; ++axis) {
      result.velocity[axis] = teleop.velocity_command_normalized[axis];
    }
    return result;
  }
  SharedControlData * state_;
  OperatorCommandInputOptions options_;
  TeleopInput individual_;
  TeleopInputHandle individual_handle_;
  GroupInputReader group_reader_;
  GroupArbiter arbiter_;
  realtime_tools::LockFreeSPSCQueue<Event, 64> log_events_;
  std::atomic<unsigned int> dropped_logs_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__GROUP_TELEOP_HANDLE_HPP_
