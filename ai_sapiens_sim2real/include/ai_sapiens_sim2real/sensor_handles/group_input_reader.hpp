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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__GROUP_INPUT_READER_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__GROUP_INPUT_READER_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "ai_sapiens_sim2real/axis_range.hpp"
#include "ai_sapiens_sim2real/mode_runtime/group_arbiter.hpp"
#include "ai_sapiens_sim2real/mode_runtime/operator_command_input_options.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"

namespace ai_sapiens_sim2real
{
// A group receiver supplies a sample, never a robot failsafe request.
// The individual TeleopInputHandle remains the only input watchdog that requests damping.
class GroupInputReader
{
public:
  GroupInputReader(
    const std::shared_ptr<TeleopInputPluginBase> & plugin, const GroupInputOptions & options)
  : plugin_(plugin), ranges_(plugin->output_axis_ranges()),
    timeout_(options.timeout), velocity_timeout_(options.vel_command_timeout),
    commands_(options.commands)
  {
    if (!std::isfinite(options.timeout) || options.timeout <= 0.0 ||
      !std::isfinite(options.vel_command_timeout) || options.vel_command_timeout <= 0.0 ||
      options.vel_command_timeout > options.timeout)
    {
      throw std::runtime_error(
          "Group input timeouts must be finite, positive and velocity <= input");
    }
  }

  GroupArbiter::Input read() const
  {
    TeleopInputCommand command;
    const bool received = plugin_->read_latest_accepted_command(command);
    const auto age = std::chrono::steady_clock::now() - command.received_at;
    GroupArbiter::Input sample;
    sample.valid = received && age <= timeout_;
    sample.command = decode(command.input_code, commands_);
    sample.selector = command.selector_code;
    sample.velocity_fresh = sample.valid && age <= velocity_timeout_;
    if (!sample.velocity_fresh) {
      return sample;
    }

    const std::array<AxisRange, 3> axes = {ranges_.linear_x, ranges_.linear_y, ranges_.angular_z};
    for (size_t axis = 0; axis < axes.size(); ++axis) {
      const double value = command.velocity[axis];
      constexpr double tolerance = 1e-3;
      if (value < axes[axis].min - tolerance || value > axes[axis].max + tolerance) {
        sample.velocity = {};
        sample.velocity_fresh = false;
        return sample;
      }
      sample.velocity[axis] = normalize_axis_to_unit(value, axes[axis]);
    }
    return sample;
  }

  static GroupArbiter::Command decode(uint16_t code, const GroupCommandMapping & mapping)
  {
    using Command = GroupArbiter::Command;
    if (code == mapping.damping_code) {return Command::Damping;}
    if (code == mapping.ready_code) {return Command::ReadyPose;}
    if (code == mapping.locomotion_code) {return Command::Locomotion;}
    if (code == mapping.mimic_code) {return Command::Mimic;}
    return Command::Hold;
  }

private:
  std::shared_ptr<TeleopInputPluginBase> plugin_;
  AxisRanges ranges_;
  std::chrono::duration<double> timeout_;
  std::chrono::duration<double> velocity_timeout_;
  GroupCommandMapping commands_;
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__GROUP_INPUT_READER_HPP_
