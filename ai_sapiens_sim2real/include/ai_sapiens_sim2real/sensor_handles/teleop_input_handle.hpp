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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__TELEOP_INPUT_HANDLE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__TELEOP_INPUT_HANDLE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/interfaces/sensor_handle_base.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_command.hpp"
#include "ai_sapiens_sim2real/teleop_input/teleop_input_plugin_base.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

class TeleopInputHandle : public SensorHandleBase
{
public:
  TeleopInputHandle(
    rclcpp::Node::SharedPtr node,
    TeleopInput * teleop,
    ModeRequests * requests,
    const AxisRanges * active_velocity_command_ranges,
    std::shared_ptr<TeleopInputPluginBase> plugin,
    double timeout_seconds);

  void update(const rclcpp::Time & /*time*/) override;
  std::string get_name() const override;
  bool is_ready() const override;

private:
  bool is_command_stale(const TeleopInputCommand & command) const;
  void copy_command_state(
    const TeleopInputCommand & command,
    bool has_accepted_command,
    bool input_unavailable);
  void apply_unavailable_teleop_input();
  void log_unavailable_input_once(const TeleopInputCommand & command, bool has_accepted_command);
  void log_out_of_range_command(const Eigen::Vector3f & command) const;
  // Normalize plugin output and flag declared-range violations.
  Eigen::Vector3f normalize_plugin_output(
    const Eigen::Vector3f & plugin_output, bool & out_of_range) const;

  rclcpp::Node::SharedPtr node_;
  TeleopInput * teleop_;
  ModeRequests * requests_;
  const AxisRanges * active_velocity_command_ranges_;
  std::shared_ptr<TeleopInputPluginBase> plugin_;
  // The plugin's declared output range, queried once after configure().
  AxisRanges plugin_output_ranges_;
  std::chrono::duration<double> timeout_;
  bool unavailable_logged_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__TELEOP_INPUT_HANDLE_HPP_
