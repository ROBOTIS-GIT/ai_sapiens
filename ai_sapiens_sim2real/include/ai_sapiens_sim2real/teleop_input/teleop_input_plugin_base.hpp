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

#ifndef AI_SAPIENS_SIM2REAL__TELEOP_INPUT__TELEOP_INPUT_PLUGIN_BASE_HPP_
#define AI_SAPIENS_SIM2REAL__TELEOP_INPUT__TELEOP_INPUT_PLUGIN_BASE_HPP_

#include <atomic>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/teleop_input/teleop_input_command.hpp"
#include "ai_sapiens_sim2real/axis_range.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief Common interface for joystick/RC/teleop input plugins.
 *
 * A teleop input plugin owns the device-specific ROS subscriptions and converts
 * raw device messages into TeleopInputCommand values. New controller types
 * should implement this interface and register the class with pluginlib.
 *
 * Device-specific validation and freshness checks belong inside the plugin
 * implementation because each raw message type may expose different sequence,
 * CRC, timestamp, or packet-valid fields. The upper-level TeleopInputHandle owns
 * system watchdog timeout policy.
 */
class TeleopInputPluginBase
{
public:
  TeleopInputPluginBase()
  {
    TeleopInputCommand initial;
    initial.received_at = std::chrono::steady_clock::now();
    buffer_.initRT(initial);
  }

  virtual ~TeleopInputPluginBase() = default;

  /**
   * @brief Read plugin YAML and create non-RT subscriptions/timers.
   */
  virtual void configure(const rclcpp::Node::SharedPtr & node, const YAML::Node & config) = 0;

  // The range the command velocity spans at full deflection, per axis. The
  // velocity is the plugin's own output, not a robot velocity: TeleopInputHandle
  // maps it onto the active policy's command range.
  virtual AxisRanges output_axis_ranges() const = 0;

  /**
   * @brief Copy the latest valid teleop command without blocking the RT loop.
   *
   * Return false until the plugin has accepted at least one valid command.
   */
  bool read_latest_accepted_command(TeleopInputCommand & command)
  {
    if (!received_.load(std::memory_order_acquire)) {
      return false;
    }

    command = *buffer_.readFromRT();
    return true;
  }

  /**
   * @brief True after the plugin has accepted its first valid input sample.
   */
  bool is_ready() const
  {
    return received_.load(std::memory_order_acquire);
  }

  /**
   * @brief Human-readable plugin name used in logs and diagnostics.
   */
  virtual std::string name() const = 0;

  /**
   * @brief Primary ROS topic that the plugin waits for during startup.
   */
  virtual std::string topic_name() const
  {
    return "unknown";
  }

protected:
  void accept_valid_command(TeleopInputCommand command)
  {
    command.received_at = std::chrono::steady_clock::now();
    buffer_.writeFromNonRT(command);
    received_.store(true, std::memory_order_release);
  }

private:
  realtime_tools::RealtimeBuffer<TeleopInputCommand> buffer_;
  std::atomic<bool> received_{false};
};

template<typename RawMessageT>
class TeleopInputPluginTemplate : public TeleopInputPluginBase
{
protected:
  // Shared plugin pipeline: valid -> fresh -> command -> accepted
  void handle_raw_message(const RawMessageT & msg)
  {
    if (!is_message_valid(msg)) {
      return;
    }
    if (!is_message_fresh(msg)) {
      on_stale_message(msg);
      return;
    }

    accept_valid_command(make_command_from_message(msg));
    on_message_accepted(msg);
  }

  // Required core hooks for new input plugins
  virtual bool is_message_valid(const RawMessageT & msg) const = 0;
  virtual bool is_message_fresh(const RawMessageT & msg) const = 0;
  virtual TeleopInputCommand make_command_from_message(const RawMessageT & msg) const = 0;

  // Optional side-effect hooks for diagnostics and freshness state
  virtual void on_stale_message(const RawMessageT & msg) const
  {
    (void)msg;
  }

  virtual void on_message_accepted(const RawMessageT & msg)
  {
    (void)msg;
  }
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__TELEOP_INPUT__TELEOP_INPUT_PLUGIN_BASE_HPP_
