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

#ifndef AI_SAPIENS_SIM2REAL__COMMAND_PUBLISHERS__JOINT_COMMAND_PUBLISHER_HPP_
#define AI_SAPIENS_SIM2REAL__COMMAND_PUBLISHERS__JOINT_COMMAND_PUBLISHER_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_interfaces/msg/joint_impedance_command.hpp"
#include "ai_sapiens_sim2real/interfaces/command_publisher_base.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief Command publisher for joint position commands.
 *
 * Publishes BehaviorOutput command buffers to a JointImpedanceCommand
 * topic in the controller joint order.
 */
class JointCommandPublisher : public CommandPublisherBase
{
public:
  /**
   * @brief Construct joint command publisher.
   * @param node ROS node for publisher creation
   * @param output Active behavior output block
   * @param topic Topic name to publish to
   * @param controller_joint_names Joint names in controller command order
   * @param enforce_position_limits Whether active policy limits should clamp commands
   * @param enabled Whether publishing is enabled
   */
  JointCommandPublisher(
    rclcpp::Node::SharedPtr node,
    BehaviorOutput * output,
    const std::string & topic,
    const std::vector<std::string> & controller_joint_names,
    bool enforce_position_limits = false,
    bool enabled = true);

  void publish(const rclcpp::Time & time) override;
  std::string get_name() const override;
  bool is_enabled() const override
  {
    return enabled_;
  }

  /**
   * @brief Enable or disable publishing.
   * @param enabled True to enable publishing
   */
  void set_enabled(bool enabled);

private:
  rclcpp::Node::SharedPtr node_;
  BehaviorOutput * output_;

  std::string topic_;
  std::vector<std::string> controller_joint_names_;
  bool enforce_position_limits_;
  bool enabled_;

  // Message storage is reused to avoid allocations in publish().
  rclcpp::Publisher<ai_sapiens_interfaces::msg::JointImpedanceCommand>::SharedPtr
    publisher_;
  ai_sapiens_interfaces::msg::JointImpedanceCommand msg_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__COMMAND_PUBLISHERS__JOINT_COMMAND_PUBLISHER_HPP_
