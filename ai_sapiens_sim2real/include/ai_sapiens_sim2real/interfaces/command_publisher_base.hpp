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
// Author: Woojin Wie

#ifndef AI_SAPIENS_SIM2REAL__INTERFACES__COMMAND_PUBLISHER_BASE_HPP_
#define AI_SAPIENS_SIM2REAL__INTERFACES__COMMAND_PUBLISHER_BASE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace ai_sapiens_sim2real
{

/**
 * @brief Base interface for all command publishers.
 *
 * Command publishers read from SharedControlData and publish to ROS topics.
 * They are called after all controllers have been updated (WRITE phase).
 *
 * Similar to ros2_control's CommandInterface pattern.
 */
class CommandPublisherBase
{
public:
  virtual ~CommandPublisherBase() = default;

  /**
   * @brief Publish command data.
   *
   * This method is called in the RT thread after all controllers
   * have been updated. It should read from SharedControlData and publish.
   *
   * @param time Current ROS time
   */
  virtual void publish(const rclcpp::Time & time) = 0;

  /**
   * @brief Get the name of this command publisher.
   * @return Publisher name for logging/debugging
   */
  virtual std::string get_name() const = 0;

  /**
   * @brief Check if publisher is enabled.
   * @return true if publishing is enabled
   */
  virtual bool is_enabled() const
  {
    return true;
  }
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__INTERFACES__COMMAND_PUBLISHER_BASE_HPP_
