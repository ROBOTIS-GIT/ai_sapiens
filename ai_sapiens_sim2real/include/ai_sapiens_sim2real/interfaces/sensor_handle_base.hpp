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

#ifndef AI_SAPIENS_SIM2REAL__INTERFACES__SENSOR_HANDLE_BASE_HPP_
#define AI_SAPIENS_SIM2REAL__INTERFACES__SENSOR_HANDLE_BASE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace ai_sapiens_sim2real
{

/**
 * @brief Base interface for all sensor handles.
 *
 * Sensor handles subscribe to ROS topics (non-RT thread) and provide
 * realtime-safe access to sensor data via the update() method.
 *
 * Similar to ros2_control's StateInterface pattern.
 */
class SensorHandleBase
{
public:
  virtual ~SensorHandleBase() = default;

  /**
   * @brief Update sensor data in the realtime loop.
   *
   * This method is called in the RT thread. It should copy data from
   * the RealtimeBuffer to SharedControlData without blocking.
   *
   * @param time Current ROS time
   */
  virtual void update(const rclcpp::Time & time) = 0;

  /**
   * @brief Get the name of this sensor handle.
   * @return Sensor handle name for logging/debugging
   */
  virtual std::string get_name() const = 0;

  /**
   * @brief Check if the sensor has received valid data.
   * @return true if at least one message has been received
   */
  virtual bool is_ready() const = 0;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__INTERFACES__SENSOR_HANDLE_BASE_HPP_
