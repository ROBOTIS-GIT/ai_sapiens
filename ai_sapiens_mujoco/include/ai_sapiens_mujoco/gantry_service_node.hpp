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

#ifndef AI_SAPIENS_MUJOCO__GANTRY_SERVICE_NODE_HPP_
#define AI_SAPIENS_MUJOCO__GANTRY_SERVICE_NODE_HPP_

#include <memory>

#include "ai_sapiens_interfaces/srv/set_gantry_height.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ai_sapiens_mujoco/mujoco_simulation.hpp"

namespace ai_sapiens_mujoco
{

/// ROS node exposing the simulation gantry as services:
/// /mujoco_sim/gantry/set_height and /mujoco_sim/gantry/release.
class GantryServiceNode : public rclcpp::Node
{
public:
  explicit GantryServiceNode(std::shared_ptr<MujocoSimulation> sim);

private:
  std::shared_ptr<MujocoSimulation> sim_;
  rclcpp::Service<ai_sapiens_interfaces::srv::SetGantryHeight>::SharedPtr set_height_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
};

}  // namespace ai_sapiens_mujoco

#endif  // AI_SAPIENS_MUJOCO__GANTRY_SERVICE_NODE_HPP_
