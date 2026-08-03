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

#include "ai_sapiens_mujoco/gantry_service_node.hpp"

#include <memory>
#include <sstream>
#include <utility>

namespace ai_sapiens_mujoco
{

GantryServiceNode::GantryServiceNode(std::shared_ptr<MujocoSimulation> sim)
: rclcpp::Node("mujoco_sim"), sim_(std::move(sim))
{
  set_height_service_ = create_service<ai_sapiens_interfaces::srv::SetGantryHeight>(
    "/mujoco_sim/gantry/set_height",
    [this](
      const ai_sapiens_interfaces::srv::SetGantryHeight::Request::SharedPtr request,
      ai_sapiens_interfaces::srv::SetGantryHeight::Response::SharedPtr response) {
      if (sim_->gantry_set_target(request->height, request->speed)) {
        std::ostringstream message;
        message << "gantry moving to height " << request->height << " m";
        response->success = true;
        response->message = message.str();
      } else {
        response->success = false;
        response->message = "gantry not attached (released or absent)";
      }
    });

  release_service_ = create_service<std_srvs::srv::Trigger>(
    "/mujoco_sim/gantry/release",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      if (sim_->gantry_release()) {
        response->success = true;
        response->message = "gantry released";
      } else {
        response->success = false;
        response->message = "gantry not attached (released or absent)";
      }
    });
}

}  // namespace ai_sapiens_mujoco
