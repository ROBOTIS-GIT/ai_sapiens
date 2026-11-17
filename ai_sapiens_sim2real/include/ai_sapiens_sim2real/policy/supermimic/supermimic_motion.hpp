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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__SUPERMIMIC__SUPERMIMIC_MOTION_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__SUPERMIMIC__SUPERMIMIC_MOTION_HPP_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "ai_sapiens_sim2real/policy/mimic/motion_reference.hpp"

namespace ai_sapiens_sim2real
{

// Immutable CSV frames in policy joint order. Velocities use MotionReference's central difference.
class SuperMimicMotion
{
public:
  struct Frame
  {
    Eigen::Matrix3f rotation;
    Eigen::VectorXf joint_position;
    Eigen::VectorXf joint_velocity;
  };

  SuperMimicMotion(
    std::shared_ptr<MotionReference> reference, double fps,
    const std::vector<std::string> & joints);
  const Frame & frame(size_t index) const {return frames_.at(index);}
  size_t size() const {return frames_.size();}
  const MotionReference * reference() const {return reference_.get();}

private:
  std::shared_ptr<MotionReference> reference_;
  std::vector<Frame> frames_;
};

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__SUPERMIMIC__SUPERMIMIC_MOTION_HPP_
