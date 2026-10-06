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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__DANCE_MOTION_REFERENCE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__DANCE_MOTION_REFERENCE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "ai_sapiens_sim2real/config/dance_reference_config.hpp"
#include "ai_sapiens_sim2real/policy/planar_motion_steering.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"

namespace ai_sapiens_sim2real
{

struct DanceReferenceOutput
{
  std::array<float, 23> joint_pos{}, joint_vel{};
  Eigen::Vector3f root_shift{Eigen::Vector3f::Zero()};
  std::array<Eigen::Vector3f, 2> foot_position{{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()}};
  std::array<Eigen::Quaternionf,
    2> foot_orientation{{Eigen::Quaternionf::Identity(), Eigen::Quaternionf::Identity()}};
  std::array<bool, 2> valid{{true, true}};
  std::array<float, 2> ik_error{}, air{};
  int added_steps{0};
};

// Single-robot port of humanoid_motion's version 4/5 planner and IK.
// Source contact is estimated from CSV poses and K1 geometry instead of labels.
// All geometry uses the original motion axes (before subtracting episode XY).
// Data allocation and validation happen at construction; step() uses fixed storage.
class DanceMotionReference
{
public:
  DanceMotionReference(
    const DanceReferenceConfig & config, const std::vector<std::string> & policy_joints,
    double step_dt, const MotionReference & motion);
  ~DanceMotionReference();
  DanceMotionReference(const DanceMotionReference &) = delete;
  DanceMotionReference & operator=(const DanceMotionReference &) = delete;

  void reset(int frame);
  void step(
    int frame, const PlanarMotionSteering & steering,
    const Eigen::Vector3f & linear_velocity_w, bool command_active);
  const DanceReferenceOutput & output() const;
  int frame_count() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__DANCE_MOTION_REFERENCE_HPP_
