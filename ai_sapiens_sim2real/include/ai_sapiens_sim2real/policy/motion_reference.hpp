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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__MOTION_REFERENCE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MOTION_REFERENCE_HPP_

#include <cctype>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

class MotionReference
{
public:
  MotionReference(
    const std::string & motion_file,
    float fps,
    const std::vector<std::string> & fallback_joint_order);

  void seek(float time);

  Eigen::VectorXf joint_pos() const
  {
    return dof_positions_[index_0_] * (1.0f - blend_) + dof_positions_[index_1_] * blend_;
  }

  Eigen::VectorXf joint_vel() const
  {
    return dof_velocities_[index_0_] * (1.0f - blend_) + dof_velocities_[index_1_] * blend_;
  }

  Eigen::Quaternionf root_quaternion() const
  {
    return root_quaternions_[index_0_].slerp(blend_, root_quaternions_[index_1_]);
  }

  float joint_pos_for_joint(const std::string & joint_name) const;
  const std::vector<std::string> & joint_order() const;
  float duration() const
  {
    return duration_;
  }

private:
  // CSV parsing helpers accept both named and fallback joint-order files.
  static std::string trim(const std::string & value);
  static std::vector<std::string> split_csv_line(const std::string & line);
  static bool parse_float(const std::string & token, float & value);
  static bool parse_numeric_row(const std::vector<std::string> & tokens, std::vector<float> & row);
  void validate_joint_order() const;
  std::vector<std::vector<float>> load_motion_csv(
    const std::string & motion_file,
    const std::vector<std::string> & fallback_joint_order);
  static std::vector<Eigen::VectorXf> compute_forward_derivative(
    const std::vector<Eigen::VectorXf> & xs, float dt);

  // Current interpolation cursor.
  float dt_{0.02f};
  int num_frames_{0};
  float duration_{0.0f};
  int index_0_{0};
  int index_1_{0};
  float blend_{0.0f};

  // Motion data in motion-file joint order.
  std::vector<std::string> joint_order_;
  std::unordered_map<std::string, Eigen::Index> joint_index_by_name_;
  std::vector<Eigen::VectorXf> root_positions_;
  std::vector<Eigen::Quaternionf> root_quaternions_;
  std::vector<Eigen::VectorXf> dof_positions_;
  std::vector<Eigen::VectorXf> dof_velocities_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__MOTION_REFERENCE_HPP_
