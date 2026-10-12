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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__ANY2TRACK_POLICY_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__ANY2TRACK_POLICY_HPP_

#include <array>
#include <cstdint>
#include <filesystem>
#include <unordered_map>
#include <vector>
#include <Eigen/Geometry>
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
#include "ai_sapiens_sim2real/config/mimic_behavior.hpp"

namespace ai_sapiens_sim2real
{
// Both ROS and standalone MuJoCo supply these pelvis-frame measurements.
struct Any2TrackSample
{
  Eigen::Vector2f xy{Eigen::Vector2f::Zero()};
  Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f gyro{Eigen::Vector3f::Zero()};
  Eigen::VectorXf position{Eigen::VectorXf::Zero(23)};
  Eigen::VectorXf velocity{Eigen::VectorXf::Zero(23)};
};

class Any2TrackPolicy
{
public:
  // Binary rows: qpos XYZ/WXYZ/joints, qvel world-linear/body-angular/joints,
  // then the four training foot-site world heights, all float32 little endian.
  using Frame = std::array<float, 63>;
  Any2TrackPolicy(
    const Sim2RealConfig & config, const MimicBehavior & mimic,
    const std::filesystem::path & model_path);
  void reset(const Any2TrackSample & sample, bool align_on_entry);
  bool observe(const Any2TrackSample & sample);
  std::vector<float> target(const std::vector<float> & action) const;
  void accept(const std::vector<float> & target);
  void advance() {++cursor_;}
  std::unordered_map<std::string, std::vector<float>> & inputs() {return inputs_;}
  const std::vector<float> & observation() const {return inputs_.at("obs");}
  const Frame & frame(size_t index) const {return frames_.at(index);}
  size_t start_frame() const {return start_;}
  size_t end_frame() const {return end_;}
  size_t cursor() const {return cursor_;}
  const std::string & completion_state() const {return completion_state_;}

private:
  void sensor_features(const Any2TrackSample & sample);
  std::vector<Frame> frames_;
  std::vector<float> defaults_, scale_, last_targets_, current_history_;
  std::unordered_map<std::string, std::vector<float>> inputs_;
  Eigen::Vector2f origin_{Eigen::Vector2f::Zero()}, reference_origin_{Eigen::Vector2f::Zero()};
  Eigen::Quaternionf alignment_{Eigen::Quaternionf::Identity()};
  size_t start_{0}, end_{0}, cursor_{0};
  bool first_{true};
  std::string completion_state_;
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__ANY2TRACK_POLICY_HPP_
