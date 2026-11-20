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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__BLINKMIMIC__BLINKMIMIC_OBSERVATION_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__BLINKMIMIC__BLINKMIMIC_OBSERVATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_history_buffer.hpp"
#include "ai_sapiens_sim2real/observation/policy_observation.hpp"
#include "ai_sapiens_sim2real/policy/blinkmimic/blinkmimic_config.hpp"
#include "ai_sapiens_sim2real/policy/blinkmimic/blinkmimic_motion.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

class BlinkMimicObservation : public PolicyObservation
{
public:
  BlinkMimicObservation(
    const Sim2RealConfig & config, const MimicBehavior & mimic,
    const std::vector<std::string> & controller_joints, const SharedControlData * data);

  void reset() override;
  std::shared_ptr<const BlinkMimicMotion> prepare_motion(const MimicBehavior & mimic) const;
  void select_motion(std::shared_ptr<const BlinkMimicMotion> motion);
  std::shared_ptr<const BlinkMimicMotion> motion() const {return motion_;}
  std::vector<float> compute() override;
  void reject_latest() override;
  std::vector<ObservationSlice> layout() const override;
  size_t get_observation_size() const override;
  std::string input_name() const override {return "observations";}

  bool has_frame() const {return cursor_ < motion_->size();}
  size_t frame_index() const {return cursor_;}
  void advance() {if (has_frame()) {++cursor_;}}
  const MotionReference * reference() const {return motion_->reference();}

private:
  std::vector<float> sensor_frame();
  void append_goals(std::vector<float> & input) const;

  const BlinkMimicConfig config_;
  const SharedControlData * data_;
  std::shared_ptr<const BlinkMimicMotion> motion_;
  std::vector<size_t> joint_indices_;
  size_t cursor_{0};
  Eigen::Matrix3f alignment_{Eigen::Matrix3f::Identity()};
  bool has_alignment_{false};
  bool had_alignment_before_sample_{false};
  ObservationHistoryBuffer history_;
};

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__BLINKMIMIC__BLINKMIMIC_OBSERVATION_HPP_
