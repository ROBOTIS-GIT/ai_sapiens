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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__ACTION_VALIDATION_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__ACTION_VALIDATION_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace ai_sapiens_sim2real
{
struct ActionSafetyLimits
{
  float absolute_position_rad{2.f * 3.14159265f};
  float delta_rad{3.14159265f / 2.f};
  // Detection only, including for Mimic exports without an actual Clip operation.
  float raw_action_boundary{10.f};
};

enum class ActionRejection {None, Nonfinite, AbsoluteLimit, InvalidReference, BoundaryAndDelta};
enum class BoundaryCheckState {Disabled, NotNeeded, Completed, SkippedInvalidAction};

struct JointActionCheck
{
  std::optional<float> reference;
  float delta{0.f};
  bool has_large_delta{false};
  bool is_at_raw_action_boundary{false};
  ActionRejection rejection{ActionRejection::None};
};

struct ActionValidation
{
  BoundaryCheckState boundary_check{BoundaryCheckState::Disabled};
  std::vector<JointActionCheck> joints;

  bool is_delta_enabled() const {return boundary_check != BoundaryCheckState::Disabled;}

  bool has_rejections() const
  {
    return std::any_of(joints.begin(), joints.end(), [](const auto & joint) {
               return joint.rejection != ActionRejection::None;
    });
  }

  bool is_valid() const {return !has_rejections();}
};

// A null reference explicitly disables the delta rule for this inference.
inline ActionValidation validate_action(
  const std::vector<float> & raw, const std::vector<float> & processed,
  const std::vector<float> * reference,
  const ActionSafetyLimits & limits)
{
  if (raw.size() != processed.size()) {
    throw std::runtime_error("Raw action size does not match processed action");
  }

  if (reference && reference->size() != processed.size()) {
    throw std::runtime_error("Action reference size does not match processed action");
  }

  ActionValidation result;
  result.joints.resize(processed.size());
  bool has_large_delta = false;

  for (size_t j = 0; j < processed.size(); ++j) {
    auto & check = result.joints[j];

    if (reference) {
      check.reference = (*reference)[j];
      check.delta = processed[j] - *check.reference;
      check.has_large_delta = std::abs(check.delta) >= limits.delta_rad;
      has_large_delta |= check.has_large_delta;
    }

    if (!std::isfinite(processed[j])) {
      check.rejection = ActionRejection::Nonfinite;
    } else if (std::abs(processed[j]) > limits.absolute_position_rad) {
      check.rejection = ActionRejection::AbsoluteLimit;
    } else if (check.reference && !std::isfinite(*check.reference)) {
      check.rejection = ActionRejection::InvalidReference;
    }
  }

  if (!reference) {
    return result;
  }

  if (result.has_rejections()) {
    result.boundary_check = BoundaryCheckState::SkippedInvalidAction;
    return result;
  }

  if (!has_large_delta) {
    result.boundary_check = BoundaryCheckState::NotNeeded;
    return result;
  }

  for (size_t j = 0; j < result.joints.size(); ++j) {
    auto & check = result.joints[j];
    check.is_at_raw_action_boundary = std::abs(raw[j]) >= limits.raw_action_boundary;

    if (check.is_at_raw_action_boundary && check.has_large_delta) {
      check.rejection = ActionRejection::BoundaryAndDelta;
    }
  }

  result.boundary_check = BoundaryCheckState::Completed;
  return result;
}

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__ACTION_VALIDATION_HPP_
