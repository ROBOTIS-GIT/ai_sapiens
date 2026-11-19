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

// Author: Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__POLICY__MIMIC__MOTION_SOURCE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MIMIC__MOTION_SOURCE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ai_sapiens_sim2real/policy/mimic/motion_reference.hpp"

namespace ai_sapiens_sim2real
{
// Configuration-scoped asset: one parse/derivative computation, independent
// playback cursors sharing read-only frame data. No process-global cache.
class MotionSource
{
public:
  MotionSource(std::string file, float fps, std::vector<std::string> joints)
  : file_(std::move(file)), fps_(fps), joints_(std::move(joints)) {}

  std::shared_ptr<MotionReference> make_cursor() const
  {
    std::call_once(loaded_, [this]() {
        reference_ = std::make_unique<const MotionReference>(file_, fps_, joints_);
      });
    return std::make_shared<MotionReference>(*reference_);
  }

private:
  std::string file_;
  float fps_;
  std::vector<std::string> joints_;
  mutable std::once_flag loaded_;
  mutable std::unique_ptr<const MotionReference> reference_;
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__MIMIC__MOTION_SOURCE_HPP_
