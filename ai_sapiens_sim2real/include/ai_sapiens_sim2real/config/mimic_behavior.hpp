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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__MIMIC_BEHAVIOR_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__MIMIC_BEHAVIOR_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace ai_sapiens_sim2real
{

// A mimic policy's reference-motion playback config, resolved from the root
// config. RootConfig produces it; MimicPolicyRuntime consumes it.
struct MimicBehavior
{
  std::filesystem::path motion_file;    // absolute path to the reference motion
  float fps{50.0f};
  float time_start{0.0f};               // absolute motion start time
  std::optional<float> time_end;        // absolute motion end time; nullopt: motion's end
  std::string on_complete{"Velocity"};  // state to hand off to when the window ends
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__MIMIC_BEHAVIOR_HPP_
