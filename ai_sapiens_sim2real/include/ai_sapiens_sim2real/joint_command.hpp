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
// Author: Woojin Wie, Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__JOINT_COMMAND_HPP_
#define AI_SAPIENS_SIM2REAL__JOINT_COMMAND_HPP_

#include <cstddef>
#include <vector>

namespace ai_sapiens_sim2real
{

// Position and PD gains form one command. The owner defines its joint order:
// policy targets use policy order; output and published commands use robot order.
struct JointCommand
{
  std::vector<float> position;
  std::vector<float> stiffness;
  std::vector<float> damping;

  void resize(size_t size)
  {
    position.resize(size);
    stiffness.resize(size);
    damping.resize(size);
  }
};

// A skipped inference is distinct from failure: it retains the last valid target.
enum class PolicyUpdateResult
{
  Skipped,
  TargetReady,
  TargetUnavailable,
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__JOINT_COMMAND_HPP_
