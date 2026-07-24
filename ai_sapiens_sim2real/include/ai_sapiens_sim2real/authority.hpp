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

#ifndef AI_SAPIENS_SIM2REAL__AUTHORITY_HPP_
#define AI_SAPIENS_SIM2REAL__AUTHORITY_HPP_

namespace ai_sapiens_sim2real
{

// One of this package's two control axes (the other is the mode state machine).
// Authority is *who* may command the robot:
//   Manual    - the teleop operator drives; the default and the failsafe owner.
//   ApiWarmup - API authority requested, settling for a short window before Api.
//   Api       - an external API client drives via the mode services and cmd_vel.
// AuthorityRuntime owns the transitions; ModeController drives them each tick.
enum class Authority
{
  Manual,
  ApiWarmup,
  Api,
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__AUTHORITY_HPP_
