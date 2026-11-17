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

#include "ai_sapiens_sim2real/policy/policy_entry_validator.hpp"

#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/motion_source.hpp"

namespace ai_sapiens_sim2real
{
PolicyEntryValidator PolicyEntryValidator::from_config(const RootConfig & config)
{
  References references;
  for (const auto & behavior : config.policy_behaviors()) {
    if (!behavior.mimic) {
      continue;
    }
    const auto & mimic = *behavior.mimic;
    auto motion = mimic.require_motion_source().make_cursor();
    motion->seek(mimic.time_start);
    references.emplace(behavior.name, motion->root_quaternion());
  }
  return PolicyEntryValidator(std::move(references));
}
}  // namespace ai_sapiens_sim2real
