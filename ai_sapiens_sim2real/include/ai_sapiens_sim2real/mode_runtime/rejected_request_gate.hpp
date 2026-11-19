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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__REJECTED_REQUEST_GATE_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__REJECTED_REQUEST_GATE_HPP_

#include <cstdint>
#include <optional>

namespace ai_sapiens_sim2real
{

// Cancels a rejected execution attempt until a valid input releases its code.
// Selector changes and missing samples cannot release it. No state names or
// device-specific switch mappings belong here.
class RejectedRequestGate
{
public:
  bool blocked() const {return rejected_input_.has_value();}
  void reject(uint16_t input_code) {rejected_input_ = input_code;}
  void observe(bool available, uint16_t input_code)
  {
    if (available && rejected_input_ && input_code != *rejected_input_) {
      rejected_input_.reset();
    }
  }

private:
  std::optional<uint16_t> rejected_input_;
};

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__REJECTED_REQUEST_GATE_HPP_
