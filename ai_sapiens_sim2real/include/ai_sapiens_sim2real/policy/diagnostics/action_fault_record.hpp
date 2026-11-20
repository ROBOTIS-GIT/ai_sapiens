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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__ACTION_FAULT_RECORD_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__ACTION_FAULT_RECORD_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_layout.hpp"
#include "ai_sapiens_sim2real/policy/policy_action_result.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{
struct FaultPolicyMetadata
{
  std::string state, model, config_path, config_yaml;
  std::vector<std::string> policy_joints, controller_joints, input_names;
  std::vector<std::vector<int64_t>> input_shapes;
  std::unordered_map<std::string, std::vector<ObservationSlice>> layouts;
  double step_dt{0.0};
  float action_limit{0.0f};
  float action_change_limit{0.0f};
  float raw_action_boundary{0.0f};
};

struct FaultInferenceCall
{
  std::unordered_map<std::string, std::vector<float>> inputs;
  std::vector<float> output;
};

struct FaultSnapshot
{
  int64_t unix_time_ns{0};
  double episode_time{0.0}, sensor_interval{0.0};
  SensorData sensors;
  std::optional<SensorData> previous_sensors;
  std::vector<FaultInferenceCall> calls;
  std::vector<float> processed_action;
  ActionValidation validation;
  std::string error;
  PolicyActionOutcome outcome{PolicyActionOutcome::Retry};
  size_t consecutive_failures{0};
};

struct ActionFaultRecord
{
  std::shared_ptr<const FaultPolicyMetadata> metadata;
  FaultSnapshot current;
  std::optional<FaultSnapshot> previous_accepted;
  uint64_t incident{0};
  size_t attempt{0};
  std::filesystem::path path;
  size_t queued_bytes{0};
};
}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__DIAGNOSTICS__ACTION_FAULT_RECORD_HPP_
