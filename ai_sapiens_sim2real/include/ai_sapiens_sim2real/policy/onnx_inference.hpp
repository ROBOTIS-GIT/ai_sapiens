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

#ifndef AI_SAPIENS_SIM2REAL__POLICY__ONNX_INFERENCE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__ONNX_INFERENCE_HPP_

#include <cstring>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onnxruntime_cxx_api.h"  // NOLINT(build/include_subdir)

namespace ai_sapiens_sim2real
{

/**
 * @brief ONNX Runtime inference wrapper for realtime-safe policy execution.
 *
 * Loads a policy model once, caches model metadata, and reuses the action
 * buffer during inference.
 */
class OnnxInference
{
public:
  /**
   * @brief Construct ONNX inference session from model file.
   * @param model_path Path to the .onnx model file
   */
  explicit OnnxInference(const std::string & model_path);

  /**
   * @brief Run inference with observation map.
   * @param obs Map of input name -> observation vector
   * @return Action vector from policy
   */
  std::vector<float> run(std::unordered_map<std::string, std::vector<float>> & obs);

  /**
   * @brief Get the last computed action (thread-safe).
   * @return Action vector
   */
  std::vector<float> get_action() const;

  /**
   * @brief Get input names for this model.
   * @return Vector of input names
   */
  const std::vector<std::string> & get_input_names() const;

  /**
   * @brief Get input sizes for this model.
   * @return Vector of input sizes
   */
  const std::vector<int64_t> & get_input_sizes() const;

  /**
   * @brief Get output size for this model.
   * @return Output action dimension
   */
  int64_t get_output_size() const
  {
    return output_size_;
  }

private:
  // Startup phases.
  void configure_session_options();
  void load_input_metadata(const std::string & model_path);
  void load_output_metadata(const std::string & model_path);
  void allocate_action_buffer();
  void log_model_summary(const std::string & model_path) const;

  // Inference steps. Keep allocations explicit and close to the ONNX call.
  void validate_inputs(const std::unordered_map<std::string, std::vector<float>> & obs) const;
  std::vector<Ort::Value> create_input_tensors(
    std::unordered_map<std::string, std::vector<float>> & obs,
    const Ort::MemoryInfo & memory_info);
  std::vector<float> copy_output_to_action(std::vector<Ort::Value> & output_tensor);

  // Shape/name helpers run during startup; inference reuses cached pointers.
  static void cache_name_pointers(
    const std::vector<std::string> & names,
    std::vector<const char *> & ptrs);
  static std::vector<int64_t> normalize_policy_shape(
    const std::vector<int64_t> & shape,
    const std::string & label);
  static int64_t tensor_size(const std::vector<int64_t> & shape, const std::string & label);
  static std::string shape_to_string(const std::vector<int64_t> & shape);

  // ONNX Runtime objects are created once and reused in the RT update path.
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  // Stable strings own the memory; pointer vectors are passed to ONNX Runtime.
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char *> input_name_ptrs_;
  std::vector<const char *> output_name_ptrs_;

  // Cached tensor metadata.
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> normalized_input_shapes_;
  std::vector<int64_t> input_sizes_;
  std::vector<int64_t> output_shape_;
  std::vector<int64_t> normalized_output_shape_;
  int64_t output_size_{0};

  // Last action is exposed to non-RT readers through a short mutex.
  mutable std::mutex action_mtx_;
  std::vector<float> action_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__ONNX_INFERENCE_HPP_
