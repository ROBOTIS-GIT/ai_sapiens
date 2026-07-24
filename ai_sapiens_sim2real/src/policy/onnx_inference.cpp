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

#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

namespace ai_sapiens_sim2real
{

OnnxInference::OnnxInference(const std::string & model_path)
{
  configure_session_options();
  session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);
  load_input_metadata(model_path);
  load_output_metadata(model_path);
  allocate_action_buffer();
  log_model_summary(model_path);
}

void OnnxInference::configure_session_options()
{
  // Keep ONNX Runtime single-threaded so policy latency stays predictable.
  env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "ai_sapiens_sim2real_onnx");
  session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
  session_options_.SetIntraOpNumThreads(1);
  session_options_.SetInterOpNumThreads(1);
  session_options_.SetExecutionMode(ORT_SEQUENTIAL);
}

void OnnxInference::load_input_metadata(const std::string & model_path)
{
  if (session_->GetInputCount() == 0) {
    throw std::runtime_error("ONNX model has no inputs: " + model_path);
  }

  // Get input information.
  for (size_t i = 0; i < session_->GetInputCount(); ++i) {
    Ort::TypeInfo input_type = session_->GetInputTypeInfo(i);
    const auto input_shape = input_type.GetTensorTypeAndShapeInfo().GetShape();
    input_shapes_.push_back(input_shape);
    normalized_input_shapes_.push_back(normalize_policy_shape(input_shape,
        "input " + std::to_string(i)));
    auto input_name = session_->GetInputNameAllocated(i, allocator_);
    input_names_.emplace_back(input_name.get());
  }

  cache_name_pointers(input_names_, input_name_ptrs_);

  // Calculate input sizes.
  for (size_t i = 0; i < normalized_input_shapes_.size(); ++i) {
    input_sizes_.push_back(tensor_size(normalized_input_shapes_[i],
        "input " + std::to_string(i)));
  }
}

void OnnxInference::load_output_metadata(const std::string & model_path)
{
  if (session_->GetOutputCount() != 1) {
    throw std::runtime_error(
      "ONNX policy model must have exactly one output, found " +
      std::to_string(session_->GetOutputCount()) + ": " + model_path);
  }

  // Get output information.
  Ort::TypeInfo output_type = session_->GetOutputTypeInfo(0);
  output_shape_ = output_type.GetTensorTypeAndShapeInfo().GetShape();
  normalized_output_shape_ = normalize_policy_shape(output_shape_, "output");
  output_size_ = tensor_size(normalized_output_shape_, "output");
  auto output_name = session_->GetOutputNameAllocated(0, allocator_);
  output_names_.emplace_back(output_name.get());
  cache_name_pointers(output_names_, output_name_ptrs_);
}

void OnnxInference::allocate_action_buffer()
{
  // Pre-allocate action buffer.
  action_.resize(output_size_);
}

void OnnxInference::log_model_summary(const std::string & model_path) const
{
  std::cout << "[OnnxInference] Model loaded: " << model_path << std::endl;
  std::cout << "[OnnxInference] Input count: " << input_names_.size() << std::endl;
  for (size_t i = 0; i < input_names_.size(); ++i) {
    std::cout << "[OnnxInference] Input " << i << ": " << input_names_[i]
              << " (shape=" << shape_to_string(input_shapes_[i])
              << ", size=" << input_sizes_[i] << ")" << std::endl;
  }

  std::cout << "[OnnxInference] Output: " << output_names_[0]
            << " (shape=" << shape_to_string(output_shape_)
            << ", size=" << output_size_ << ")" << std::endl;
}

std::vector<float> OnnxInference::run(std::unordered_map<std::string, std::vector<float>> & obs)
{
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
  validate_inputs(obs);
  auto input_tensors = create_input_tensors(obs, memory_info);

  // Run inference
  auto output_tensor = session_->Run(
          Ort::RunOptions{nullptr},
          input_name_ptrs_.data(),
          input_tensors.data(),
          input_tensors.size(),
          output_name_ptrs_.data(),
          1
  );

  return copy_output_to_action(output_tensor);
}

void OnnxInference::validate_inputs(
  const std::unordered_map<std::string, std::vector<float>> & obs) const
{
  for (const auto & name : input_names_) {
    if (obs.find(name) == obs.end()) {
      throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
    }
  }
}

std::vector<Ort::Value> OnnxInference::create_input_tensors(
  std::unordered_map<std::string, std::vector<float>> & obs,
  const Ort::MemoryInfo & memory_info)
{
  std::vector<Ort::Value> input_tensors;
  input_tensors.reserve(input_names_.size());
  for (size_t i = 0; i < input_names_.size(); ++i) {
    const std::string name_str(input_names_[i]);
    auto & input_data = obs.at(name_str);
    const size_t expected_size = static_cast<size_t>(input_sizes_[i]);
    const size_t actual_size = input_data.size();
    if (actual_size != expected_size) {
      throw std::runtime_error(
        "Input '" + name_str + "' expects " + std::to_string(expected_size) +
        " values, received " + std::to_string(actual_size));
    }

    auto input_tensor = Ort::Value::CreateTensor<float>(
      memory_info,
      input_data.data(),
      input_sizes_[i],
      normalized_input_shapes_[i].data(),
      normalized_input_shapes_[i].size()
    );
    input_tensors.push_back(std::move(input_tensor));
  }

  return input_tensors;
}

std::vector<float> OnnxInference::copy_output_to_action(std::vector<Ort::Value> & output_tensor)
{
  if (output_tensor.size() != 1) {
    throw std::runtime_error(
      "ONNX inference expected exactly one output, received " +
      std::to_string(output_tensor.size()));
  }
  if (!output_tensor.front().IsTensor()) {
    throw std::runtime_error("ONNX inference output must be a tensor");
  }

  const size_t expected_output_size = static_cast<size_t>(output_size_);
  const auto actual_output_size =
    output_tensor.front().GetTensorTypeAndShapeInfo().GetElementCount();
  if (actual_output_size != expected_output_size) {
    throw std::runtime_error(
      "ONNX output size mismatch: expected " + std::to_string(expected_output_size) +
      ", received " + std::to_string(actual_output_size));
  }

  auto floatarr = output_tensor.front().GetTensorMutableData<float>();
  std::lock_guard<std::mutex> lock(action_mtx_);
  std::memcpy(action_.data(), floatarr, output_size_ * sizeof(float));
  return action_;
}

std::vector<float> OnnxInference::get_action() const
{
  std::lock_guard<std::mutex> lock(action_mtx_);
  return action_;
}

const std::vector<std::string> & OnnxInference::get_input_names() const
{
  return input_names_;
}

const std::vector<int64_t> & OnnxInference::get_input_sizes() const
{
  return input_sizes_;
}

void OnnxInference::cache_name_pointers(
  const std::vector<std::string> & names,
  std::vector<const char *> & ptrs)
{
  ptrs.clear();
  ptrs.reserve(names.size());
  for (const auto & name : names) {
    ptrs.push_back(name.c_str());
  }
}

std::vector<int64_t> OnnxInference::normalize_policy_shape(
  const std::vector<int64_t> & shape,
  const std::string & label)
{
  if (shape.empty()) {
    throw std::runtime_error("ONNX " + label + " tensor shape is empty");
  }

  std::vector<int64_t> normalized_shape;
  normalized_shape.reserve(shape.size());
  for (size_t i = 0; i < shape.size(); ++i) {
    const auto dim = shape[i];
    const bool is_dynamic_dim = dim < 0;
    if (is_dynamic_dim && i != 0) {
      throw std::runtime_error(
        "ONNX " + label + " has dynamic non-batch dimension " +
        shape_to_string(shape) +
        ". Only the leading batch dimension may be dynamic.");
    }

    normalized_shape.push_back(is_dynamic_dim ? 1 : dim);
  }

  return normalized_shape;
}

int64_t OnnxInference::tensor_size(const std::vector<int64_t> & shape, const std::string & label)
{
  int64_t size = 1;
  for (const auto dim : shape) {
    if (dim <= 0) {
      throw std::runtime_error("Invalid ONNX " + label + " shape " + shape_to_string(shape));
    }
    if (size > std::numeric_limits<int64_t>::max() / dim) {
      throw std::runtime_error("ONNX " + label + " shape is too large " + shape_to_string(shape));
    }

    size *= dim;
  }

  return size;
}

std::string OnnxInference::shape_to_string(const std::vector<int64_t> & shape)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }

    oss << shape[i];
  }

  oss << "]";
  return oss.str();
}


}  // namespace ai_sapiens_sim2real
