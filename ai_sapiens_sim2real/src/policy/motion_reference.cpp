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

#include "ai_sapiens_sim2real/policy/motion_reference.hpp"

#include <cmath>

namespace ai_sapiens_sim2real
{

MotionReference::MotionReference(
  const std::string & motion_file,
  float fps,
  const std::vector<std::string> & fallback_joint_order)
{
  if (!std::isfinite(fps) || fps <= 0.0f) {
    throw std::runtime_error("MotionReference fps must be finite and positive");
  }

  dt_ = 1.0 / fps;
  auto data = load_motion_csv(motion_file, fallback_joint_order);
  num_frames_ = static_cast<int>(data.size());
  if (num_frames_ <= 0) {
    throw std::runtime_error("Motion CSV contains no frames: " + motion_file);
  }

  duration_ = (num_frames_ - 1) * dt_;

  for (int i = 0; i < num_frames_; ++i) {
    frames_->root_positions.push_back(Eigen::VectorXf::Map(data[i].data(), 3));
    frames_->root_quaternions.push_back(
      Eigen::Quaternionf(data[i][6], data[i][3], data[i][4], data[i][5]));
    const float norm = frames_->root_quaternions.back().norm();
    if (!frames_->root_quaternions.back().coeffs().allFinite() ||
      !std::isfinite(norm) || norm < 1.0e-6f)
    {
      throw std::runtime_error("Motion has an invalid root quaternion");
    }
    frames_->root_quaternions.back().normalize();
    frames_->dof_positions.push_back(Eigen::VectorXf::Map(data[i].data() + 7, data[i].size() - 7));
  }

  // Match training: central differences inside, one-sided differences at endpoints.
  frames_->dof_velocities = compute_central_derivative(frames_->dof_positions, dt_);

  for (size_t i = 0; i < frames_->joint_order.size(); ++i) {
    frames_->joint_index_by_name[frames_->joint_order[i]] = static_cast<Eigen::Index>(i);
  }

  seek(0.0);
}

void MotionReference::seek(double time)
{
  if (!std::isfinite(time)) {
    throw std::runtime_error("Motion time must be finite");
  }
  const double frame = std::clamp(time / dt_, 0.0, static_cast<double>(num_frames_ - 1));
  // Snap nominal ticks despite decimal configuration and floating-point roundoff.
  const double nearest = std::round(frame);
  const double cursor = std::abs(frame - nearest) < 1.0e-3 ? nearest : frame;
  index_0_ = static_cast<int>(std::floor(cursor));
  index_1_ = std::min(index_0_ + 1, num_frames_ - 1);
  blend_ = static_cast<float>(cursor - index_0_);
}

float MotionReference::joint_pos_for_joint(const std::string & joint_name) const
{
  const auto it = frames_->joint_index_by_name.find(joint_name);
  if (it == frames_->joint_index_by_name.end()) {
    throw std::runtime_error("Motion CSV is missing joint: " + joint_name);
  }

  return joint_pos()(it->second);
}

const std::vector<std::string> & MotionReference::joint_order() const
{
  return frames_->joint_order;
}

std::string MotionReference::trim(const std::string & value)
{
  size_t begin = 0;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }

  size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }

  return value.substr(begin, end - begin);
}

std::vector<std::string> MotionReference::split_csv_line(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    tokens.push_back(trim(token));
  }

  return tokens;
}

bool MotionReference::parse_float(const std::string & token, float & value)
{
  try {
    size_t pos = 0;
    value = std::stof(token, &pos);
    return pos == token.size();
  } catch (const std::exception &) {
    return false;
  }
}

bool MotionReference::parse_numeric_row(
  const std::vector<std::string> & tokens,
  std::vector<float> & row)
{
  row.clear();
  row.reserve(tokens.size());
  for (const auto & token : tokens) {
    float value = 0.0f;
    if (!parse_float(token, value)) {
      row.clear();
      return false;
    }

    row.push_back(value);
  }

  return true;
}

void MotionReference::validate_joint_order() const
{
  std::unordered_map<std::string, bool> seen;
  for (const auto & joint_name : frames_->joint_order) {
    if (joint_name.empty()) {
      throw std::runtime_error("Motion CSV joint header contains an empty joint name");
    }
    if (seen.count(joint_name) > 0) {
      throw std::runtime_error(
        "Motion CSV joint header contains duplicate joint name: " + joint_name);
    }

    seen[joint_name] = true;
  }
}

std::vector<std::vector<float>> MotionReference::load_motion_csv(
  const std::string & motion_file,
  const std::vector<std::string> & fallback_joint_order)
{
  std::ifstream file(motion_file);
  if (!file.is_open()) {
    throw std::runtime_error("Error opening motion CSV: " + motion_file);
  }

  std::vector<std::vector<float>> data;
  std::string line;
  bool is_first_data_line = true;
  while (std::getline(file, line)) {
    if (trim(line).empty()) {
      continue;
    }

    auto tokens = split_csv_line(line);
    std::vector<float> row;
    const bool is_numeric_row = parse_numeric_row(tokens, row);
    if (is_first_data_line) {
      is_first_data_line = false;
      if (!is_numeric_row) {
        if (tokens.size() < 8U) {
          throw std::runtime_error(
            "Motion CSV header must contain root columns and at least one joint");
        }

        frames_->joint_order.assign(tokens.begin() + 7, tokens.end());
        validate_joint_order();
        continue;
      }
      if (fallback_joint_order.empty()) {
        throw std::runtime_error("Motion CSV has no header; robot_joint_order is required");
      }

      frames_->joint_order = fallback_joint_order;
      validate_joint_order();
    } else if (!is_numeric_row) {
      throw std::runtime_error("Motion CSV contains a non-numeric data row: " + line);
    }

    if (row.size() != frames_->joint_order.size() + 7U) {
      throw std::runtime_error(
        "Motion CSV row width does not match root columns plus motion joints");
    }

    if (!std::all_of(row.begin(), row.end(),
      [](float value) {return std::isfinite(value);}))
    {
      throw std::runtime_error("Motion CSV contains non-finite values");
    }

    data.push_back(std::move(row));
  }

  return data;
}

std::vector<Eigen::VectorXf> MotionReference::compute_forward_derivative(
  const std::vector<Eigen::VectorXf> & xs, float dt)
{
  std::vector<Eigen::VectorXf> dxs;
  dxs.reserve(xs.size());
  if (xs.empty()) {
    return dxs;
  }
  if (xs.size() == 1U) {
    dxs.push_back(Eigen::VectorXf::Zero(xs[0].size()));
    return dxs;
  }

  for (size_t i = 0; i + 1 < xs.size(); ++i) {
    dxs.push_back((xs[i + 1] - xs[i]) / dt);
  }

  dxs.push_back(dxs.back());
  return dxs;
}


std::vector<Eigen::VectorXf> MotionReference::compute_central_derivative(
  const std::vector<Eigen::VectorXf> & xs, float dt)
{
  std::vector<Eigen::VectorXf> dxs;
  dxs.reserve(xs.size());
  if (xs.empty()) {
    return dxs;
  }
  if (xs.size() == 1U) {
    dxs.push_back(Eigen::VectorXf::Zero(xs[0].size()));
    return dxs;
  }

  dxs.push_back((xs[1] - xs[0]) / dt);
  for (size_t i = 1; i + 1 < xs.size(); ++i) {
    dxs.push_back((xs[i + 1] - xs[i - 1]) / (2.0f * dt));
  }
  dxs.push_back((xs.back() - xs[xs.size() - 2]) / dt);
  return dxs;
}


}  // namespace ai_sapiens_sim2real
