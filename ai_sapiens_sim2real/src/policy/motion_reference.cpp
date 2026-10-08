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
  const std::vector<std::string> & fallback_joint_order, bool adapter)
: adapter_(adapter)
{
  if (!std::isfinite(fps) || fps <= 0.0f) {
    throw std::runtime_error("MotionReference fps must be finite and positive");
  }

  dt_ = 1.0f / fps;
  if (adapter_) {
    load_adapter_csv(motion_file, fallback_joint_order);
    num_frames_ = static_cast<int>(dof_positions_.size());
    duration_ = num_frames_ * dt_;
    for (size_t i = 0; i < joint_order_.size(); ++i) {
      joint_index_by_name_[joint_order_[i]] = i;
    }
    seek(0.0f);
    return;
  }
  auto data = load_motion_csv(motion_file, fallback_joint_order);
  num_frames_ = static_cast<int>(data.size());
  if (num_frames_ <= 0) {
    throw std::runtime_error("Motion CSV contains no frames: " + motion_file);
  }

  duration_ = num_frames_ * dt_;

  for (int i = 0; i < num_frames_; ++i) {
    root_positions_.push_back(Eigen::VectorXf::Map(data[i].data(), 3));
    root_quaternions_.push_back(
      Eigen::Quaternionf(data[i][6], data[i][3], data[i][4], data[i][5]));
    dof_positions_.push_back(Eigen::VectorXf::Map(data[i].data() + 7, data[i].size() - 7));
  }

  // TODO(kiwoong): Consider central differences for smoother motion velocity after
  // policy revalidation.
  dof_velocities_ = compute_forward_derivative(dof_positions_, dt_);

  for (size_t i = 0; i < joint_order_.size(); ++i) {
    joint_index_by_name_[joint_order_[i]] = static_cast<Eigen::Index>(i);
  }

  seek(0.0f);
}

void MotionReference::load_adapter_csv(
  const std::string & path, const std::vector<std::string> & joints)
{
  joint_order_ = joints;
  validate_joint_order();
  if (joints.empty()) {throw std::runtime_error("Adapter motion requires joint names");}
  std::ifstream file(path);
  std::string line;
  if (!file || !std::getline(file, line)) {
    throw std::runtime_error("Cannot read Adapter CSV: " + path);
  }
  const auto header = split_csv_line(line);
  std::unordered_map<std::string, size_t> columns;
  for (size_t i = 0; i < header.size(); ++i) {
    if (header[i].empty() || !columns.emplace(header[i], i).second) {
      throw std::runtime_error("Empty or duplicate Adapter CSV header");
    }
  }
  std::vector<std::string> keys = {
    "root_x", "root_y", "root_z", "root_qw", "root_qx", "root_qy", "root_qz"};
  keys.insert(keys.end(), joints.begin(), joints.end());
  for (const auto * key : {"root_vx", "root_vy", "root_vz", "root_wx", "root_wy", "root_wz"}) {
    keys.emplace_back(key);
  }
  for (const auto & joint : joints) {
    keys.push_back("velocity:" + joint);
  }
  for (const auto * key : {"height:left_foot", "height:right_foot", "height:left_foot_top",
      "height:right_foot_top", "ref_root_height"})
  {
    keys.emplace_back(key);
  }
  for (const auto & key : keys) {
    if (!columns.count(key)) {throw std::runtime_error("Adapter CSV missing column: " + key);}
  }
  if (columns.size() != keys.size() || trim(line).back() == ',') {
    throw std::runtime_error("Unexpected Adapter CSV columns");
  }
  size_t line_number = 1;
  while (std::getline(file, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty()) {continue;}
    std::vector<float> row;
    if (line.back() == ',' || !parse_numeric_row(split_csv_line(line), row) ||
      row.size() != columns.size() ||
      !std::all_of(row.begin(), row.end(), [](float x) {return std::isfinite(x);}))
    {
      throw std::runtime_error("Invalid Adapter CSV row " + std::to_string(line_number));
    }
    auto value = [&](const std::string & key) {return row[columns.at(key)];};
    Eigen::Quaternionf q(value("root_qw"), value("root_qx"), value("root_qy"), value("root_qz"));
    if (std::abs(q.squaredNorm() - 1.0f) > 1e-3f) {
      throw std::runtime_error("Adapter CSV quaternion must have unit norm");
    }
    root_positions_.emplace_back(Eigen::Vector3f(value("root_x"), value("root_y"),
        value("root_z")));
    root_quaternions_.push_back(q);
    Eigen::VectorXf positions(joints.size()), velocities(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
      positions[i] = value(joints[i]);
      velocities[i] = value("velocity:" + joints[i]);
    }
    dof_positions_.push_back(positions);
    dof_velocities_.push_back(velocities);
    feet_heights_.emplace_back(value("height:left_foot"), value("height:right_foot"),
      value("height:left_foot_top"), value("height:right_foot_top"));
    root_heights_.push_back(value("ref_root_height"));
  }
  if (!file.eof() || dof_positions_.empty()) {throw std::runtime_error("Incomplete Adapter CSV");}
}

void MotionReference::seek(float time)
{
  if (!std::isfinite(time)) {throw std::runtime_error("Motion time must be finite");}
  if (adapter_) {
    // Tolerance covers float episode-time conversion at exact 50 Hz boundaries.
    const double frame = std::floor(std::clamp(time, 0.0f, duration_) / dt_ + 1e-4);
    index_0_ = std::clamp(static_cast<int>(frame), 0, num_frames_ - 1);
    index_1_ = index_0_;
    blend_ = 0.0f;
    return;
  }
  const float phase = std::clamp(time / duration_, 0.0f, 1.0f);
  // TODO(kiwoong): Consider switching to floor(frame) + alpha interpolation after
  // validating motion timing against the already-tested ai_sapiens_rl_inference path.
  // Note: the current round-index + blend mix can produce a negative blend.
  index_0_ = static_cast<int>(std::round(phase * (num_frames_ - 1)));
  index_0_ = std::max(0, std::min(index_0_, num_frames_ - 1));
  index_1_ = std::min(index_0_ + 1, num_frames_ - 1);
  const bool has_next_frame = num_frames_ > 1;
  blend_ = has_next_frame ?
    std::round((time - index_0_ * dt_) / dt_ * 1e5f) / 1e5f :
    0.0f;
}

float MotionReference::joint_pos_for_joint(const std::string & joint_name) const
{
  const auto it = joint_index_by_name_.find(joint_name);
  if (it == joint_index_by_name_.end()) {
    throw std::runtime_error("Motion CSV is missing joint: " + joint_name);
  }

  return joint_pos()(it->second);
}

const std::vector<std::string> & MotionReference::joint_order() const
{
  return joint_order_;
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
  for (const auto & joint_name : joint_order_) {
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

        joint_order_.assign(tokens.begin() + 7, tokens.end());
        validate_joint_order();
        continue;
      }
      if (fallback_joint_order.empty()) {
        throw std::runtime_error("Motion CSV has no header; robot_joint_order is required");
      }

      joint_order_ = fallback_joint_order;
      validate_joint_order();
    } else if (!is_numeric_row) {
      throw std::runtime_error("Motion CSV contains a non-numeric data row: " + line);
    }

    if (row.size() != joint_order_.size() + 7U) {
      throw std::runtime_error(
        "Motion CSV row width does not match root columns plus motion joints");
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


}  // namespace ai_sapiens_sim2real
