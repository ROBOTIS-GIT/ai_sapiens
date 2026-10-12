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

#include "ai_sapiens_sim2real/policy/any2track_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <openssl/evp.h>

namespace ai_sapiens_sim2real
{
namespace
{
void require(bool ok, const std::string & message)
{
  if (!ok) {throw std::runtime_error("Any2Track: " + message);}
}
std::string sha256(const std::filesystem::path & path)
{
  std::ifstream file(path, std::ios::binary);
  require(file.good(), "cannot open " + path.string());
  std::unique_ptr<EVP_MD_CTX, decltype(& EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  require(ctx && EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) == 1, "SHA256 init failed");
  char buffer[65536];
  while (file.read(buffer, sizeof(buffer)) || file.gcount()) {
    require(EVP_DigestUpdate(ctx.get(), buffer, file.gcount()) == 1, "SHA256 update failed");
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int size = 0;
  require(EVP_DigestFinal_ex(ctx.get(), digest, &size) == 1, "SHA256 failed");
  std::ostringstream text;
  for (unsigned int i = 0; i < size; ++i) {
    text << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(digest[i]);
  }
  return text.str();
}
float yaw(const Eigen::Quaternionf & q)
{
  return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
    1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}
Eigen::Quaternionf rotation(const Any2TrackPolicy::Frame & f)
{
  return Eigen::Quaternionf(f[3], f[4], f[5], f[6]).normalized();
}
void validate_sample(const Any2TrackSample & s)
{
  require(s.position.size() == 23 && s.velocity.size() == 23 &&
    s.position.allFinite() && s.velocity.allFinite() && s.xy.allFinite() &&
    s.gyro.allFinite() && s.orientation.coeffs().allFinite() &&
    std::abs(s.orientation.norm() - 1.0f) < 0.01f, "invalid pelvis/joint sample");
}
}  // namespace

Any2TrackPolicy::Any2TrackPolicy(
  const Sim2RealConfig & config, const MimicBehavior & mimic,
  const std::filesystem::path & model_path)
: defaults_(config.joint_properties().default_position), scale_(config.action_properties().scale),
  last_targets_(23), current_history_(52), completion_state_(mimic.on_complete)
{
  require(config.any2track(), "requires any2track_gt_xy_adapter");
  require(std::abs(mimic.fps - 50.0f) < 1e-5 && mimic.mjlab_format.value_or(true),
      "requires 50 Hz MJLab CSV");
  const auto folder = config.path().parent_path();
  const auto manifest = YAML::LoadFile((folder / "any2track_reference.yaml").string());
  require(manifest["version"].as<int>() == 1 && manifest["columns"].as<int>() == 63 &&
    manifest["step_dt"].as<double>() == config.step_dt() &&
    manifest["joint_names"].as<std::vector<std::string>>() == config.policy_joints(),
      "reference schema mismatch");
  const auto binary = folder / manifest["reference_file"].as<std::string>();
  for (const auto & item : std::vector<std::pair<std::string, std::filesystem::path>>{
      {"reference_sha256", binary}, {"motion_csv_sha256", mimic.motion_file},
      {"policy_sha256", model_path}, {"sim2real_sha256", config.path()}})
  {
    require(sha256(item.second) == manifest[item.first].as<std::string>(),
        "asset hash mismatch: " + item.first);
  }
  std::ifstream in(binary, std::ios::binary);
  char magic[8];
  uint32_t count = 0;
  in.read(magic, 8);
  in.read(reinterpret_cast<char *>(&count), 4);
  const uint32_t endian = 1;
  require(*reinterpret_cast<const char *>(&endian) == 1, "reference requires a little-endian host");
  require(in && std::memcmp(magic, "A2TREF01", 8) == 0 && count >= 2 && count <= 1000000 &&
    count == manifest["frames"].as<uint32_t>(), "invalid reference header");
  frames_.resize(count);
  in.read(reinterpret_cast<char *>(frames_.data()), count * sizeof(Frame));
  require(in && in.peek() == std::char_traits<char>::eof(), "invalid reference length");
  for (const auto & f : frames_) {
    require(std::all_of(f.begin(), f.end(), [](float x) {
        return std::isfinite(x);
        }), "nonfinite reference");
    require(std::abs(Eigen::Quaternionf(f[3], f[4], f[5], f[6]).norm() - 1) < 1e-3,
      "reference quaternion must be WXYZ unit length");
  }
  const auto frame_index = [&](double time) {
      const double f = time / config.step_dt();
      require(std::isfinite(f) && f >= 0 && f <= count - 1 && std::abs(f - std::round(f)) < 1e-3,
        "motion window must lie on reference frames");
      return static_cast<size_t>(std::llround(f));
    };
  start_ = frame_index(mimic.time_start);
  end_ = mimic.time_end ? frame_index(*mimic.time_end) : count - 1;
  require(start_ < end_, "motion window requires at least two frames");
  require(completion_state_ != "stay", "finite path must hand off at motion end");
  inputs_["obs"].resize(134);
  inputs_["velocity_history"].resize(20 * 75);
  inputs_["history"].resize(75 * 79);
}

void Any2TrackPolicy::sensor_features(const Any2TrackSample & s)
{
  validate_sample(s);
  const Eigen::Vector3f gravity = s.orientation.normalized().conjugate() * Eigen::Vector3f(0, 0,
      -1);
  // Adapter history has GYRO first; velocity-estimator history has GRAVITY first.
  for (int j = 0; j < 3; ++j) {
    current_history_[j] = s.gyro[j] * 0.05f;
    current_history_[3 + j] = gravity[j];
  }
  for (int j = 0; j < 23; ++j) {
    current_history_[6 + j] = s.position[j] - defaults_[j];
    current_history_[29 + j] = s.velocity[j] * 0.05f;
  }
}

void Any2TrackPolicy::reset(const Any2TrackSample & s, bool align_on_entry)
{
  sensor_features(s);
  cursor_ = start_ + 1;  // reset observes the upcoming frame, as in training
  first_ = true;
  const auto & f = frames_[start_];
  alignment_ = align_on_entry ? Eigen::Quaternionf(Eigen::AngleAxisf(
    yaw(rotation(f)) - yaw(s.orientation.normalized()),
      Eigen::Vector3f::UnitZ())) : Eigen::Quaternionf::Identity();
  origin_ = align_on_entry ? s.xy : Eigen::Vector2f::Zero();
  reference_origin_ = align_on_entry ? Eigen::Vector2f(f[0], f[1]) : Eigen::Vector2f::Zero();
  // Training reset sets targets to the actual reset joint pose, not zero/raw action.
  for (int j = 0; j < 23; ++j) {
    last_targets_[j] = s.position[j];
  }
  auto & history = inputs_.at("history");
  for (int c = 0; c < 75; ++c) {
    const float value = c < 52 ? current_history_[c] : last_targets_[c - 52];
    std::fill_n(history.begin() + c * 79, 79, value);
  }
}

bool Any2TrackPolicy::observe(const Any2TrackSample & s)
{
  if (cursor_ > end_) {return false;}
  sensor_features(s);
  const auto & f = frames_[cursor_];
  auto & obs = inputs_.at("obs");
  for (int j = 0; j < 23; ++j) {
    obs[j] = f[7 + j] - s.position[j];
    obs[23 + j] = (f[36 + j] - s.velocity[j]) * 0.05f;
    obs[52 + j] = current_history_[6 + j];
    obs[75 + j] = current_history_[29 + j];
    obs[98 + j] = last_targets_[j];
  }
  for (int j = 0; j < 3; ++j) {
    obs[46 + j] = current_history_[3 + j];
    obs[49 + j] = current_history_[j];
  }
  const Eigen::Quaternionf robot = (alignment_ * s.orientation.normalized()).normalized();
  const Eigen::Matrix3f relative = (robot.conjugate() * rotation(f)).toRotationMatrix();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 2; ++col) {
      obs[121 + row * 2 + col] = relative(row, col);
    }
  }
  const Eigen::Vector2f robot_xy = reference_origin_ + (alignment_ *
    Eigen::Vector3f(s.xy.x() - origin_.x(), s.xy.y() - origin_.y(), 0)).head<2>();
  const Eigen::Vector2f error = Eigen::Rotation2Df(-yaw(robot)) * (Eigen::Vector2f(f[0],
      f[1]) - robot_xy);
  obs[127] = error.x(); obs[128] = error.y();
  for (int j = 0; j < 4; ++j) {
    obs[129 + j] = f[59 + j];
  }
  obs[133] = f[2];
  std::array<float, 75> velocity{};
  std::copy_n(obs.begin() + 46, 6, velocity.begin());
  std::copy_n(obs.begin() + 52, 69, velocity.begin() + 6);
  auto & history = inputs_.at("velocity_history");
  if (first_) {
    for (int t = 0; t < 20; ++t) {
      std::copy(velocity.begin(), velocity.end(), history.begin() + t * 75);
    }
    first_ = false;
  } else {
    std::move(history.begin() + 75, history.end(), history.begin());
    std::copy(velocity.begin(), velocity.end(), history.end() - 75);
  }
  return true;
}

std::vector<float> Any2TrackPolicy::target(const std::vector<float> & action) const
{
  require(action.size() == 23 && cursor_ <= end_, "invalid action or reference cursor");
  std::vector<float> result(23);
  for (int j = 0; j < 23; ++j) {
    require(std::isfinite(action[j]) && std::abs(action[j]) <= 1.00001f, "invalid tanh residual");
    result[j] = frames_[cursor_][7 + j] + action[j] * scale_[j];
  }
  return result;
}

void Any2TrackPolicy::accept(const std::vector<float> & target)
{
  require(target.size() == 23 && std::all_of(target.begin(), target.end(),
    [](float x) {return std::isfinite(x);}), "invalid motor target");
  auto & h = inputs_.at("history");
  for (int c = 0; c < 75; ++c) {
    std::move(h.begin() + c * 79 + 1, h.begin() + (c + 1) * 79, h.begin() + c * 79);
    h[(c + 1) * 79 - 1] = c < 52 ? current_history_[c] : target[c - 52];
  }
  last_targets_ = target;
}
}  // namespace ai_sapiens_sim2real
