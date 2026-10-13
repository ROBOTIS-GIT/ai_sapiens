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
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"

namespace ai_sapiens_sim2real
{
namespace
{
void require(bool ok, const std::string & message)
{
  if (!ok) {throw std::runtime_error("Any2Track: " + message);}
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
  const Sim2RealConfig & config, const MimicBehavior & mimic)
: defaults_(config.joint_properties().default_position), scale_(config.action_properties().scale),
  last_targets_(23), current_history_(52), path_tracking_(config.any2track_path_tracking()),
  completion_state_(mimic.on_complete)
{
  require(config.any2track(), "requires a supported Any2Track adapter");
  require(std::abs(mimic.fps - 50.0f) < 1e-5 && mimic.mjlab_format.value_or(true),
      "requires 50 Hz MJLab CSV");
  // Use the same CSV loader and installed K1 kinematics as the existing Mimic
  // runtime. Reference FK is computed once at load, never from live simulator state.
  MotionReference motion(mimic.motion_file.string(), mimic.fps, config.policy_joints(), true);
  const int count = motion.frame_count();
  require(count >= 2, "motion requires at least two frames");
  const auto xml = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("ai_sapiens_description")) / "mujoco/k1/k1.xml";
  char error[1024]{};
  std::unique_ptr<mjModel, decltype(& mj_deleteModel)> model(
    mj_loadXML(xml.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  require(static_cast<bool>(model), "cannot load K1 kinematics: " + std::string(error));
  require(model->nq == 30 && model->nv == 29 && model->jnt_type[0] == mjJNT_FREE,
    "expected a free-base K1 model with 23 joints");
  std::unique_ptr<mjData, decltype(& mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  std::array<int, 23> csv_indices{}, qadr{}, vadr{};
  for (int j = 0; j < 23; ++j) {
    const auto & name = config.policy_joints()[j];
    const auto found = std::find(motion.joint_order().begin(), motion.joint_order().end(), name);
    const int joint = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
    require(found != motion.joint_order().end() && joint > 0 &&
      model->jnt_type[joint] == mjJNT_HINGE, "missing K1 motion joint: " + name);
    csv_indices[j] = std::distance(motion.joint_order().begin(), found);
    qadr[j] = model->jnt_qposadr[joint];
    vadr[j] = model->jnt_dofadr[joint];
  }
  const std::array<int, 2> feet{
    mj_name2id(model.get(), mjOBJ_BODY, "left_ankle_roll_link"),
    mj_name2id(model.get(), mjOBJ_BODY, "right_ankle_roll_link")};
  require(feet[0] >= 0 && feet[1] >= 0, "missing K1 foot bodies");
  // Training sites, expressed in the ankle-roll link: sole and front/top.
  // Order: left sole, right sole, left top, right top.
  const std::array<std::array<mjtNum, 3>, 2> foot_points{{
    {{0.0325, 0, -0.0635}}, {{0.10, 0, -0.0535}}}};
  std::vector<std::array<mjtNum, 30>> poses(count);
  for (int i = 0; i < count; ++i) {
    const auto f = motion.frame(i);
    auto & q = poses[i];
    for (int j = 0; j < 3; ++j) {
      q[j] = f.root_position[j];
    }
    q[3] = f.root_quaternion.w(); q[4] = f.root_quaternion.x();
    q[5] = f.root_quaternion.y(); q[6] = f.root_quaternion.z();
    mju_normalize4(q.data() + 3);
    for (int j = 0; j < 23; ++j) {
      q[qadr[j]] = f.joint_pos[csv_indices[j]];
    }
  }
  frames_.resize(count);
  for (int i = 0; i < count; ++i) {
    const int lo = std::max(0, i - 1), hi = std::min(count - 1, i + 1);
    mj_differentiatePos(model.get(), data->qvel, (hi - lo) * config.step_dt(),
      poses[lo].data(), poses[hi].data());
    std::copy(poses[i].begin(), poses[i].end(), data->qpos);
    mj_kinematics(model.get(), data.get());
    auto & f = frames_[i];
    std::copy_n(data->qpos, 7, f.begin());
    std::copy_n(data->qvel, 6, f.begin() + 30);
    for (int j = 0; j < 23; ++j) {
      f[7 + j] = data->qpos[qadr[j]];
      f[36 + j] = data->qvel[vadr[j]];
    }
    for (int j = 0; j < 4; ++j) {
      const int body = feet[j % 2];
      const auto & point = foot_points[j / 2];
      f[59 + j] = data->xpos[3 * body + 2] +
        data->xmat[9 * body + 6] * point[0] +
        data->xmat[9 * body + 7] * point[1] +
        data->xmat[9 * body + 8] * point[2];
    }
    require(std::all_of(f.begin(), f.end(), [](float x) {return std::isfinite(x);}),
      "nonfinite motion reference");
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
  inputs_["obs"].resize(path_tracking_ ? 134 : 132);
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
  if (path_tracking_) {
    const Eigen::Vector2f robot_xy = reference_origin_ + (alignment_ *
      Eigen::Vector3f(s.xy.x() - origin_.x(), s.xy.y() - origin_.y(), 0)).head<2>();
    const Eigen::Vector2f error = Eigen::Rotation2Df(-yaw(robot)) * (Eigen::Vector2f(f[0],
      f[1]) - robot_xy);
    obs[127] = error.x(); obs[128] = error.y();
  }
  const size_t feet_offset = path_tracking_ ? 129 : 127;
  for (int j = 0; j < 4; ++j) {
    obs[feet_offset + j] = f[59 + j];
  }
  obs[feet_offset + 4] = f[2];
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
