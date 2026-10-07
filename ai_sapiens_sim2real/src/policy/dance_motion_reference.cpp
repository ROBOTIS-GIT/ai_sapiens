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

#include "ai_sapiens_sim2real/policy/dance_motion_reference.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include <openssl/evp.h>

#include "ai_sapiens_sim2real/policy/motion_reference.hpp"

#include <Eigen/LU>

namespace ai_sapiens_sim2real
{
namespace
{
using V3 = Eigen::Vector3f;
using Q = Eigen::Quaternionf;
using Joints = std::array<float, 23>;

void require(bool ok, const std::string & message)
{
  if (!ok) {
    throw std::runtime_error("Dance reference: " + message);
  }
}

// Avoid constructing a heap-backed error string on successful control ticks.
void require(bool ok, const char * message)
{
  if (!ok) {
    throw std::runtime_error(std::string("Dance reference: ") + message);
  }
}

float wrap(float x) {return std::atan2(std::sin(x), std::cos(x));}
float clamp(float x, float low, float high) {return std::clamp(x, low, high);}
Q yaw_q(float yaw) {return Q(Eigen::AngleAxisf(yaw, V3::UnitZ()));}
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> rotation_vector(Eigen::Quaternion<Scalar> q)
{
  if (q.w() < 0) {q.coeffs() *= -1;}
  const Scalar length = q.vec().norm();
  const Scalar angle = 2 * std::atan2(length, std::max(q.w(), Scalar(1e-8)));
  return q.vec() * (length > Scalar(1e-6) ? angle / length : Scalar(2));
}

Q nlerp(const Q & low, Q high, float fraction)
{
  if (low.coeffs().dot(high.coeffs()) < 0) {high.coeffs() *= -1;}
  Q result;
  result.coeffs() = low.coeffs() + fraction * (high.coeffs() - low.coeffs());
  return result.normalized();
}

struct Frame
{
  Joints joints{}, velocity{};
  V3 root;
  Q root_q;
  std::array<V3, 2> feet;
  std::array<Q, 2> foot_q;
  std::array<float, 2> air{}, unload{}, accent{}, clearance{};
  std::array<int, 2> ends{};
  std::array<std::array<int, 9>, 2> candidates{};
  int next_start;
  float heading;
};

struct Gesture
{
  std::array<V3, 65> profile;
  float duration;
  int source;
  int side;
};

struct Leg
{
  std::array<int, 6> indices;
  std::array<V3, 6> positions, anchors, axes;
  std::array<Q, 6> quaternions;
  std::array<float, 6> q0, low, high;
};

template<typename Scalar>
struct ChainT
{
  std::array<Eigen::Matrix<Scalar, 3, 1>, 6> position, anchor, axis;
  std::array<Eigen::Quaternion<Scalar>, 6> quaternion;
};
using Chain = ChainT<float>;

template<typename Scalar>
ChainT<Scalar> forward(
  const Leg & leg, const std::array<Scalar, 23> & joints,
  Eigen::Matrix<Scalar, 3, 1> p, Eigen::Quaternion<Scalar> q)
{
  using Quat = Eigen::Quaternion<Scalar>;
  ChainT<Scalar> chain;
  for (int i = 0; i < 6; ++i) {
    p += q * leg.positions[i].cast<Scalar>();
    q = q * leg.quaternions[i].cast<Scalar>();
    chain.anchor[i] = p + q * leg.anchors[i].cast<Scalar>();
    chain.axis[i] = q * leg.axes[i].cast<Scalar>();
    q = q * Quat(Eigen::AngleAxis<Scalar>(joints[leg.indices[i]] - Scalar(leg.q0[i]),
      leg.axes[i].cast<Scalar>()));
    p = chain.anchor[i] - q * leg.anchors[i].cast<Scalar>();
    chain.position[i] = p;
    chain.quaternion[i] = q;
  }
  return chain;
}

bool solve_precise(
  const Leg & leg, const Joints & nominal, Joints & output,
  const V3 & root, const Q & root_q, const V3 & goal, const Q & goal_q,
  const DanceReferenceConfig & cfg)
{
  std::array<double, 23> joints;
  std::copy(output.begin(), output.end(), joints.begin());
  for (int j = 0; j < 6; ++j) {
    const int index = leg.indices[j];
    joints[index] = std::clamp(std::clamp(joints[index], double(leg.low[j]), double(leg.high[j])),
      double(nominal[index]) - cfg.ik_joint_correction_limit,
      double(nominal[index]) + cfg.ik_joint_correction_limit);
  }
  const Eigen::Vector3d root_d = root.cast<double>(), goal_d = goal.cast<double>();
  const Eigen::Quaterniond root_qd = root_q.cast<double>(), goal_qd = goal_q.cast<double>();
  double step_scale = 1;
  for (int iteration = 0; iteration < cfg.ik_iterations; ++iteration) {
    const auto chain = forward(leg, joints, root_d, root_qd);
    Eigen::Matrix<double, 6, 6> jacobian;
    for (int j = 0; j < 6; ++j) {
      jacobian.block<3, 1>(0, j) = chain.axis[j].cross(chain.position[5] - chain.anchor[j]);
      jacobian.block<3, 1>(3, j) = .15 * chain.axis[j];
    }
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = goal_d - chain.position[5];
    error.tail<3>() = .15 * rotation_vector(goal_qd * chain.quaternion[5].conjugate());
    const Eigen::Matrix<double, 6, 6> normal = jacobian * jacobian.transpose() +
      double(cfg.ik_damping) * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6,
      1> update = jacobian.transpose() * normal.partialPivLu().solve(error);
    if (!update.allFinite()) {return false;}
    auto candidate = joints;
    for (int j = 0; j < 6; ++j) {
      const int index = leg.indices[j];
      const double value = std::clamp(joints[index] + std::clamp(update[j], -.12, .12) * step_scale,
        double(nominal[index]) - cfg.ik_joint_correction_limit,
        double(nominal[index]) + cfg.ik_joint_correction_limit);
      candidate[index] = std::clamp(value, double(leg.low[j]), double(leg.high[j]));
    }
    const auto after = forward(leg, candidate, root_d, root_qd);
    const double next_error = (goal_d - after.position[5]).squaredNorm() +
      (.15 * rotation_vector(goal_qd * after.quaternion[5].conjugate())).squaredNorm();
    if (next_error < error.squaredNorm() * (1 - 1e-4) - 1e-12) {
      joints = candidate;
      step_scale = 1;
    } else {
      step_scale *= .5;
    }
  }
  for (const int index : leg.indices) {
    output[index] = static_cast<float>(joints[index]);
  }
  return true;
}

struct FootState
{
  V3 translation{V3::Zero()}, output_translation{V3::Zero()}, goal_translation{V3::Zero()};
  float yaw{0}, output_yaw{0}, goal_yaw{0};
  bool was_swing{false}, source_was_swing{false};
  float age{-1}, duration{1}, stance_age{0};
  int gesture{0};
  float source_start{0}, source_end{0}, source_duration{1}, remaining{0}, dance_phase{0};
};
}  // namespace

struct DanceMotionReference::Impl
{
  DanceReferenceConfig cfg;
  float dt;
  std::vector<Frame> frames;
  std::vector<Gesture> gestures;
  std::array<Leg, 2> legs;
  std::array<std::vector<V3>, 2> points;
  std::array<FootState, 2> feet;
  DanceReferenceOutput out;
  Joints correction{};
  V3 previous_offset{V3::Zero()};
  float previous_yaw{0};
  int previous_frame{-1};
  bool just_reset{true};

  Impl(
    const DanceReferenceConfig & config, const std::vector<std::string> & joints,
    double step_dt, const MotionReference & source)
  : cfg(config), dt(static_cast<float>(step_dt))
  {
    require(joints.size() == 23, "expected 23 K1 policy joints");
    load_kinematics(joints);
    const int count = source.frame_count();
    require(count >= 4, "motion requires at least four frames");
    std::array<int, 23> joint_map{};
    for (size_t j = 0; j < joints.size(); ++j) {
      const auto & names = source.joint_order();
      const auto found = std::find(names.begin(), names.end(), joints[j]);
      require(found != names.end(), "CSV is missing joint: " + joints[j]);
      joint_map[j] = static_cast<int>(found - names.begin());
    }
    frames.resize(count);
    for (int i = 0; i < count; ++i) {
      const auto sample = source.frame(i);
      auto & frame = frames[i];
      const auto & positions = sample.joint_pos;
      const auto & velocities = sample.joint_vel;
      for (size_t j = 0; j < joints.size(); ++j) {
        frame.joints[j] = positions[joint_map[j]];
        frame.velocity[j] = velocities[joint_map[j]];
      }
      frame.root = sample.root_position;
      frame.root_q = sample.root_quaternion;
      frame.heading = PlanarMotionSteering::heading(frame.root_q);
      for (int side = 0; side < 2; ++side) {
        const auto chain = forward(legs[side], frame.joints, frame.root, frame.root_q);
        frame.feet[side] = chain.position.back();
        frame.foot_q[side] = chain.quaternion.back();
        frame.clearance[side] = std::max(sole_height(frame.feet[side], frame.foot_q[side], side),
            0.0f);
        // CSV deployment estimates soft contact from the geometric sole height.
        // The 8--25 mm transition matches the source label's height-to-air band,
        // but uses the actual CSV pose, not the training label's smoothed height.
        const float lift = clamp((frame.clearance[side] - .008f) / .017f, 0, 1);
        frame.air[side] = lift * lift * (3 - 2 * lift);
        frame.ends[side] = i;
      }
    }
    if (!cfg.training_reference_file.empty()) {load_training_reference();}
    extract_gestures(step_dt);
  }

  void load_training_reference()
  {
    std::ifstream input(cfg.training_reference_file, std::ios::binary);
    require(static_cast<bool>(input),
        "cannot open training reference: " + cfg.training_reference_file);
    // Validate manually copied assets too, using the bytes actually consumed below.
    std::unique_ptr<EVP_MD_CTX, decltype(& EVP_MD_CTX_free)> digest(
      EVP_MD_CTX_new(), &EVP_MD_CTX_free);
    require(digest && EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) == 1,
      "cannot initialize training reference SHA256");
    const auto read = [&](void * destination, size_t bytes) {
        input.read(static_cast<char *>(destination), static_cast<std::streamsize>(bytes));
        require(static_cast<bool>(input), "truncated training reference");
        require(EVP_DigestUpdate(digest.get(), destination, bytes) == 1,
          "cannot update training reference SHA256");
      };
    const uint32_t endian = 1;
    require(*reinterpret_cast<const char *>(&endian) == 1, "reference requires little-endian host");
    char magic[8];
    uint32_t count, point_count;
    float reference_dt;
    read(magic, sizeof(magic));
    read(&count, sizeof(count));
    read(&point_count, sizeof(point_count));
    read(&reference_dt, sizeof(reference_dt));
    require(std::memcmp(magic, "K1DREF1\0", 8) == 0, "unsupported training reference format");
    require(count == frames.size() && point_count > 0 && point_count <= 100000 &&
      std::isfinite(reference_dt) && std::abs(reference_dt - dt) < 1e-7f,
      "training reference dimensions/timestep differ from motion");
    for (auto & frame : frames) {
      std::array<float, 71> row;
      read(row.data(), sizeof(row));
      require(std::all_of(row.begin(), row.end(), [](float x) {return std::isfinite(x);}),
        "non-finite training reference");
      const auto vector = [&](int start) {return V3(row[start], row[start + 1], row[start + 2]);};
      const auto quaternion = [&](int start) {
          Q q(row[start], row[start + 1], row[start + 2], row[start + 3]);
          require(std::abs(q.squaredNorm() - 1) < 1e-4f, "invalid reference quaternion");
          return q;
        };
      require((frame.root - vector(0)).norm() < 1e-4f &&
        rotation_vector(frame.root_q * quaternion(3).conjugate()).norm() < 1e-4f,
        "training reference root differs from CSV");
      for (int j = 0; j < 23; ++j) {
        require(std::abs(frame.joints[j] - row[7 + j]) < 1e-5f &&
          std::abs(frame.velocity[j] - row[30 + j]) < 1e-3f,
          "training reference joints differ from CSV");
        frame.joints[j] = row[7 + j];
        frame.velocity[j] = row[30 + j];
      }
      frame.root = vector(0);
      frame.root_q = quaternion(3);
      frame.heading = PlanarMotionSteering::heading(frame.root_q);
      for (int side = 0; side < 2; ++side) {
        frame.feet[side] = vector(53 + side * 3);
        frame.foot_q[side] = quaternion(59 + side * 4);
        frame.air[side] = row[67 + side];
        frame.clearance[side] = row[69 + side];
        require(frame.air[side] >= 0 && frame.air[side] <= 1 && frame.clearance[side] >= 0,
          "invalid training contact/clearance");
      }
    }
    for (auto & foot : points) {
      foot.resize(point_count);
      for (auto & point : foot) {
        std::array<float, 3> xyz;
        read(xyz.data(), sizeof(xyz));
        point = V3(xyz[0], xyz[1], xyz[2]);
        require(point.allFinite(), "non-finite sole point");
      }
    }
    require(input.peek() == std::char_traits<char>::eof(), "trailing training reference data");
    std::array<unsigned char, EVP_MAX_MD_SIZE> hash{};
    unsigned int hash_size = 0;
    require(EVP_DigestFinal_ex(digest.get(), hash.data(), &hash_size) == 1 && hash_size == 32,
      "cannot finalize training reference SHA256");
    constexpr char hex[] = "0123456789abcdef";
    std::string actual;
    actual.reserve(64);
    for (unsigned int i = 0; i < hash_size; ++i) {
      actual += hex[hash[i] >> 4];
      actual += hex[hash[i] & 15];
    }
    require(actual == cfg.training_reference_sha256,
      "training reference SHA256 differs from sim2real.yaml: " + cfg.training_reference_file);
  }

  void load_kinematics(const std::vector<std::string> & joints)
  {
    const auto xml = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("ai_sapiens_description")) /
      "mujoco/k1/k1.xml";
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(& mj_deleteModel)> model(
      mj_loadXML(xml.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    require(static_cast<bool>(model), "cannot load K1 kinematics: " + std::string(error));
    const char * sides[] = {"left", "right"};
    const char * parts[] = {"hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch",
      "ankle_roll"};
    for (int side = 0; side < 2; ++side) {
      int parent = mj_name2id(model.get(), mjOBJ_BODY, "pelvis");
      auto & leg = legs[side];
      for (int j = 0; j < 6; ++j) {
        const std::string stem = std::string(sides[side]) + "_" + parts[j];
        const int body = mj_name2id(model.get(), mjOBJ_BODY, (stem + "_link").c_str());
        require(body >= 0 && model->body_parentid[body] == parent && model->body_jntnum[body] == 1,
          "expected a K1 six-hinge leg chain");
        const int joint = model->body_jntadr[body];
        const auto found = std::find(joints.begin(), joints.end(), stem + "_joint");
        require(found != joints.end() && model->jnt_type[joint] == mjJNT_HINGE &&
          model->jnt_limited[joint] &&
          std::string(mj_id2name(model.get(), mjOBJ_JOINT, joint)) == *found,
          "invalid K1 leg joint");
        leg.indices[j] = static_cast<int>(found - joints.begin());
        const auto vector = [](const mjtNum * value) {
            return V3(value[0], value[1], value[2]);
          };
        leg.positions[j] = vector(model->body_pos + 3 * body);
        const auto * quat = model->body_quat + 4 * body;
        leg.quaternions[j] = Q(quat[0], quat[1], quat[2], quat[3]);
        leg.anchors[j] = vector(model->jnt_pos + 3 * joint);
        leg.axes[j] = vector(model->jnt_axis + 3 * joint);
        leg.q0[j] = model->qpos0[model->jnt_qposadr[joint]];
        leg.low[j] = model->jnt_range[2 * joint];
        leg.high[j] = model->jnt_range[2 * joint + 1];
        parent = body;
      }
      // MuJoCo has already applied STL scale and mesh principal-axis transforms.
      // Transform the visual mesh vertices into the ankle frame once at startup.
      // Keeping the complete mesh avoids an external sampled foot-envelope file.
      for (int geom = 0; geom < model->ngeom; ++geom) {
        if (model->geom_bodyid[geom] != parent || model->geom_type[geom] != mjGEOM_MESH) {
          continue;
        }
        const int mesh = model->geom_dataid[geom];
        const auto * p = model->geom_pos + 3 * geom;
        const auto * q = model->geom_quat + 4 * geom;
        const Eigen::Vector3d position(p[0], p[1], p[2]);
        const Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
        for (int vertex = 0; vertex < model->mesh_vertnum[mesh]; ++vertex) {
          const auto * v = model->mesh_vert + 3 * (model->mesh_vertadr[mesh] + vertex);
          const V3 point = (position + rotation * Eigen::Vector3d(v[0], v[1], v[2])).cast<float>();
          require(point.allFinite(), "non-finite foot geometry");
          points[side].push_back(point);
        }
      }
      require(!points[side].empty(), "K1 ankle visual mesh is missing");
    }
  }

  // Direct port of humanoid_motion/mdp/dance_motion.py. Only original motion is
  // inspected here; commanded velocity and added placements are evaluated in step().
  void extract_gestures(double step_dt)
  {
    const int count = static_cast<int>(frames.size());
    std::vector<std::tuple<int, int, int>> runs;
    std::vector<int> starts;
    for (int side = 0; side < 2; ++side) {
      for (int start = 0; start < count; ) {
        if (frames[start].air[side] <= .5f) {++start; continue;}
        int end = start + 1;
        while (end < count && frames[end].air[side] > .5f) {++end;}
        for (int i = start; i < end; ++i) {frames[i].ends[side] = std::min(end, count - 1);}
        runs.emplace_back(start, end, side);
        starts.push_back(start);
        start = end;
      }
    }
    std::sort(runs.begin(), runs.end());
    std::sort(starts.begin(), starts.end());
    for (int i = 0; i < count; ++i) {
      auto & frame = frames[i];
      const auto next = std::upper_bound(starts.begin(), starts.end(), i);
      frame.next_start = next == starts.end() ? count - 1 : *next;
      const Eigen::Vector2f lateral = (frame.feet[0] - frame.feet[1]).head<2>();
      const float load = clamp((frame.root - frame.feet[1]).head<2>().dot(lateral) /
        std::max(lateral.squaredNorm(), .01f), 0, 1);
      frame.unload = {{1 - load, load}};
    }
    for (int i = 0; i < count; ++i) {
      const int low = std::max(i - 1, 0), high = std::min(i + 1, count - 1);
      for (int side = 0; side < 2; ++side) {
        const float rate = (frames[high].unload[side] - frames[low].unload[side]) /
          ((high - low) * dt);
        frames[i].accent[side] = clamp(std::max(rate, 0.0f) * .2f +
          frames[i].clearance[side] / .04f, 0, 1);
      }
    }
    for (const auto & [start, end, side] : runs) {
      if (start == 0 || end >= count || end - start < 8) {continue;}
      float maximum = 0;
      bool isolated = true;
      for (int i = start; i < end; ++i) {
        maximum = std::max(maximum, frames[i].clearance[side]);
        isolated = isolated && frames[i].air[1 - side] <= .5f;
      }
      if (!isolated || maximum < .015f || maximum > .12f) {continue;}
      const int length = end - start + 2;
      std::vector<Eigen::Vector3d> profile(length);
      for (int i = 0; i < length; ++i) {
        const auto & frame = frames[start - 1 + i];
        const Eigen::Vector2f local = (frame.feet[side] - frame.root).head<2>();
        const float c = std::cos(frame.heading), s = std::sin(frame.heading);
        profile[i] = Eigen::Vector3d(c * local.x() + s * local.y(),
          -s * local.x() + c * local.y(), frame.clearance[side]);
      }
      const auto first = profile.front(), last = profile.back();
      double maximum_arc = 1e-8, maximum_height = 1e-8;
      for (int i = 0; i < length; ++i) {
        const double phase = static_cast<double>(i) / (length - 1);
        profile[i] -= (1 - phase) * first + phase * last;
        profile[i].z() = std::max(profile[i].z(), 0.0);
        maximum_arc = std::max(maximum_arc, profile[i].head<2>().norm());
        maximum_height = std::max(maximum_height, profile[i].z());
      }
      for (auto & point : profile) {
        point.head<2>() *= std::min(1.0, .03 / maximum_arc);
        point.z() *= std::min(1.0, cfg.max_step_lift / maximum_height);
      }
      Gesture gesture;
      gesture.duration = static_cast<float>((end - start + 1) * step_dt);
      gesture.source = start;
      gesture.side = side;
      for (int sample = 0; sample < 65; ++sample) {
        const double coordinate = sample * (length - 1) / 64.0;
        const int low = static_cast<int>(coordinate),
          high = static_cast<int>(std::ceil(coordinate));
        gesture.profile[sample] = (profile[low] + (coordinate - low) *
          (profile[high] - profile[low])).cast<float>();
      }
      gestures.push_back(gesture);
    }
    for (int side = 0; side < 2; ++side) {
      std::vector<int> pool;
      for (size_t i = 0; i < gestures.size(); ++i) {
        if (gestures[i].side == side) {pool.push_back(static_cast<int>(i));}
      }
      require(!pool.empty(), "motion needs isolated foot gestures of at least eight frames");
      const int shortest = *std::min_element(pool.begin(), pool.end(), [&](int a, int b) {
            return gestures[a].duration < gestures[b].duration;
        });
      for (int frame = 0; frame < count; ++frame) {
        auto nearest = pool;
        std::stable_sort(nearest.begin(), nearest.end(), [&](int a, int b) {
            return std::abs(gestures[a].source - frame) < std::abs(gestures[b].source - frame);
          });
        const size_t available = std::min<size_t>(nearest.size(), 8);
        for (size_t j = 0; j < 8; ++j) {frames[frame].candidates[side][j] = nearest[j % available];}
        frames[frame].candidates[side][8] = shortest;
      }
    }
  }

  float sole_height(const V3 & position, const Q & quaternion, int side) const
  {
    const float x = quaternion.x(), y = quaternion.y(), z = quaternion.z(), w = quaternion.w();
    const V3 row(2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
    float height = std::numeric_limits<float>::infinity();
    for (const auto & point : points[side]) {height = std::min(height, row.dot(point));}
    return position.z() + height;
  }

  void reset(int frame)
  {
    require(frame >= 0 && frame < static_cast<int>(frames.size()), "reset frame outside clip");
    const auto & f = frames[frame];
    feet = {};
    out = {};
    out.joint_pos = f.joints;
    out.joint_vel = f.velocity;
    out.root_shift.setZero();
    out.foot_position = f.feet;
    out.foot_orientation = f.foot_q;
    out.air = f.air;
    correction.fill(0);
    previous_frame = frame;
    previous_offset.setZero();
    previous_yaw = 0;
    just_reset = true;
    for (int side = 0; side < 2; ++side) {
      feet[side].was_swing = f.air[side] > .5f;
      feet[side].source_start = feet[side].source_end = static_cast<float>(frame);
    }
  }

  void step(
    int frame, const PlanarMotionSteering & steering, const V3 & velocity, bool command_active)
  {
    require(previous_frame >= 0 && frame == previous_frame + 1 &&
      frame < static_cast<int>(frames.size()), "step must advance one frame; reset on entry/seek");
    const auto & f = frames[frame];
    const V3 offset(steering.offset.x(), steering.offset.y(), 0);
    const auto previous_shift = out.root_shift;
    const auto previous_valid = out.valid;
    // Match the training release rebase, including the final velocity cutoff tick.
    const V3 delta = f.root - frames[previous_frame].root;
    const float expected_yaw = previous_yaw + steering.velocity.z() * dt;
    const V3 expected_offset = previous_offset +
      yaw_q(previous_yaw + .5f * steering.velocity.z() * dt) * delta - delta + velocity * dt;
    const float rebase_yaw = wrap(steering.yaw - expected_yaw);
    const V3 rebase_translation = offset - expected_offset;
    if (!just_reset &&
      (rebase_translation.cwiseAbs().maxCoeff() > 1e-6f || std::abs(rebase_yaw) > 1e-6f))
    {
      const V3 pivot = f.root + expected_offset;
      const auto rotation = yaw_q(rebase_yaw);
      for (auto & foot : feet) {
        foot.translation = rotation * (foot.translation - pivot) + pivot + rebase_translation;
        foot.output_translation = rotation * (foot.output_translation - pivot) + pivot +
          rebase_translation;
        foot.goal_translation = rotation * (foot.goal_translation - pivot) + pivot +
          rebase_translation;
        foot.yaw += rebase_yaw;
        foot.output_yaw += rebase_yaw;
        foot.goal_yaw += rebase_yaw;
      }
    }

    // V3 separates operator intent from the decaying velocity filter. Keep actual
    // velocity for release rebasing above and root crouch below; gate only planning.
    const bool planning_active = !cfg.stop_new_steps_on_release || command_active;
    const V3 planning_velocity = planning_active ? velocity : V3::Zero();
    const float planning_yaw_rate = planning_active ? steering.velocity.z() : 0.0f;
    const float speed = planning_velocity.head<2>().norm() + .15f * std::abs(planning_yaw_rate);
    std::array<V3, 2> raw_p, current;
    std::array<Q, 2> raw_q;
    std::array<float, 2> raw_air{}, debt{}, lag{}, turn_lag{};
    std::array<bool, 2> natural{}, active{}, needs_step{}, eligible{};
    std::array<int, 2> gesture{};
    std::array<float, 2> duration{};
    // sample_dance: shorten a natural swing only when opposite support reach requires it.
    for (int side = 0; side < 2; ++side) {
      const auto & foot = feet[side];
      current[side] = yaw_q(foot.yaw) * f.feet[side] + foot.translation;
      const V3 mapped = f.root + offset + yaw_q(steering.yaw) * (f.feet[side] - f.root);
      debt[side] = (mapped - current[side]).head<2>().norm();
    }
    for (int side = 0; side < 2; ++side) {
      auto & foot = feet[side];
      const bool swing = f.air[side] > .5f;
      if (swing && !foot.source_was_swing) {
        const float original = static_cast<float>(std::max(f.ends[side] - frame, 1));
        const float available = std::max(cfg.support_reach - debt[1 - side], .02f);
        const float reach_frames = std::floor((available / std::max(speed, 1e-6f) -
            cfg.support_transfer_duration) / dt);
        foot.source_start = static_cast<float>(frame);
        foot.source_end = static_cast<float>(f.ends[side]);
        foot.source_duration = std::min(original, std::max(reach_frames, 4.0f));
      }
      const float elapsed = frame - foot.source_start;
      const float phase = clamp(elapsed / foot.source_duration, 0, 1);
      const float sample = swing ? foot.source_start + phase *
        (foot.source_end - foot.source_start) : frame;
      const int low = static_cast<int>(sample), high = static_cast<int>(std::ceil(sample));
      require(low >= 0 && high < static_cast<int>(frames.size()), "sample outside clip");
      const float fraction = sample - low;
      const auto & a = frames[low];
      const auto & b = frames[high];
      raw_p[side] = a.feet[side] + fraction * (b.feet[side] - a.feet[side]);
      raw_q[side] = nlerp(a.foot_q[side], b.foot_q[side], fraction);
      const V3 sampled_root = a.root + fraction * (b.root - a.root);
      const Q sampled_q = nlerp(a.root_q, b.root_q, fraction);
      const auto rotation = yaw_q(PlanarMotionSteering::heading(f.root_q * sampled_q.conjugate()));
      const V3 local = rotation * (raw_p[side] - sampled_root);
      raw_p[side].head<2>() = f.root.head<2>() + local.head<2>();
      raw_q[side] = rotation * raw_q[side];
      raw_air[side] = a.air[side] + fraction * (b.air[side] - a.air[side]);
      const bool compressed = swing && foot.source_duration < foot.source_end - foot.source_start;
      if (compressed) {
        raw_air[side] = std::max(raw_air[side],
          clamp((sole_height(raw_p[side], raw_q[side], side) - .005f) / .02f, 0, 1));
      }
      foot.remaining = swing ? std::max(foot.source_duration - elapsed, 0.0f) * dt : 0;
      foot.dance_phase = clamp(elapsed / std::max(foot.source_duration - 1, 1.0f), 0, 1);
      foot.source_was_swing = swing;
    }

    // DanceFootstepPlanner.step: source-motion gestures selected by reach debt.
    for (int side = 0; side < 2; ++side) {
      auto & foot = feet[side];
      natural[side] = raw_air[side] > .5f;
      if (foot.age >= 0) {foot.age += dt;}
      if (foot.age >= foot.duration - 1e-6f) {foot.age = -1;}
      foot.stance_age = foot.was_swing ? 0 : foot.stance_age + dt;
      active[side] = natural[side] || foot.age >= 0;
      if (foot.was_swing && !active[side]) {
        foot.translation = foot.output_translation;
        foot.yaw = foot.output_yaw;
      }
      current[side] = yaw_q(foot.yaw) * raw_p[side] + foot.translation;
      const V3 mapped = f.root + offset + yaw_q(steering.yaw) * (raw_p[side] - f.root);
      lag[side] = (mapped - current[side]).head<2>().norm();
      turn_lag[side] = std::abs(wrap(steering.yaw - foot.yaw));
      debt[side] = lag[side] + .15f * turn_lag[side];
      needs_step[side] = lag[side] > .02f || turn_lag[side] > .04f;
      float best = std::numeric_limits<float>::infinity();
      bool fits = false;
      gesture[side] = f.candidates[side][0];
      const float gap = (f.next_start - frame) * dt;
      for (const int index : f.candidates[side]) {
        const auto & g = gestures[index];
        if (g.duration + cfg.support_transfer_duration > gap) {continue;}
        fits = true;
        const float predicted = debt[side] + speed * (g.duration + cfg.support_transfer_duration);
        const float cost = std::abs(g.source - frame) * dt * .001f +
          std::max(predicted - cfg.support_reach, 0.0f) * 20;
        if (cost < best) {best = cost; gesture[side] = index;}
      }
      duration[side] = gestures[gesture[side]].duration;
      const float predicted = debt[side] + speed * (duration[side] + cfg.support_transfer_duration);
      const bool opportunity = (predicted > .65f * cfg.support_reach && f.accent[side] > .15f) ||
        predicted >= cfg.support_reach ||
        (!cfg.stop_new_steps_on_release && speed < .015f && needs_step[side]);
      eligible[side] = planning_active && opportunity && needs_step[side] && fits &&
        foot.stance_age >= cfg.min_stance_duration - 1e-6f;
    }
    const bool can_add = !active[0] && !active[1] &&
      std::min(feet[0].stance_age, feet[1].stance_age) >= cfg.support_transfer_duration - 1e-6f;
    int selected = -1;
    float best_score = -std::numeric_limits<float>::infinity();
    for (int side = 0; side < 2; ++side) {
      if (can_add && eligible[side]) {
        const float score = debt[side] / cfg.support_reach + .3f * f.unload[side] + .15f *
          f.accent[side];
        if (score > best_score) {selected = side; best_score = score;}
      }
    }
    float support_shift = 0;
    for (int side = 0; side < 2; ++side) {
      auto & foot = feet[side];
      if (selected == side) {
        foot.age = 0;
        foot.duration = duration[side];
        foot.gesture = gesture[side];
        ++out.added_steps;
      }
      const bool synthetic = foot.age >= 0;
      const bool swing = natural[side] || synthetic;
      const float remaining = synthetic ? foot.duration - foot.age : foot.remaining;
      const float support_time = std::min(cfg.support_reach / std::max(speed, .015f), 1.0f);
      const float lead = remaining + .5f * support_time;
      const float goal_angle = clamp(wrap(steering.yaw + planning_yaw_rate * lead - foot.yaw),
        -cfg.max_step_yaw, cfg.max_step_yaw);
      const float new_yaw = foot.yaw + goal_angle;
      const V3 future = f.root + offset + planning_velocity * lead +
        yaw_q(new_yaw) * (raw_p[side] - f.root);
      V3 displacement = future - current[side];
      displacement.z() = 0;
      const float bound = std::min(remaining, cfg.max_step_correction);
      displacement *= bound / std::max(std::max(displacement.norm(), bound), 1e-8f);
      if (swing && !foot.was_swing) {
        foot.goal_translation = current[side] + displacement - yaw_q(new_yaw) * raw_p[side];
        foot.goal_yaw = new_yaw;
      }
      const float phase = clamp(synthetic ? foot.age / std::max(foot.duration - dt,
          dt) : foot.dance_phase, 0, 1);
      const float blend = swing ? phase * phase * phase * (10 - 15 * phase + 6 * phase * phase) : 0;
      foot.output_yaw = foot.yaw + blend * (foot.goal_yaw - foot.yaw);
      const V3 endpoint = yaw_q(foot.goal_yaw) * raw_p[side] + foot.goal_translation;
      V3 target = current[side] + blend * (endpoint - current[side]);
      const auto & g = gestures[foot.gesture];
      const float coordinate = phase * 64;
      const int low = static_cast<int>(coordinate), high = static_cast<int>(std::ceil(coordinate));
      V3 profile = synthetic ? V3(g.profile[low] + (coordinate - low) *
          (g.profile[high] - g.profile[low])) : V3::Zero();
      profile = yaw_q(f.heading + steering.yaw) * profile;
      target += profile;
      foot.output_translation = target - yaw_q(foot.output_yaw) * raw_p[side];
      foot.output_translation.z() -= profile.z();
      out.air[side] = std::max(raw_air[side], clamp(profile.z() / .015f, 0, 1));
      float amplitude = .015f;
      for (const auto & point : g.profile) {amplitude = std::max(amplitude, point.z());}
      support_shift += profile.z() / amplitude * (side == 0 ? -1 : 1);
      foot.was_swing = swing;
      out.foot_position[side] = target;
      out.foot_orientation[side] = yaw_q(foot.output_yaw) * raw_q[side];
    }

    const Q root_q = yaw_q(steering.yaw) * f.root_q;
    out.root_shift = yaw_q(PlanarMotionSteering::heading(root_q)) * V3(0,
        cfg.weight_shift * support_shift, 0);
    const float moving = std::min(std::max(steering.velocity.head<2>().norm() / .15f,
      std::abs(steering.velocity.z()) / .2f), 1.0f);
    out.root_shift.z() = previous_shift.z() + (1 - std::exp(-dt / .25f)) *
      (-cfg.moving_crouch * moving - previous_shift.z());
    const V3 root = f.root + offset + out.root_shift;
    std::array<Chain, 2> nominal;
    std::array<V3, 2> base_p, goal_p;
    std::array<Q, 2> base_q, goal_q;
    bool unchanged = true;
    for (int side = 0; side < 2; ++side) {
      nominal[side] = forward(legs[side], f.joints, root, root_q);
      base_p[side] = f.root + offset + yaw_q(steering.yaw) * (f.feet[side] - f.root) +
        out.root_shift;
      base_q[side] = yaw_q(steering.yaw) * f.foot_q[side];
      goal_p[side] = nominal[side].position[5] + out.foot_position[side] - base_p[side];
      goal_q[side] = out.foot_orientation[side] * base_q[side].conjugate() *
        nominal[side].quaternion[5];
      unchanged = unchanged &&
        (out.foot_position[side] - base_p[side]).cwiseAbs().maxCoeff() < 1e-6f &&
        rotation_vector(out.foot_orientation[side] *
          base_q[side].conjugate()).cwiseAbs().maxCoeff() < 1e-6f;
    }
    out.joint_pos = f.joints;
    out.joint_vel = f.velocity;
    std::array<bool, 2> finite{{true, true}};
    if (!unchanged) {
      for (int i = 0; i < 23; ++i) {out.joint_pos[i] += correction[i];}
      for (int side = 0; side < 2; ++side) {
        const auto & leg = legs[side];
        if (cfg.ik_monotonic) {
          finite[side] = solve_precise(leg, f.joints, out.joint_pos, root, root_q,
              goal_p[side], goal_q[side], cfg);
        } else {
          for (int iteration = 0; iteration < cfg.ik_iterations; ++iteration) {
            const auto chain = forward(leg, out.joint_pos, root, root_q);
            Eigen::Matrix<float, 6, 6> jacobian;
            for (int j = 0; j < 6; ++j) {
              jacobian.block<3, 1>(0, j) = chain.axis[j].cross(chain.position[5] - chain.anchor[j]);
              jacobian.block<3, 1>(3, j) = .15f * chain.axis[j];
            }
            Eigen::Matrix<float, 6, 1> error;
            error.head<3>() = goal_p[side] - chain.position[5];
            error.tail<3>() = .15f *
              rotation_vector(goal_q[side] * chain.quaternion[5].conjugate());
            const Eigen::Matrix<float, 6, 6> normal = jacobian * jacobian.transpose() +
              cfg.ik_damping * Eigen::Matrix<float, 6, 6>::Identity();
            const Eigen::Matrix<float, 6,
              1> update = jacobian.transpose() * normal.partialPivLu().solve(error);
            if (!update.allFinite()) {finite[side] = false; break;}
            for (int j = 0; j < 6; ++j) {
              const int index = leg.indices[j];
              const float value = clamp(out.joint_pos[index] + clamp(update[j], -.12f, .12f),
              f.joints[index] - cfg.ik_joint_correction_limit,
                f.joints[index] + cfg.ik_joint_correction_limit);
              out.joint_pos[index] = clamp(value, leg.low[j], leg.high[j]);
            }
          }
        }
        for (int j = 0; j < 6; ++j) {
          const int index = leg.indices[j];
          if (!finite[side] || !std::isfinite(out.joint_pos[index])) {
            out.joint_pos[index] = f.joints[index];
          }
          const float change = clamp(out.joint_pos[index] - f.joints[index] - correction[index],
            -cfg.ik_max_correction_rate * dt, cfg.ik_max_correction_rate * dt);
          out.joint_pos[index] = clamp(f.joints[index] + correction[index] + change, leg.low[j],
              leg.high[j]);
        }
      }
    }
    for (int side = 0; side < 2; ++side) {
      const auto solved = forward(legs[side], out.joint_pos, root, root_q);
      const V3 achieved_p = base_p[side] + solved.position[5] - nominal[side].position[5];
      const Q achieved_q = solved.quaternion[5] * nominal[side].quaternion[5].conjugate() *
        base_q[side];
      out.ik_error[side] = (achieved_p - out.foot_position[side]).norm();
      const float orientation_error = rotation_vector(achieved_q *
          out.foot_orientation[side].conjugate()).norm();
      const float height_error = std::abs(sole_height(achieved_p, achieved_q, side) -
        sole_height(out.foot_position[side], out.foot_orientation[side], side));
      out.valid[side] = finite[side] && out.ik_error[side] <= cfg.ik_position_tolerance &&
        orientation_error <= cfg.ik_orientation_tolerance &&
        height_error <= cfg.ik_position_tolerance;
      for (const int index : legs[side].indices) {
        const float next_correction = out.joint_pos[index] - f.joints[index];
        if (out.valid[side] && previous_valid[side] && !just_reset) {
          out.joint_vel[index] += (next_correction - correction[index]) / dt;
        }
        correction[index] = next_correction;
      }
    }
    previous_frame = frame;
    previous_offset = offset;
    previous_yaw = steering.yaw;
    just_reset = false;
  }
};

DanceMotionReference::DanceMotionReference(
  const DanceReferenceConfig & config, const std::vector<std::string> & joints,
  double dt, const MotionReference & motion)
: impl_(std::make_unique<Impl>(config, joints, dt, motion)) {}
DanceMotionReference::~DanceMotionReference() = default;
void DanceMotionReference::reset(int frame) {impl_->reset(frame);}
void DanceMotionReference::step(
  int frame, const PlanarMotionSteering & steering,
  const V3 & velocity, bool command_active)
{
  impl_->step(frame, steering, velocity, command_active);
}
const DanceReferenceOutput & DanceMotionReference::output() const {return impl_->out;}
int DanceMotionReference::frame_count() const {return static_cast<int>(impl_->frames.size());}

}  // namespace ai_sapiens_sim2real
