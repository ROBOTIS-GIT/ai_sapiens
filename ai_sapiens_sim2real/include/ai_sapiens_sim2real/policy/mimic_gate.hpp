#pragma once
// Walk + mimic gate for unified walking and motion policies. No ROS / actuator dependencies.
// Port of feature-newton-amp scripts/mimic_hangang/record_student_mujoco.py (gate_step / ref / observation),
// plus the second-SD-edge abort of mdp/student.py MimicGateCommand. Keep numerically in agreement with
// logs/mimic-student-sapiens-candidate/runtime/parity (Python reference).
//
//   WALK --trigger rising edge--> PENDING (command decays to 0, wait quiet 0.7 s or 2.5 s)
//        --> CLIP_IN (w 0->1 quintic 0.5 s, reference = latched stance)
//        --> BRIDGE_IN (stance -> frame 0, minimum jerk 0.5-1.2 s, w 1) --> CLIP (play)
//        --> BRIDGE_OUT (last frame -> stance) --> CLIP_OUT (w 1->0 on the stance) --> WALK
//   stop (abort_on_release: trigger falling edge; otherwise a second rising edge):
//   PENDING -> WALK (w is still 0); CLIP_IN -> CLIP_OUT; BRIDGE_IN / CLIP -> BRIDGE_OUT from the
//   current reference pose, executed once the reference torso is upright (tilt < 0.5 rad).
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace k1_mimic_gate
{

enum State {WALK, PENDING, CLIP_IN, CLIP, CLIP_OUT, BRIDGE_IN, BRIDGE_OUT};
constexpr int kJoints = 23;
constexpr int kTeacherDim = 124;
using Vec = Eigen::Matrix<float, kJoints, 1>;
// Timers and ramps accumulate dt in float; compare with a small tolerance so a transition happens on the tick the
// reference (double / float32 torch) reaches it, not one tick later (0.02f * 35 = 0.69999...).
constexpr float kTimeEps = 1e-4f;

struct Parameters
{
  float dt{0.02f}, quiet_seconds{0.7f}, pending_timeout_seconds{2.5f}, ramp_seconds{0.5f};
  float bridge_min_seconds{0.5f}, bridge_max_seconds{1.2f}, bridge_joint_speed{1.2f},
  bridge_torso_speed{0.1f};
  float abort_max_reference_tilt{0.5f};
  std::array<float, 3> acceleration_limits{1.f, 1.f, 1.5f};
  float quiet_command{0.1f}, quiet_leg_speed{0.35f}, quiet_ang_speed{0.35f};
  int waist_index{12};  // waist_yaw_joint in the robot joint order
  bool abort_on_release{false};  // release the configured trigger to stop a regular clip
  // 2026-10-12 walk-greet: per-clip flags (empty = every clip standstill-only, the original behaviour)
  std::vector<bool> walk_allowed;   // start without the standstill wait, stick passes through
  std::vector<bool> arms_only_ref;  // reference legs/waist/torso := robot (arm motion only)
  std::vector<bool> fast_start;     // 2026-10-13: leave PENDING on the first quiet tick, CLIP_IN ramp fast_ramp_seconds
  float fast_ramp_seconds{0.f};
  float switch_walk_seconds{0.f};  // 2026-10-13: walk clip started standing -> walk path once the stick moves (leg handoff s)
  // 2026-10-13 interactive clip: hold at hold_frame until the confirm switch changes; low -> low_clip/low_frame
  int inter_clip{-1}, inter_hold_frame{0}, inter_low_clip{-1}, inter_low_frame{0};
  bool confirm_is_trigger{false};  // ignore the trigger's first edge as interactive confirmation
  // 2026-10-15 loop slots: clip indices played in order, forever; stop = finish the current clip
  std::vector<std::vector<int>> loops;
  // A held clip resumes after SA is released or the confirm switch is flipped.
  std::vector<int> hold_frames;  // -1 means this clip has no hold
  float hold_max_seconds{0.f};   // 0 means wait indefinitely
};

struct Clip
{
  std::vector<Vec> q, dq;                 // policy (= robot) joint order
  std::vector<Eigen::Quaternionf> root;   // reference root (pelvis) orientation
  std::vector<float> torso_tilt;          // tilt of root * Rz(waist) [rad]
  int frames() const {return static_cast<int>(q.size());}
};

// Robot CSV: x y z qx qy qz qw + 23 joints (robot order), 50 fps; non-numeric (header) lines skipped.
inline Clip load_clip(const std::string & path, int waist_index, float dt)
{
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("mimic_gate: cannot open clip " + path);}
  Clip c;
  std::string line;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string tok;
    std::vector<float> v;
    bool ok = true;
    while (std::getline(ss, tok, ',')) {
      try {
        size_t used = 0; v.push_back(std::stof(tok, &used));
      } catch (...) {
        ok = false; break;
      }
    }
    if (!ok || v.empty()) {continue;}
    if (v.size() != 30) {
      throw std::runtime_error("mimic_gate: clip row must have 30 columns: " + path);
    }
    Vec q;
    for (int j = 0; j < kJoints; ++j) {
      q[j] = v[7 + j];
    }
    c.q.push_back(q);
    c.root.emplace_back(v[6], v[3], v[4], v[5]);
  }
  if (c.q.size() < 2) {
    throw std::runtime_error("mimic_gate: clip has fewer than 2 frames: " + path);
  }
  const int n = c.frames();
  c.dq.resize(n);
  for (int i = 0; i + 1 < n; ++i) {
    c.dq[i] = (c.q[i + 1] - c.q[i]) / dt;
  }                                                                      // forward difference, last repeated
  c.dq[n - 1] = c.dq[n - 2];
  for (int i = 0; i < n; ++i) {
    const Eigen::Quaternionf t = c.root[i] * Eigen::AngleAxisf(c.q[i][waist_index],
        Eigen::Vector3f::UnitZ());
    c.torso_tilt.push_back(std::acos(std::clamp(t.toRotationMatrix()(2, 2), -1.f, 1.f)));
  }
  return c;
}

inline float quintic(float x) {return x * x * x * (10.f - 15.f * x + 6.f * x * x);}
inline float dquintic(float x) {return 30.f * x * x * (1.f - x) * (1.f - x);}

inline Eigen::Quaternionf yaw_quat(const Eigen::Quaternionf & q)
{
  const float yaw = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()),
      1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));
  return Eigen::Quaternionf(std::cos(yaw * .5f), 0.f, 0.f, std::sin(yaw * .5f));
}

// record_student_mujoco.slerp (wxyz, shortest path, linear fallback)
inline Eigen::Quaternionf slerp(const Eigen::Quaternionf & a, Eigen::Quaternionf b, float s)
{
  Eigen::Vector4f q0(a.w(), a.x(), a.y(), a.z()), q1(b.w(), b.x(), b.y(), b.z());
  if (q0.dot(q1) < 0.f) {q1 = -q1;}
  const float dot = std::min(1.f, std::abs(q0.dot(q1)));
  const float th = std::acos(dot);
  Eigen::Vector4f o = std::sin(th) < 1e-6f ? Eigen::Vector4f((1 - s) * q0 + s * q1) :
    Eigen::Vector4f((std::sin((1 - s) * th) * q0 + std::sin(s * th) * q1) / std::sin(th));
  o.normalize();
  return Eigen::Quaternionf(o[0], o[1], o[2], o[3]);
}

struct Pose {Vec q{Vec::Zero()}; Eigen::Quaternionf torso{Eigen::Quaternionf::Identity()};};

struct Inputs
{
  bool sd{false};                 // trigger switch on (SA CH5 or SD CH8)
  int dial{-1};                   // clip index selected by the dial (-1: none)
  Eigen::Vector3f stick{Eigen::Vector3f::Zero()};
  Vec q{Vec::Zero()}, dq{Vec::Zero()};
  Eigen::Vector3f gyro{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf torso{Eigen::Quaternionf::Identity()};  // IMU * Rz(waist), world
  bool confirm{false};             // confirm switch position (SD toggle); a change of position confirms
  int select{0};                   // 0 none / middle, 1 low, 2 high (select channel, CH10 pot)
  int loop{-1};                    // 2026-10-15: loop slot selected by the dial (-1: none)
};

class Gate {
public:
  Parameters params;
  std::vector<Clip> clips;
  int state{WALK}, clip{0}, frame{0};
  float phase{0}, w{0}, quiet{0}, pending{0}, tau{0}, T{0.5f};
  Eigen::Vector3f cmd{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf init{Eigen::Quaternionf::Identity()};
  Pose stance, b_from, b_to;
  bool sd_previous{false}, sd_known{false}, abort_requested{false};
  bool walk_mode{false};           // triggered while moving: walk path (arms only, legs walk)
  bool decided{false}, holding{false}, confirm_previous{false}, confirm_known{false};
  int choice{0};                   // interactive choice remembered from the trigger on (0 none, 1 low, 2 high)
  float handoff{1.f};              // leg handoff to walking after a mid-clip switch
  int loop{-1}, loop_pos{0};       // 2026-10-15: active loop slot (-1 none) and its current clip position
  bool loop_stop{false};           // stop requested: finish the current clip, start no next one
  bool hold_release_requested{false};
  float hold_elapsed{0.f};

  Gate(Parameters p, std::vector<Clip> c)
  : params(p), clips(std::move(c))
  {
    if (clips.empty()) {throw std::runtime_error("mimic_gate: no clips");}
  }
  int observation_size() const {return kTeacherDim + 1 + static_cast<int>(clips.size());}
  bool walk_clip(int c) const
  {
    return c >= 0 && c < static_cast<int>(params.walk_allowed.size()) && params.walk_allowed[c];
  }
  bool fast_clip(int c) const
  {
    return c >= 0 && c < static_cast<int>(params.fast_start.size()) && params.fast_start[c];
  }
  bool arms_only_clip(int c) const
  {
    return c >= 0 && c < static_cast<int>(params.arms_only_ref.size()) && params.arms_only_ref[c];
  }
  bool hold_clip(int c) const
  {
    if (c < 0 || c >= static_cast<int>(params.hold_frames.size())) {return false;}
    return params.hold_frames[c] >= 0;
  }
  // A walk-allowed clip is playing (arm masking and per-joint composition active).
  bool walking_clip_active() const
  {
    return state != WALK && w > 0.f && walk_clip(clip) && walk_mode;
  }

  void reset()
  {
    state = WALK; clip = frame = 0; phase = w = quiet = pending = tau = 0.f;
    T = params.bridge_min_seconds;
    cmd.setZero(); init.setIdentity(); stance = b_from = b_to = Pose{};
    sd_known = abort_requested = false;
    walk_mode = decided = holding = confirm_known = false;
    choice = 0; handoff = 1.f;
    loop = -1; loop_pos = 0; loop_stop = false;
    hold_release_requested = false; hold_elapsed = 0.f;
  }

  Pose frame_pose(int f) const
  {
    const Clip & c = clips[clip];
    return {c.q[f],
      c.root[f] * Eigen::AngleAxisf(c.q[f][params.waist_index], Eigen::Vector3f::UnitZ())};
  }

  float bridge_seconds(const Pose & a, const Pose & b) const
  {
    const Eigen::Vector3f ua = a.torso.toRotationMatrix().row(2),
      ub = b.torso.toRotationMatrix().row(2);
    const float tilt = std::acos(std::clamp(ua.dot(ub), -1.f, 1.f));
    const float joint = (b.q - a.q).cwiseAbs().maxCoeff() / params.bridge_joint_speed;
    return std::clamp(std::max(joint, tilt / params.bridge_torso_speed), params.bridge_min_seconds,
        params.bridge_max_seconds);
  }

  // Reference joints, joint velocities and torso orientation (reference frame) of this tick.
  void reference(Vec & q, Vec & dq, Eigen::Quaternionf & torso) const
  {
    if (state == BRIDGE_IN || state == BRIDGE_OUT) {
      const float s = quintic(tau), ds = dquintic(tau) / T;
      q = b_from.q + s * (b_to.q - b_from.q);
      dq = ds * (b_to.q - b_from.q);
      torso = slerp(b_from.torso, b_to.torso, s);
      return;
    }
    if (state == CLIP_IN || state == CLIP_OUT) {
      q = stance.q; dq.setZero(); torso = stance.torso; return;
    }
    const Pose p = frame_pose(frame);
    q = p.q;
    dq = state == CLIP && !holding ? clips[clip].dq[frame] : Vec::Zero();
    torso = p.torso;
  }

  void step(const Inputs & in)
  {
    const float dt = params.dt;
    const bool rise = sd_known && in.sd && !sd_previous;
    const bool fall = sd_known && !in.sd && sd_previous;
    const bool stop = params.abort_on_release ? fall : rise;
    sd_previous = in.sd;
    sd_known = true;
    const bool valid_dial = in.dial >= 0 && in.dial < static_cast<int>(clips.size());
    const bool valid_loop = in.loop >= 0 && in.loop < static_cast<int>(params.loops.size());
    const bool starting = state == WALK && rise && (valid_dial || valid_loop);
    const bool release_hold = params.abort_on_release && fall && state != WALK &&
      state != PENDING && hold_clip(clip);
    const bool release_interactive = params.confirm_is_trigger && params.abort_on_release &&
      fall && state != WALK && state != PENDING && clip == params.inter_clip && !decided;
    bool cancel = false;
    if (starting) {
      loop = valid_dial ? -1 : in.loop; loop_pos = 0; loop_stop = false;
      start_clip(valid_dial ? in.dial : params.loops[loop][0], in);
      // Singleton loop slots place held clips after the repeating sequence in the dial table.
      if (loop >= 0 && params.loops[loop].size() == 1U && hold_clip(clip)) {
        loop_stop = true;
      }
    } else if (release_hold) {
      hold_release_requested = true;
      if (loop >= 0) {loop_stop = true;}
    } else if (release_interactive) {
      if (in.select != 0) {choice = in.select;}
    } else if (stop && loop >= 0 && state != PENDING) {
      loop_stop = true;  // 2026-10-15 loop: finish the current clip, no abort
    } else if (stop && state == PENDING) {
      cancel = true;  // w is still 0: back to walking
    } else if (stop && (state == CLIP_IN || state == BRIDGE_IN || state == CLIP)) {
      abort_requested = true;
    }
    if (params.abort_on_release && rise && loop >= 0 && state != WALK && !hold_clip(clip)) {
      loop_stop = false;  // Trigger back on: keep a repeating sequence running
    }
    if (cancel) {state = WALK; loop = -1;}
    const bool confirm_edge = confirm_known && in.confirm != confirm_previous &&
      !(params.confirm_is_trigger && starting);
    confirm_previous = in.confirm;
    confirm_known = true;
    // A walk clip started standing hands the legs to walking once the stick moves.
    if (params.switch_walk_seconds > 0.f &&
      (state == PENDING || state == CLIP_IN || state == BRIDGE_IN || state == CLIP) &&
      walk_clip(clip) && !walk_mode && in.stick.cwiseAbs().maxCoeff() > params.quiet_command)
    {
      walk_mode = true; handoff = w > 0.f ? 0.f : 1.f;
    }
    if (walk_mode && handoff < 1.f) {
      handoff = std::min(1.f, handoff + dt / params.switch_walk_seconds);
    }
    // 2026-10-13 patch3: a flip any time after the trigger (hand still rising) is remembered
    if (state != WALK && clip == params.inter_clip && !decided && confirm_edge && in.select != 0) {
      choice = in.select;
    }
    const bool moving_ok = state != WALK && walk_clip(clip) && walk_mode;
    if (state == WALK || moving_ok) {
      cmd = in.stick;
    } else {
      for (int i = 0; i < 3; ++i) {
        const float lim = params.acceleration_limits[i] * dt;
        cmd[i] += std::clamp(-cmd[i], -lim, lim);
      }
    }
    if (state == PENDING) {
      const float leg = std::sqrt(in.dq.head<12>().squaredNorm() / 12.f);
      const bool q = cmd.cwiseAbs().maxCoeff() < params.quiet_command &&
        leg < params.quiet_leg_speed &&
        in.gyro.norm() < params.quiet_ang_speed;
      quiet = q ? quiet + dt : 0.f;
      pending += dt;
    }
    if (state <= PENDING || moving_ok) {
      init = yaw_quat(in.torso) * yaw_quat(clips[clip].root[frame]).conjugate();
    }
    if (state == PENDING &&
      (moving_ok || quiet >= params.quiet_seconds - kTimeEps ||
      pending >= params.pending_timeout_seconds - kTimeEps ||
      (fast_clip(clip) && quiet > 0.f)))
    {
      state = CLIP_IN; phase = 0.f;
      stance = {in.q, init.conjugate() * in.torso};
    }
    bool to_clip = false, entered = false;
    // abort (mdp/student.py): executed only while the reference torso is upright
    if (abort_requested && (state == CLIP_IN || state == BRIDGE_IN || state == CLIP) &&
      clips[clip].torso_tilt[frame] < params.abort_max_reference_tilt)
    {
      abort_requested = false;
      if (state == CLIP_IN) {
        state = CLIP_OUT;
      } else {
        Pose now;
        Vec unused;
        reference(now.q, unused, now.torso);
        enter_bridge_out(now);
        entered = true;
      }
    }
    if (state == CLIP_IN) {
      phase = std::min(1.f,
          phase + dt / (fast_clip(clip) &&
          params.fast_ramp_seconds > 0.f ? params.fast_ramp_seconds : params.ramp_seconds));
      if (phase >= 1.f - kTimeEps) {
        phase = 1.f;
        state = BRIDGE_IN; tau = 0.f; frame = 0; b_from = stance; b_to = frame_pose(0);
        T = bridge_seconds(b_from, b_to);
        entered = true;
      }
    }
    if (state == BRIDGE_IN && !entered) {
      tau = std::min(1.f, tau + dt / T);
      if (tau >= 1.f - kTimeEps) {tau = 1.f; state = CLIP; to_clip = true;}
    }
    holding = false;
    if (state == CLIP && !to_clip && !entered && clip == params.inter_clip && !decided &&
      (choice != 0 || frame >= params.inter_hold_frame))
    {
      if (choice == 1 && params.inter_low_clip >= 0) {
        const Pose from = frame_pose(frame);
        clip = params.inter_low_clip; frame = params.inter_low_frame; decided = true;
        b_from = from; b_to = frame_pose(frame); T = bridge_seconds(b_from, b_to);
        state = BRIDGE_IN; tau = 0.f;
        entered = true;
      } else if (choice == 2) {
        decided = true;
      } else if (frame >= params.inter_hold_frame) {
        holding = true;
      }
    }
    if (state == CLIP && !to_clip && !entered && hold_clip(clip) &&
      frame >= params.hold_frames[clip])
    {
      if (confirm_edge) {hold_release_requested = true;}
      if (params.hold_max_seconds > 0.f &&
        hold_elapsed + dt >= params.hold_max_seconds - kTimeEps)
      {
        hold_release_requested = true;
      }
      holding = !hold_release_requested;
      if (holding) {hold_elapsed += dt;}
    }
    if (state == CLIP && !to_clip && !entered && !holding) {
      const int last = clips[clip].frames() - 1;
      if (frame >= last) {enter_bridge_out(frame_pose(last)); entered = true;} else {++frame;}
    }
    if (state == BRIDGE_OUT && !entered) {
      tau = std::min(1.f, tau + dt / T);
      if (tau >= 1.f - kTimeEps) {tau = 1.f; state = CLIP_OUT; phase = 1.f; entered = true;}
    }
    if (state == CLIP_OUT && !entered) {
      phase = std::max(0.f, phase - dt / params.ramp_seconds);
      if (phase <= kTimeEps) {
        phase = 0.f;state = WALK; abort_requested = false;
        // 2026-10-15 loop: the next clip starts like a fresh press (PENDING), unless a stop was requested
        if (loop >= 0 && !loop_stop && (in.sd || !params.abort_on_release)) {
          loop_pos = (loop_pos + 1) % static_cast<int>(params.loops[loop].size());
          start_clip(params.loops[loop][loop_pos], in);
        } else {
          loop = -1;
        }
      }
    }
    if (state == WALK) {phase = 0.f;}
    if (state == CLIP || state == BRIDGE_IN || state == BRIDGE_OUT) {phase = 1.f;}
    w = state <= PENDING ? 0.f : quintic(phase);
  }

  // [ref q, ref dq, anchor 6, gyro, q - default, dq, last action, w, clip one-hot]
  void observation(
    const Inputs & in, const Vec & default_q, const float * last_action,
    float * out) const
  {
    Vec rq, rdq;
    Eigen::Quaternionf rt;
    reference(rq, rdq, rt);
    if (state != WALK && walk_clip(clip) && walk_mode && arms_only_clip(clip)) {
      rq.head<13>() = in.q.head<13>();
      rdq.head<13>() = in.dq.head<13>();
      rt = init.conjugate() * in.torso;
    }
    const Eigen::Matrix3f r = ((init * rt).conjugate() * in.torso).toRotationMatrix().transpose();
    int k = 0;
    for (int j = 0; j < kJoints; ++j) {out[k++] = rq[j];}
    for (int j = 0; j < kJoints; ++j) {out[k++] = rdq[j];}
    const float anchor[6] = {r(0, 0), r(0, 1), r(1, 0), r(1, 1), r(2, 0), r(2, 1)};
    for (float v : anchor) {out[k++] = v;}
    for (int j = 0; j < 3; ++j) {out[k++] = in.gyro[j];}
    for (int j = 0; j < kJoints; ++j) {out[k++] = in.q[j] - default_q[j];}
    for (int j = 0; j < kJoints; ++j) {out[k++] = in.dq[j];}
    for (int j = 0; j < kJoints; ++j) {out[k++] = last_action[j];}
    out[k++] = w;
    for (int c = 0; c < static_cast<int>(clips.size()); ++c) {out[k++] = c == clip ? 1.f : 0.f;}
  }

private:
  void start_clip(int c, const Inputs & in)
  {
    state = PENDING; clip = c; frame = 0; quiet = pending = 0.f; abort_requested = false;
    walk_mode = walk_clip(c) && in.stick.cwiseAbs().maxCoeff() > params.quiet_command;
    decided = holding = false;
    choice = 0; handoff = 1.f;
    hold_release_requested = false; hold_elapsed = 0.f;
  }

  void enter_bridge_out(const Pose & from)
  {
    b_from = from; b_to = stance; T = bridge_seconds(b_from, b_to); state = BRIDGE_OUT; tau = 0.f;
  }
};

}  // namespace k1_mimic_gate
