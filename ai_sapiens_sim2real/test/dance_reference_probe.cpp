// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0.
// Offline probe driven by recorded training observations; never publishes commands.

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
#include "ai_sapiens_sim2real/observation/observations.hpp"
#include "ai_sapiens_sim2real/policy/dance_motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/motion_steering_release.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

using namespace ai_sapiens_sim2real;  // NOLINT

void check_command_release(const char * config_path, const char * motion_path)
{
  Sim2RealConfig cfg(config_path);
  if (!cfg.dance_reference() || !cfg.dance_reference()->stop_new_steps_on_release ||
    !cfg.steering())
  {
    throw std::runtime_error("Release regression requires a v3 dance policy");
  }
  MotionReference motion(motion_path, 1.0 / cfg.step_dt(), cfg.policy_joints(), true);
  const auto require = [](bool condition, const char * message) {
      if (!condition) {throw std::runtime_error(message);}
    };
  for (const Eigen::Vector3f & request : {Eigen::Vector3f(.3f, 0, 0),
      Eigen::Vector3f(0, -.3f, 0), Eigen::Vector3f(0, 0, .3f), Eigen::Vector3f(-.3f, .3f, -.3f)})
  {
    DanceMotionReference dance(*cfg.dance_reference(), cfg.policy_joints(), cfg.step_dt(), motion);
    PlanarMotionSteering steering;
    motion.seek(0);
    dance.reset(0);
    Eigen::Vector2f previous_root = motion.root_position().head<2>();
    int frame = 0;
    const auto advance = [&](const Eigen::Vector3f & input) {
        require(++frame < motion.frame_count(), "Release test exceeded motion length");
        motion.seek(frame * cfg.step_dt());
        const Eigen::Vector2f root = motion.root_position().head<2>();
        const auto command = MotionSteeringRelease::filter_command(
          input, cfg.steering()->command_deadband);
        steering.step(previous_root, root, Eigen::Quaternionf::Identity(), command,
          cfg.step_dt(), *cfg.steering());
        // Deliberately retain accumulated root error: it must not cause catch-up
        // placements after release, even before the velocity filter has settled.
        dance.step(frame, steering,
          Eigen::Vector3f(steering.velocity.x(), steering.velocity.y(), 0),
          !command.isZero(0.0f));
        previous_root = root;
        for (float value : dance.output().joint_pos) {
          require(std::isfinite(value), "Non-finite release joint target");
        }
      };
    while (dance.output().added_steps == 0 && frame < 1500) {advance(request);}
    require(dance.output().added_steps > 0, "Active command did not generate a placement");
    // Advance into the gesture instead of releasing at its zero-height first frame.
    for (int i = 0; i < 3; ++i) {
      advance(request);
    }
    require(dance.output().root_shift.head<2>().norm() > 1e-7f,
      "Release scenario did not reach an active gesture");
    const int added = dance.output().added_steps;
    advance(Eigen::Vector3f(.1f, -.1f, .1f));  // inclusive deadband means released
    require(steering.velocity.norm() > .01f, "Release scenario has no residual filtered velocity");
    require(dance.output().root_shift.head<2>().norm() > 1e-7f,
      "Releasing the command cancelled the active gesture");
    require(dance.output().added_steps == added, "Release started a new placement");
    for (int i = 0; i < 500; ++i) {
      advance(Eigen::Vector3f::Zero());
      require(dance.output().added_steps == added, "Released planner added a catch-up placement");
    }
    require(dance.output().root_shift.head<2>().norm() < 1e-6f,
      "Released gesture did not finish");
    for (int i = 0; i < 500 && dance.output().added_steps == added; ++i) {
      advance(request);
    }
    require(dance.output().added_steps > added, "Re-applied command did not enable placements");
    dance.reset(0);
    require(dance.output().added_steps == 0 && dance.output().root_shift.isZero(),
      "Re-entry did not clear the previous planner state");
    std::cout << "PASS: v3 release, completion, re-command and reset: " << request.transpose() <<
      '\n';
  }
}

int main(int argc, char ** argv)
{
  try {
    if (argc == 4 && std::string(argv[1]) == "--check-release") {
      check_command_release(argv[2], argv[3]);
      return 0;
    }
    if (argc != 5) {
      throw std::runtime_error(
        "usage: dance_reference_probe sim2real.yaml motion.csv input.csv output.csv");
    }
    Sim2RealConfig cfg(argv[1]);
    if (!cfg.dance_reference() || !cfg.steering()) {
      throw std::runtime_error("Missing dance config");
    }
    MotionReference motion(argv[2], 1.0 / cfg.step_dt(), cfg.policy_joints(), true);
    DanceMotionReference dance(*cfg.dance_reference(), cfg.policy_joints(), cfg.step_dt(), motion);
    SharedControlData shared;
    shared.resize(23, 23);
    shared.policy.uses_global_position = shared.policy.uses_motion_steering = true;
    PolicyJointContext joints;
    joints.policy_joint_names = cfg.policy_joints();
    for (size_t i = 0; i < 23; ++i) {
      joints.policy_to_controller.push_back(i);
      shared.output.default_joint_pos[i] = cfg.joint_properties().default_position[i];
    }
    ObservationContext context{shared, joints, &motion};
    MotionSteeringRelease release;
    Eigen::Vector2f previous_root = Eigen::Vector2f::Zero();
    std::ifstream input(argv[3]);
    std::ofstream output(argv[4]);
    if (!input || !output) {throw std::runtime_error("Cannot open probe input/output");}
    output << std::setprecision(9);
    std::string line;
    while (std::getline(input, line)) {
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      int frame, reset;
      Eigen::Vector3f requested;
      Eigen::Vector2f robot;
      float qw, qx, qy, qz;
      row >> frame >> reset >> requested.x() >> requested.y() >> requested.z() >> robot.x() >>
      robot.y();
      row >> qw >> qx >> qy >> qz;
      std::array<float, 131> expected{};
      for (auto & value : expected) {
        row >> value;
      }
      if (!row) {throw std::runtime_error("Invalid probe row");}
      const Eigen::Quaternionf orientation(qw, qx, qy, qz);
      motion.seek(frame * cfg.step_dt());
      auto & steering = shared.policy.motion_steering;
      if (reset) {
        steering.reset();
        release.reset();
        dance.reset(frame);
        shared.policy.motion_frame.align(robot, orientation, motion.root_position().head<2>(),
          motion.root_quaternion());
      }
      shared.localization.position = robot;
      shared.localization.orientation = orientation;
      shared.sensors.orientation = orientation;
      const auto command = release.filter_command(requested, cfg.steering()->command_deadband);
      const auto root_q = shared.policy.motion_frame.orientation(orientation);
      if (!reset) {
        steering.step(previous_root, motion.root_position().head<2>(), root_q, command,
          cfg.step_dt(), *cfg.steering());
      }
      if (cfg.steering()->release_on_zero) {
        release.apply(steering, command, shared.policy.motion_frame.position(robot), root_q,
          shared.policy.motion_frame.reference_position(motion.root_position().head<2>()),
          motion.root_quaternion(),
          cfg.steering()->release_velocity_threshold);
      }
      if (!reset) {
        const Eigen::Vector2f velocity = Eigen::Rotation2Df(PlanarMotionSteering::heading(root_q)) *
          steering.velocity.head<2>();
        dance.step(frame, steering, Eigen::Vector3f(velocity.x(), velocity.y(), 0),
          !command.isZero(0.0f));
      }
      previous_root = motion.root_position().head<2>();
      const auto & ref = dance.output();
      motion.set_joint_targets(Eigen::Map<const Eigen::VectorXf>(ref.joint_pos.data(), 23),
        Eigen::Map<const Eigen::VectorXf>(ref.joint_vel.data(), 23), ref.root_shift);
      // Replay identical proprioceptive samples, then assemble every observation term.
      shared.sensors.angular_velocity = Eigen::Vector3f(expected[52], expected[53], expected[54]);
      for (int j = 0; j < 23; ++j) {
        shared.sensors.joint_pos[j] = expected[55 + j] + shared.output.default_joint_pos[j];
        shared.sensors.joint_vel[j] = expected[78 + j];
        shared.policy.last_action[j] = expected[101 + j];
      }
      const auto write = [&](float value) {output << value << ',';};
      for (const auto & term : cfg.observations()) {
        const auto values =
          ObservationRegistry::get_registry().at(term.first.as<std::string>())(context,
          term.second["params"]);
        for (float value : values) {
          write(value);
        }
      }
      for (int j = 0; j < 3; ++j) {
        write(ref.root_shift[j]);
      }
      for (const auto & foot : ref.foot_position) {
        for (int j = 0; j < 3; ++j) {
          write(foot[j]);
        }
      }
      for (const auto & q : ref.foot_orientation) {
        write(q.w()); write(q.x()); write(q.y()); write(q.z());
      }
      for (bool valid : ref.valid) {
        write(valid ? 1.0f : 0.0f);
      }
      for (float value : ref.air) {
        write(value);
      }
      output << ref.added_steps << '\n';
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
