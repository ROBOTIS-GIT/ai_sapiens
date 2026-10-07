#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "ai_sapiens_sim2real/axis_range.hpp"

namespace ai_sapiens_sim2real
{

// Deployment adapter for policies with an arm_mode observation. A rising SA edge
// toggles the target; the observation and action offset share the same blend.
class ArmMode
{
public:
  ArmMode(const YAML::Node & config, const std::vector<std::string> & joints,
    const std::vector<float> & hold_offset, double dt)
  : hold_(hold_offset), ready_(hold_offset), offset_(hold_offset), dt_(dt)
  {
    const auto params = config["observations"]["arm_mode"]["params"];
    channel = params["sa_channel"].as<int>(5);
    on_min = params["sa_on_min_us"].as<int>(1950);
    on_max = params["sa_on_max_us"].as<int>(2050);
    duration_ = params["transition_seconds"].as<double>(1.0);
    if (channel < 1 || channel > 16 || on_min < 500 || on_max > 2500 ||
      on_min > on_max || !std::isfinite(duration_) || duration_ <= 0 ||
      !std::isfinite(dt_) || dt_ <= 0 || joints.size() != hold_.size())
    {
      throw std::runtime_error("Invalid arm_mode channel, PWM, timing or joint configuration");
    }
    const auto action = config["actions"]["joint_pos"];
    if (action["command_name"].as<std::string>("") != "arm_mode") {
      throw std::runtime_error("Arm mode requires joint_pos.command_name: arm_mode");
    }
    const auto ready = action["ready_arm_joint_pos"];
    if (!ready.IsMap() || ready.size() == 0) {
      throw std::runtime_error("Arm mode requires ready_arm_joint_pos");
    }
    for (const auto & item : ready) {
      const auto name = item.first.as<std::string>();
      const auto joint = std::find(joints.begin(), joints.end(), name);
      const float value = item.second.as<float>();
      if (joint == joints.end() || !std::isfinite(value)) {
        throw std::runtime_error("Invalid ready arm joint: " + name);
      }
      ready_[static_cast<size_t>(joint - joints.begin())] = value;
    }
    for (float value : hold_) {
      if (!std::isfinite(value)) throw std::runtime_error("Invalid hold offset");
    }
    const auto ranges = config["commands"]["base_velocity"];
    walk_ranges = read_ranges(ranges["ranges"]);
    hold_ranges = read_ranges(ranges["hold_ranges"]);
    reset(false);
  }

  void reset(bool pressed)
  {
    previous_ = pressed;  // An already-held button on entry is not a new press.
    target_ = false;
    ratio_ = 0.0F;
    offset_ = ready_;
  }

  void step(bool pressed)
  {
    if (pressed && !previous_) target_ = !target_;
    previous_ = pressed;
    const float target = target_ ? 1.0F : 0.0F;
    const float increment = static_cast<float>(dt_ / duration_);
    ratio_ += std::clamp(target - ratio_, -increment, increment);
    if (std::abs(target - ratio_) < 1e-6F) ratio_ = target;
    for (size_t j = 0; j < offset_.size(); ++j) {
      offset_[j] = ready_[j] + ratio_ * (hold_[j] - ready_[j]);
    }
  }

  float ratio() const {return ratio_;}
  bool target() const {return target_;}
  const std::vector<float> & offsets() const {return offset_;}
  const AxisRanges & ranges() const
  {
    return target_ || ratio_ > 0.0F ? hold_ranges : walk_ranges;
  }

  int channel{5}, on_min{1950}, on_max{2050};
  AxisRanges walk_ranges, hold_ranges;

private:
  static AxisRanges read_ranges(const YAML::Node & node)
  {
    const auto read = [&](const char * key) {
        const auto values = node[key].as<std::vector<double>>();
        if (values.size() != 2 || !std::isfinite(values[0]) ||
          !std::isfinite(values[1]) || values[0] > 0 || values[1] < 0 ||
          values[0] >= values[1])
        {
          throw std::runtime_error(std::string("Invalid arm mode range: ") + key);
        }
        return AxisRange{values[0], values[1]};
      };
    return AxisRanges{read("lin_vel_x"), read("lin_vel_y"), read("ang_vel_z")};
  }

  std::vector<float> hold_, ready_, offset_;
  double dt_, duration_{1.0};
  bool previous_{false}, target_{false};
  float ratio_{0.0F};
};

}  // namespace ai_sapiens_sim2real
