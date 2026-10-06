// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_DEPTH__DEPTH_PROCESSOR_HPP_
#define AI_SAPIENS_DEPTH__DEPTH_PROCESSOR_HPP_
#include <array>
#include <deque>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <ai_sapiens_interfaces/msg/depth_history.hpp>
namespace ai_sapiens_depth
{
struct DepthConfig
{
  int width{32}, height{18}, frames{8}, raw_width{64}, raw_height{36}, stride{5};
  std::array<int, 4> crop{18, 0, 16, 16};
  std::array<double, 2> fov{87, 58};
  std::array<double, 3> position{};
  std::array<double, 4> quaternion{};  // MuJoCo wxyz, normalized
  double near{0.2}, far{2.5}, period{0.02}, max_age{0.1}, sigma{1.0};
  std::string parent, frame, topic;
  static DepthConfig parse(const YAML::Node & observation);
  static DepthConfig load(const std::string & policy_yaml);
};
// Canonical input is metric image-plane depth, 64x36. Real input must be a
// rectified depth image with CameraInfo.P for that same image (not RGB).
std::vector<float> metric_image(
  const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & info,
  const DepthConfig & config, double uint16_scale);
std::vector<float> preprocess(
  std::vector<float> raw, const DepthConfig & config, bool inpaint);
class DepthHistory
{
public:
  enum class Sampling {Timestamp, FrameIndex};
  explicit DepthHistory(DepthConfig config, Sampling sampling = Sampling::Timestamp)
  : config_(config), sampling_(sampling) {}
  void reset();
  bool push(const std::vector<float> & frame, double stamp);
  ai_sapiens_interfaces::msg::DepthHistory sample(int delay_steps = 0) const;

private:
  struct Frame {double stamp; std::vector<float> values;};
  DepthConfig config_;
  Sampling sampling_;
  std::deque<Frame> frames_;
};
}  // namespace ai_sapiens_depth
#endif  // AI_SAPIENS_DEPTH__DEPTH_PROCESSOR_HPP_
