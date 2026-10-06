// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "ai_sapiens_depth/depth_processor.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace ai_sapiens_depth
{
namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
}
DepthConfig DepthConfig::load(const std::string & path)
{
  return parse(YAML::LoadFile(path)["observations"]["depth_image"]);
}
DepthConfig DepthConfig::parse(const YAML::Node & obs)
{
  DepthConfig c;
  const auto p = obs["params"], camera = p["camera"];
  require(obs["history_length"].as<int>() == 1, "depth_image already contains history");
  require(obs["scale"].as<double>() == 1.0, "depth_image scale must be 1");
  require(obs["clip"].as<std::vector<double>>() == std::vector<double>({0, 1}),
    "depth_image clip must be [0,1]");
  c.width = p["width"].as<int>(); c.height = p["height"].as<int>();
  c.frames = p["frames"].as<int>(); c.topic = p["topic"].as<std::string>();
  c.max_age = p["max_age_s"].as<double>();
  require(c.width == 32 && c.height == 18 && c.frames == 8, "Expected 8x18x32 depth history");
  require(camera["resolution"].as<std::vector<int>>() == std::vector<int>({32, 18}) &&
    camera["raw_resolution"].as<std::vector<int>>() == std::vector<int>({64, 36}),
    "Unsupported depth resolution");
  require(camera["crop_region"].as<std::vector<int>>() == std::vector<int>({18, 0, 16, 16}),
    "Unsupported depth crop");
  c.stride = camera["history_skip_frames"].as<int>();
  c.period = camera["update_period"].as<double>();
  require(c.stride == 5 && camera["history_length"].as<int>() == 8 &&
    std::abs(c.period - 0.02) < 1e-9, "Unsupported depth timing");
  require(camera["delay_range"].as<std::vector<int>>() == std::vector<int>({0, 1}),
    "Unsupported depth delay");
  for (int i = 0; i < 2; ++i) {
    c.fov[i] = camera["fov_deg"][i].as<double>();
  }
  c.near = camera["depth_range_m"][0].as<double>();
  c.far = camera["depth_range_m"][1].as<double>();
  c.sigma = camera["gaussian_blur"]["sigma"].as<double>();
  require(std::isfinite(c.near) && std::isfinite(c.far) && c.near > 0 && c.far > c.near &&
    std::isfinite(c.max_age) && c.max_age > 0 && c.max_age <= 0.1 &&
    std::isfinite(c.sigma) && c.sigma > 0, "Invalid depth range/timeout/blur");
  require(camera["gaussian_blur"]["kernel_size"].as<int>() == 3, "Expected 3x3 blur");
  require(camera["normalization"]["type"].as<std::string>() == "linear" &&
    camera["normalization"]["output_range"].as<std::vector<double>>() ==
    std::vector<double>({0, 1}) &&
    camera["normalization"]["invalid_value"].as<std::string>() ==
    "nearest_valid_inpaint_else_max_distance", "Unsupported depth normalization");
  require(camera["onnx"]["input_name"].as<std::string>() == "obs" &&
    camera["onnx"]["input_layout"].as<std::string>() == "flat_term_major_tail",
    "Unsupported depth ONNX layout");
  c.parent = camera["parent_frame"].as<std::string>();
  c.frame = camera["optical_frame"].as<std::string>();
  require(!c.parent.empty() && !c.frame.empty() && !c.topic.empty(), "Missing depth frames/topic");
  require(camera["mount"]["convention"].as<std::string>() == "world", "Expected world mount");
  for (int i = 0; i < 3; ++i) {
    c.position[i] = camera["mount"]["position"][i].as<double>();
    require(std::isfinite(c.position[i]), "Invalid camera mount");
  }
  double norm = 0;
  for (int i = 0; i < 4; ++i) {
    c.quaternion[(i + 1) % 4] = camera["mount"]["rotation_xyzw"][i].as<double>();
    norm += c.quaternion[(i + 1) % 4] * c.quaternion[(i + 1) % 4];
  }
  require(std::isfinite(norm) && norm > 1e-12, "Invalid mount quaternion");
  for (auto & v : c.quaternion) {
    v /= std::sqrt(norm);
  }
  for (auto v : c.fov) {
    require(std::isfinite(v) && v > 0 && v < 180, "Invalid FOV");
  }
  return c;
}
std::vector<float> metric_image(
  const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & info,
  const DepthConfig & c, double uint16_scale)
{
  const bool u16 = image.encoding == "16UC1";
  const size_t bytes = u16 ? 2 : 4;
  require(u16 || image.encoding == "32FC1", "Depth must be 16UC1 or 32FC1");
  require(std::isfinite(uint16_scale) && uint16_scale > 0, "Invalid uint16 depth scale");
  require(image.width > 0 && image.height > 0 && info.width == image.width &&
    info.height == image.height && image.header.frame_id == info.header.frame_id,
    "Depth image/CameraInfo mismatch");
  require(image.step >= image.width * bytes &&
    image.data.size() >= static_cast<size_t>(image.step) * image.height, "Truncated depth image");
  const double fx = info.p[0], fy = info.p[5], cx = info.p[2], cy = info.p[6];
  require(std::isfinite(fx) && fx > 0 && std::isfinite(fy) && fy > 0 &&
    std::isfinite(cx) && std::isfinite(cy) && info.p[3] == 0 && info.p[7] == 0,
    "Expected rectified depth CameraInfo.P at the depth optical origin");
  const double tx = c.raw_width / (2 * std::tan(c.fov[0] * std::acos(-1.0) / 360));
  const double ty = c.raw_height / (2 * std::tan(c.fov[1] * std::acos(-1.0) / 360));
  std::vector<float> result(c.raw_width * c.raw_height);
  for (int y = 0; y < c.raw_height; ++y) {
    for (int x = 0; x < c.raw_width; ++x) {
      const double sx = fx * (x + 0.5 - c.raw_width * 0.5) / tx + cx;
      const double sy = fy * (y + 0.5 - c.raw_height * 0.5) / ty + cy;
      require(sx >= -0.5 && sx < image.width - 0.5 && sy >= -0.5 && sy < image.height - 0.5,
        "Real depth FOV does not cover the training camera; check profile and calibration");
      const auto offset = static_cast<size_t>(std::lround(sy)) * image.step +
        static_cast<size_t>(std::lround(sx)) * bytes;
      uint32_t bits = 0;
      for (size_t b = 0; b < bytes; ++b) {
        bits |= uint32_t(image.data[offset + b]) << (8 * (image.is_bigendian ? bytes - b - 1 : b));
      }
      float value;
      if (u16) {value = bits * uint16_scale;} else {std::memcpy(&value, &bits, 4);}
      result[y * c.raw_width + x] = value;
    }
  }
  return result;
}
std::vector<float> preprocess(std::vector<float> raw, const DepthConfig & c, bool inpaint)
{
  require(raw.size() == static_cast<size_t>(c.raw_width * c.raw_height), "Raw depth size mismatch");
  auto valid = [](float v) {return std::isfinite(v) && v > 0;};
  // Search original valid samples only. Filling holes cannot create new donors.
  if (inpaint) {
    const auto original = raw;
    for (int i = 0; i < static_cast<int>(raw.size()); ++i) {
      if (valid(original[i])) {continue;}
      int best = std::numeric_limits<int>::max();
      raw[i] = c.far;
      for (int j = 0; j < static_cast<int>(raw.size()); ++j) {
        if (!valid(original[j])) {continue;}
        const int dx = i % c.raw_width - j % c.raw_width, dy = i / c.raw_width - j / c.raw_width;
        if (dx * dx + dy * dy < best) {best = dx * dx + dy * dy; raw[i] = original[j];}
      }
    }
  }
  std::vector<float> crop(c.width * c.height), result(crop.size());
  for (int y = 0; y < c.height; ++y) {
    for (int x = 0; x < c.width; ++x) {
      double v = raw[(y + c.crop[0]) * c.raw_width + x + c.crop[2]];
      if (!valid(v)) {v = c.far;}
      crop[y * c.width + x] = (std::clamp(v, c.near, c.far) - c.near) / (c.far - c.near);
    }
  }
  double weights[3], sum = 0;
  for (int i = 0; i < 3; ++i) {
    weights[i] = std::exp(-0.5 * (i - 1) * (i - 1) / (c.sigma * c.sigma)); sum += weights[i];
  }
  for (auto & w : weights) {
    w /= sum;
  }
  for (int y = 0; y < c.height; ++y) {
    for (int x = 0; x < c.width; ++x) {
      double v = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          v += weights[dy + 1] * weights[dx + 1] *
            crop[std::clamp(y + dy, 0, c.height - 1) * c.width + std::clamp(x + dx, 0,
              c.width - 1)];
        }
      }
      result[y * c.width + x] = std::clamp(static_cast<float>(v), 0.0f, 1.0f);
    }
  }
  return result;
}
void DepthHistory::reset() {frames_.clear();}
bool DepthHistory::push(const std::vector<float> & values, double stamp)
{
  require(values.size() == static_cast<size_t>(config_.width * config_.height) &&
    std::isfinite(stamp) && stamp >= 0, "Invalid depth frame");
  for (float v : values) {
    require(std::isfinite(v) && v >= 0 && v <= 1, "Invalid depth pixel");
  }
  if (!frames_.empty()) {
    if (stamp < frames_.back().stamp || stamp - frames_.back().stamp > config_.max_age) {
      reset();
    } else if (stamp <= frames_.back().stamp + 1e-9) {return false;}
  }
  frames_.push_back({stamp, values});
  if (sampling_ == Sampling::FrameIndex) {
    const size_t count = (config_.frames - 1) * config_.stride + 2;
    while (frames_.size() > count) {frames_.pop_front();}
    return true;
  }
  const double span = ((config_.frames - 1) * config_.stride + 1) * config_.period;
  while (frames_.size() > 2 && frames_[1].stamp < stamp - span - config_.max_age) {
    frames_.pop_front();
  }
  // Bound memory even when an unexpected high-rate publisher is connected.
  while (frames_.size() > 1024) {frames_.pop_front();}
  return true;
}
ai_sapiens_interfaces::msg::DepthHistory DepthHistory::sample(int delay_steps) const
{
  require(!frames_.empty() && delay_steps >= 0 && delay_steps <= 1, "Invalid depth history/delay");
  ai_sapiens_interfaces::msg::DepthHistory out;
  out.width = config_.width; out.height = config_.height; out.frames = config_.frames;
  out.header.frame_id = config_.frame;
  for (int i = 0; i < config_.frames; ++i) {
    if (sampling_ == Sampling::FrameIndex) {
      // Match the simulator's frame stride and 0/1-frame delay. Timestamp
      // jitter must not turn a one-frame delay into a two-frame delay.
      const size_t offset = (config_.frames - 1 - i) * config_.stride + delay_steps;
      const auto & chosen = frames_[offset < frames_.size() ? frames_.size() - 1 - offset : 0];
      out.frame_stamps[i] = chosen.stamp;
      std::copy(chosen.values.begin(), chosen.values.end(),
          out.data.begin() + i * config_.width * config_.height);
      continue;
    }
    const double target = frames_.back().stamp -
      ((config_.frames - 1 - i) * config_.stride + delay_steps) * config_.period;
    auto chosen = frames_.begin();
    for (auto it = frames_.begin(); it != frames_.end() && it->stamp <= target + 1e-8; ++it) {
      chosen = it;
    }
    // Warm-fill from the first actual sample, retaining its true timestamp.
    require(target < frames_.front().stamp || target - chosen->stamp <= config_.max_age,
      "Depth history contains a timing gap");
    out.frame_stamps[i] = chosen->stamp;
    std::copy(chosen->values.begin(), chosen->values.end(),
        out.data.begin() + i * config_.width * config_.height);
  }
  return out;
}
}  // namespace ai_sapiens_depth
