// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "mujoco_hardware_interface/depth_camera_node.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <type_traits>
namespace mujoco_hardware_interface
{
namespace
{
template<typename Ray>
mjtNum cast_ray(
  Ray ray, const mjModel * model, const mjData * data,
  const mjtNum * position, const mjtNum * direction, const mjtByte * groups, int excluded)
{
  int hit;
  if constexpr (std::is_invocable_v<Ray, const mjModel *, const mjData *,
    const mjtNum *, const mjtNum *, const mjtByte *, int, int, int *, mjtNum *>)
  {
    return ray(model, data, position, direction, groups, 1, excluded, &hit, nullptr);
  } else {
    return ray(model, data, position, direction, groups, 1, excluded, &hit);
  }
}
}
DepthCameraNode::DepthCameraNode(
  std::shared_ptr<ai_sapiens_mujoco::MujocoSimulation> sim,
  const std::string & path)
: Node("mujoco_depth_camera"), sim_(sim),
  config_(ai_sapiens_depth::DepthConfig::load(path))
{
  parent_ = mj_name2id(sim_->model(), mjOBJ_BODY, config_.parent.c_str());
  excluded_ = mj_name2id(sim_->model(), mjOBJ_BODY, "head_link");
  if (parent_ < 0 || excluded_ < 0) {throw std::runtime_error("D436 body/mount is missing");}
  rays_.resize(3 * config_.raw_width * config_.raw_height);
  const double fx = config_.raw_width / (2 * std::tan(config_.fov[0] * std::acos(-1.0) / 360));
  const double fy = config_.raw_height / (2 * std::tan(config_.fov[1] * std::acos(-1.0) / 360));
  for (int y = 0; y < config_.raw_height; ++y) {
    for (int x = 0; x < config_.raw_width; ++x) {
      auto * ray = &rays_[3 * (y * config_.raw_width + x)];
      ray[0] = 1; ray[1] = -(x + 0.5 - config_.raw_width * 0.5) / fx;
      ray[2] = -(y + 0.5 - config_.raw_height * 0.5) / fy;
      mju_normalize3(ray);
    }
  }
  {
    std::lock_guard<std::mutex> lock(sim_->mutex());
    model_.reset(mj_copyModel(nullptr, sim_->model()));
  }
  if (!model_) {throw std::runtime_error("Could not copy depth model");}
  snapshot_.reset(mj_makeData(model_.get()));
  if (!snapshot_) {throw std::runtime_error("Could not allocate depth snapshot");}
  publisher_ = create_publisher<sensor_msgs::msg::Image>("/k1/d436/depth/image_rect_raw",
      rclcpp::SensorDataQoS());
  timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {capture();});
}
DepthCameraNode::~DepthCameraNode() = default;
void DepthCameraNode::capture()
{
  const auto * model = model_.get();
  bool reset = false;
  rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(sim_->mutex());
    const double time = sim_->data()->time;
    reset = last_time_ >= 0 && time < last_time_;
    if (!reset && last_time_ >= 0 && time < last_time_ + config_.period - 1e-9) {return;}
    mj_copyData(snapshot_.get(), model, sim_->data());
    stamp = now();
    last_time_ = time;
  }
  if (reset) {
    sensor_msgs::msg::Image empty;
    empty.header.stamp = stamp; empty.header.frame_id = config_.frame;
    publisher_->publish(empty);  // Clears downstream history on a simulation reset.
  }
  mj_forward(model, snapshot_.get());
  mjtNum position[3], mount[9], rotation[9];
  mju_mulMatVec3(position, snapshot_->xmat + 9 * parent_, config_.position.data());
  mju_addTo3(position, snapshot_->xpos + 3 * parent_);
  mju_quat2Mat(mount, config_.quaternion.data());
  mju_mulMatMat(rotation, snapshot_->xmat + 9 * parent_, mount, 3, 3, 3);
  const mjtByte groups[mjNGROUP] = {1, 0, 1, 0, 0, 0};
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp; image.header.frame_id = config_.frame;
  image.width = config_.raw_width; image.height = config_.raw_height;
  image.encoding = "32FC1"; image.step = image.width * sizeof(float);
  image.data.resize(image.step * image.height);
  for (size_t i = 0; i < rays_.size() / 3; ++i) {
    mjtNum world[3];
    mju_mulMatVec3(world, rotation, &rays_[3 * i]);
    const auto distance = cast_ray(mj_ray, model, snapshot_.get(), position, world, groups,
        excluded_);
    const float value = distance < 0 ? config_.far : std::min(config_.far, distance * rays_[3 * i]);
    std::memcpy(image.data.data() + i * sizeof(float), &value, sizeof(float));
  }
  publisher_->publish(image);
}
}  // namespace mujoco_hardware_interface
