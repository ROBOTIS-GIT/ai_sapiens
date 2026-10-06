// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "ai_sapiens_depth/depth_processor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <memory>
#include <cstring>
namespace ai_sapiens_depth
{
class DepthHistoryNode : public rclcpp::Node
{
public:
  DepthHistoryNode()
  : Node("depth_history"), config_(DepthConfig::load(
        declare_parameter<std::string>("policy_yaml", ""))),
    simulated_(declare_parameter<bool>("simulated", false)),
    history_(config_, simulated_ ? DepthHistory::Sampling::FrameIndex :
      DepthHistory::Sampling::Timestamp)
  {
    scale_ = declare_parameter<double>("uint16_depth_scale", 0.001);
    frame_ = declare_parameter<std::string>("input_frame", config_.frame);
    const auto input = declare_parameter<std::string>("image_topic",
        "/k1/d436/depth/image_rect_raw");
    const auto info = declare_parameter<std::string>("camera_info_topic",
        "/k1/d436/depth/camera_info");
    publisher_ = create_publisher<ai_sapiens_interfaces::msg::DepthHistory>(config_.topic,
        rclcpp::SensorDataQoS());
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(info, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {info_ = msg;});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(input, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {receive(*msg);});
  }

private:
  void receive(const sensor_msgs::msg::Image & image)
  {
    try {
      const double stamp = rclcpp::Time(image.header.stamp).seconds();
      const double age = now().seconds() - stamp;
      if (image.header.frame_id != frame_ || age < -0.02 || age > config_.max_age) {
        throw std::runtime_error("Depth frame or acquisition timestamp mismatch");
      }
      std::vector<float> raw;
      if (simulated_) {
        if (image.encoding != "32FC1" || image.width != 64 || image.height != 36 ||
          image.is_bigendian || image.step != 64 * 4 || image.data.size() != 64 * 36 * 4)
        {throw std::runtime_error("Invalid canonical virtual depth frame");}
        raw.resize(64 * 36);
        std::memcpy(raw.data(), image.data.data(), image.data.size());
      } else {
        if (!info_) {throw std::runtime_error("Waiting for depth CameraInfo");}
        raw = metric_image(image, *info_, config_, scale_);
      }
      if (history_.push(preprocess(raw, config_, !simulated_), stamp)) {
        auto output = history_.sample(simulated_ ? std::uniform_int_distribution<int>(0,
            1)(random_) : 0);
        output.header.stamp = image.header.stamp;
        publisher_->publish(output);
      }
    } catch (const std::exception & e) {
      history_.reset();
      // Do not republish old depth or refresh the watchdog on bad/empty images.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    }
  }
  DepthConfig config_;
  bool simulated_{false};
  DepthHistory history_;
  double scale_{0.001};
  std::string frame_;
  std::mt19937 random_{42};
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<ai_sapiens_interfaces::msg::DepthHistory>::SharedPtr publisher_;
};
}  // namespace ai_sapiens_depth
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ai_sapiens_depth::DepthHistoryNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_history"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
