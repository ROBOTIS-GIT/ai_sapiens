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
// Author: Eunsung Kim

#ifndef AI_SAPIENS_RC_BROADCASTER__AI_SAPIENS_RC_BROADCASTER_HPP_
#define AI_SAPIENS_RC_BROADCASTER__AI_SAPIENS_RC_BROADCASTER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ai_sapiens_interfaces/msg/rc_status.hpp"
#include "ai_sapiens_rc_broadcaster/ai_sapiens_rc_broadcaster_parameters.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/publisher.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace ai_sapiens_rc_broadcaster
{

class AiSapiensRcBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  controller_interface::CallbackReturn read_parameters();
  static float normalize_pwm(double pwm, double deadzone);
  static bool is_axis_in_deadzone(double pwm, double deadzone);

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::string sensor_name_;
  std::string input_topic_name_;
  std::string status_topic_name_;
  std::string frame_id_;
  std::vector<std::string> channels_;
  std::vector<std::string> channel_names_;
  std::vector<std::string> status_interfaces_;
  std::vector<std::string> state_interface_names_;
  std::vector<size_t> status_interface_indices_by_item_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  ordered_state_interfaces_;

  bool normalize_axes_{true};
  bool publish_buttons_{true};
  double deadzone_{50.0};
  double button_threshold_{1700.0};
  double input_publish_rate_{100.0};
  double status_publish_rate_{100.0};
  double watchdog_crsf_frame_timeout_ms_{100.0};
  double watchdog_realtime_tick_timeout_ms_{100.0};
  double min_crsf_link_quality_{1.0};
  rclcpp::Duration input_publish_period_{0, 0};
  rclcpp::Duration status_publish_period_{0, 0};
  int64_t input_publish_elapsed_ns_{0};
  int64_t status_publish_elapsed_ns_{0};
  int64_t realtime_tick_age_ns_{0};
  uint32_t last_realtime_tick_{0};
  bool has_last_realtime_tick_{false};
  bool active_{false};

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr input_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Joy>>
  realtime_input_publisher_;

  rclcpp::Publisher<ai_sapiens_interfaces::msg::RcStatus>::SharedPtr status_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      ai_sapiens_interfaces::msg::RcStatus>> realtime_status_publisher_;
};

}  // namespace ai_sapiens_rc_broadcaster

#endif  // AI_SAPIENS_RC_BROADCASTER__AI_SAPIENS_RC_BROADCASTER_HPP_
