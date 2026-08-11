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

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/controllers/mode_controller.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace mode_controller_test
{

constexpr uint16_t kReadyPoseInputCode = 2U;
constexpr uint16_t kVelocityInputCode = 3U;
constexpr uint16_t kMimicInputCode = 4U;
constexpr uint16_t kManualMimicServiceInputCode = 5U;
constexpr uint16_t kMimicSquatSelectorCode = 200U;

void ensure_rclcpp_initialized()
{
  static std::once_flag flag;
  std::call_once(
    flag, []() {
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
    });
}

std::filesystem::path write_root_config()
{
  static int index = 0;
  const auto path = std::filesystem::temp_directory_path() /
    ("ai_sapiens_sim2real_mode_controller_test_" + std::to_string(++index) + ".yaml");

  std::ofstream file(path);
  file <<
    R"(
robot_joint_order: [joint]
teleop_input:
  plugin: test_plugin
  config: test_plugin.yaml
teleop_conditions:
  DampingRequested:
    input_code: 1
  ReadyPoseRequested:
    input_code: 2
  VelocityRequested:
    input_code: 3
  ManualMimicServiceAllowed:
    input_code: 5
  MimicRequested:
    input_code: 4
selectors:
  mimic_selector:
    table:
      200: MimicSquat
authority:
  api_entry:
    warmup_duration: 0.0
    velocity_neutral_threshold: 0.05
    allowed_from_states: [Damping, ReadyPose, Velocity]
  default_velocity_state: Velocity
state_machine:
  initial: Damping
  states:
    Damping:
      run: damping
      transitions:
        - when: ReadyPoseRequested
          to: ReadyPose
    ReadyPose:
      run: ready_pose
      transitions:
        - when: DampingRequested
          to: Damping
        - when: VelocityRequested
          to: Velocity
        - when: MimicRequested
          select: mimic_selector
    Velocity:
      run: velocity_policy
      transitions:
        - when: DampingRequested
          to: Damping
        - when: ReadyPoseRequested
          to: ReadyPose
        - when: MimicRequested
          trigger: edge
          select: mimic_selector
    Mimic:
      abstract: true
      transitions:
        - when: DampingRequested
          to: Damping
        - when: ReadyPoseRequested
          to: ReadyPose
        - when: VelocityRequested
          to: Velocity
    MimicSquat:
      parent: Mimic
      run: mimic_run
state_behaviors:
  damping:
    kind: damping
    damping:
      values: [0.0]
  ready_pose:
    kind: posture
    duration: 0.01
    stiffness:
      values: [1.0]
    damping:
      values: [1.0]
    target_position:
      values: [0.0]
  velocity_policy:
    kind: policy
    asset: test/velocity
  mimic_run:
    kind: mimic
    asset: test/mimic
)";
  return path;
}

class ModeControllerFixture
{
public:
  ModeControllerFixture()
  {
    ensure_rclcpp_initialized();

    root_config_ = std::make_unique<ai_sapiens_sim2real::RootConfig>(write_root_config());

    shared_data_.joint_map.controller_joint_names = {"joint"};
    shared_data_.resize(1U, 1U);
    shared_data_.teleop.received.store(true);
    shared_data_.teleop.unavailable.store(false);
    shared_data_.teleop.input_code = kReadyPoseInputCode;
    shared_data_.sensors.projected_gravity = Eigen::Vector3f(0.0F, 0.0F, -1.0F);

    node_ = std::make_shared<rclcpp::Node>("mode_controller_test");
    controller_ = std::make_unique<ai_sapiens_sim2real::ModeController>(
      node_, *root_config_, &shared_data_);
  }

  void update_controller()
  {
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.001));
  }

  void release_startup_gate()
  {
    shared_data_.teleop.input_code = kReadyPoseInputCode;
    update_controller();
  }

  void set_api_heartbeat_valid(bool valid)
  {
    shared_data_.api.heartbeat_received.store(valid);
    shared_data_.api.heartbeat_stale.store(!valid);
  }

  bool enter_api()
  {
    release_startup_gate();
    if (controller_->status_snapshot().active_state != "ReadyPose") {
      return false;
    }

    set_api_heartbeat_valid(true);
    shared_data_.teleop.api_mode_requested = true;
    update_controller();
    if (std::string(controller_->status_snapshot().authority) != "API_WARMUP") {
      return false;
    }

    update_controller();
    return std::string(controller_->status_snapshot().authority) == "API";
  }

  bool enter_mimic_through_service()
  {
    if (!enter_api()) {
      return false;
    }

    const auto result = controller_->set_mode_by_name("MimicSquat");
    if (!result.success) {
      return false;
    }
    update_controller();
    return controller_->status_snapshot().active_state == "MimicSquat";
  }

  bool enter_manual_velocity_with_service_switch()
  {
    release_startup_gate();
    shared_data_.teleop.input_code = kVelocityInputCode;
    update_controller();
    if (controller_->status_snapshot().active_state != "Velocity") {
      return false;
    }

    shared_data_.teleop.input_code = kManualMimicServiceInputCode;
    update_controller();
    return std::string(controller_->status_snapshot().authority) == "MANUAL";
  }

  bool enter_manual_mimic_through_service()
  {
    if (!enter_manual_velocity_with_service_switch()) {
      return false;
    }

    const auto result = controller_->set_mode_by_name("MimicSquat");
    if (!result.success) {
      return false;
    }

    update_controller();
    return controller_->status_snapshot().active_state == "MimicSquat";
  }

  std::shared_ptr<rclcpp::Node> node_;
  ai_sapiens_sim2real::SharedControlData shared_data_;
  std::unique_ptr<ai_sapiens_sim2real::RootConfig> root_config_;
  std::unique_ptr<ai_sapiens_sim2real::ModeController> controller_;
};

}  // namespace mode_controller_test

using mode_controller_test::ModeControllerFixture;
using mode_controller_test::kManualMimicServiceInputCode;
using mode_controller_test::kMimicInputCode;
using mode_controller_test::kMimicSquatSelectorCode;
using mode_controller_test::kVelocityInputCode;

TEST(ModeController, TeleopTransitionRunsAfterStartupInputIsSafe)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();

  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();

  const auto status = fixture.controller_->status_snapshot();
  EXPECT_EQ(status.active_state, "Velocity");
  EXPECT_EQ(fixture.shared_data_.mode.active_policy_name, "velocity_policy");
}

TEST(ModeController, DampingRequestOverridesTeleopTransition)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  ASSERT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");

  fixture.shared_data_.requests.damping = true;
  fixture.update_controller();

  const auto status = fixture.controller_->status_snapshot();
  EXPECT_EQ(status.active_state, "Damping");
  EXPECT_STREQ(status.last_transition_reason, "damping_request");
}

TEST(ModeController, TeleopLossOverridesTheActiveState)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  ASSERT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");

  fixture.shared_data_.teleop.unavailable.store(true);
  fixture.update_controller();

  const auto status = fixture.controller_->status_snapshot();
  EXPECT_EQ(status.active_state, "Damping");
  EXPECT_STREQ(status.last_transition_reason, "teleop_input_timeout");
}

TEST(ModeController, ApiEntryRequiresValidHeartbeatAndFreshSwitchEdge)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.set_api_heartbeat_valid(false);

  fixture.shared_data_.teleop.api_mode_requested = true;
  fixture.update_controller();
  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");

  fixture.set_api_heartbeat_valid(true);
  fixture.update_controller();
  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");

  fixture.shared_data_.teleop.api_mode_requested = false;
  fixture.update_controller();
  fixture.shared_data_.teleop.api_mode_requested = true;
  fixture.update_controller();
  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "API_WARMUP");
}

TEST(ModeController, ApiEntryRejectsNonNeutralTeleopVelocity)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.set_api_heartbeat_valid(true);
  fixture.shared_data_.teleop.velocity_commands.x() = 0.1F;

  fixture.shared_data_.teleop.api_mode_requested = true;
  fixture.update_controller();

  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");
}

TEST(ModeController, ApiAcceptsMimicServiceRequest)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_mimic_through_service());

  const auto status = fixture.controller_->status_snapshot();
  EXPECT_STREQ(status.authority, "API");
  EXPECT_EQ(status.active_state, "MimicSquat");
  EXPECT_EQ(fixture.shared_data_.mode.active_policy_name, "mimic_run");
}

TEST(ModeController, ManualNeutralSwitchAcceptsOnlyMimicServiceRequests)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_manual_velocity_with_service_switch());

  const auto available = fixture.controller_->available_service_state_names();
  EXPECT_EQ(available, std::vector<std::string>({"MimicSquat"}));
  EXPECT_TRUE(fixture.controller_->status_snapshot().api_request_available);

  const auto velocity_result = fixture.controller_->set_mode_by_name("Velocity");
  EXPECT_FALSE(velocity_result.success);

  const auto mimic_result = fixture.controller_->set_mode_by_name("MimicSquat");
  ASSERT_TRUE(mimic_result.success) << mimic_result.message;
  fixture.update_controller();

  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "MimicSquat");
}

TEST(ModeController, ManualVelocitySwitchRejectsMimicServiceRequest)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();

  const auto result = fixture.controller_->set_mode_by_name("MimicSquat");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(fixture.controller_->status_snapshot().api_request_available);
  EXPECT_TRUE(fixture.controller_->available_service_state_names().empty());

  fixture.shared_data_.teleop.input_code = 0U;
  fixture.update_controller();
  const auto unmapped_input_result = fixture.controller_->set_mode_by_name("MimicSquat");

  EXPECT_FALSE(unmapped_input_result.success);
  EXPECT_FALSE(fixture.controller_->status_snapshot().api_request_available);
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
}

TEST(ModeController, ManualVelocitySwitchImmediatelyStopsServiceMimic)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_manual_mimic_through_service());

  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();

  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
  EXPECT_STREQ(
    fixture.controller_->status_snapshot().last_transition_reason,
    "teleop_input_condition");
}

TEST(ModeController, ManualMimicSwitchDoesNotRestartServiceMimic)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_manual_mimic_through_service());
  const auto transition_count = fixture.shared_data_.mode.transition_count;

  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.update_controller();

  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "MimicSquat");
  EXPECT_EQ(fixture.shared_data_.mode.transition_count, transition_count);
}

TEST(ModeController, CompletedServiceMimicNeedsFreshManualMimicEdge)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_manual_mimic_through_service());

  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.update_controller();
  fixture.shared_data_.requests.state_name = "Velocity";
  fixture.update_controller();
  ASSERT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");

  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");

  fixture.shared_data_.teleop.input_code = kManualMimicServiceInputCode;
  fixture.update_controller();
  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.update_controller();

  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "MimicSquat");
  EXPECT_STREQ(
    fixture.controller_->status_snapshot().last_transition_reason,
    "teleop_input_condition");
}

TEST(ModeController, ManualVelocityOverrideDiscardsPendingMimicServiceRequest)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_manual_velocity_with_service_switch());
  ASSERT_TRUE(fixture.controller_->set_mode_by_name("MimicSquat").success);

  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");

  fixture.shared_data_.teleop.input_code = kManualMimicServiceInputCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
}

TEST(ModeController, MimicCompletionRequestReturnsToVelocity)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_mimic_through_service());

  fixture.shared_data_.requests.state_name = "Velocity";
  fixture.update_controller();

  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
  EXPECT_TRUE(fixture.shared_data_.requests.state_name.empty());
}

TEST(ModeController, MimicInputBlocksApiReleaseUntilNonMimicHandoff)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_mimic_through_service());

  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.shared_data_.teleop.api_mode_requested = false;
  fixture.update_controller();

  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "API");
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "MimicSquat");

  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();

  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
}

TEST(ModeController, HeartbeatLossWinsOverReleaseAndStopsMimic)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_mimic_through_service());

  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.shared_data_.teleop.api_mode_requested = false;
  fixture.set_api_heartbeat_valid(false);
  fixture.update_controller();

  const auto status = fixture.controller_->status_snapshot();
  EXPECT_STREQ(status.authority, "MANUAL");
  EXPECT_EQ(status.active_state, "Velocity");
  EXPECT_STREQ(status.last_transition_reason, "api_authority_lost");
  EXPECT_TRUE(fixture.shared_data_.mode.velocity_commands.isZero());
}
