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
constexpr uint16_t kMimicSquatSelectorCode = 200U;

std::filesystem::path write_root_config(
  bool prefer_teleop = true, const std::string & initial = "Damping")
{
  static int index = 0;
  const auto path = std::filesystem::temp_directory_path() /
    ("ai_sapiens_sim2real_mode_controller_test_" + std::to_string(++index) + ".yaml");
  const auto motion_path = path.string() + ".csv";
  std::ofstream(motion_path) << "0,0,0,0,0,0,1,0\n0,0,0,0,0,0,1,0\n";

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
  MimicRequested:
    input_code: 4
selectors:
  mimic_selector:
    table:
      200: MimicSquat
authority:
)";
  file << "  teleop_overrides_cmd_vel: " << std::boolalpha << prefer_teleop;
  file <<
    R"(
  api_entry:
    warmup_duration: 0.0
    velocity_neutral_threshold: 0.05
    allowed_from_states: [Damping, ReadyPose, Velocity]
  default_velocity_state: Velocity
state_machine:
)";
  file << "  initial: " << initial << '\n';
  file              <<
    R"(  damping_state: Damping
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
      values: [3.5]
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
    policy_path: )"
                    << path.string() << R"(
    sim2real_yaml_path: )" << path.string() <<
    R"(
  mimic_run:
    kind: mimic
    asset: test/mimic
    policy_path: )" << path.string() << R"(
    sim2real_yaml_path: )" << path.string() << R"(
    motion_file: )" << motion_path << R"(
)";
  return path;
}

class ModeControllerFixture
{
public:
  explicit ModeControllerFixture(
    bool require_heartbeat = true, bool prefer_teleop = true,
    const std::string & initial = "Damping")
  {
    root_config_ =
      std::make_unique<ai_sapiens_sim2real::RootConfig>(write_root_config(prefer_teleop, initial));

    shared_data_.joint_map.controller_joint_names = {"joint"};
    shared_data_.resize(1U, 1U);
    shared_data_.teleop.is_velocity_command_valid = true;
    shared_data_.teleop.received.store(true);
    shared_data_.teleop.unavailable.store(false);
    shared_data_.teleop.input_code = kReadyPoseInputCode;
    shared_data_.sensors.projected_gravity = Eigen::Vector3f(0.0F, 0.0F, -1.0F);
    shared_data_.sensors.is_imu_usable = true;

    node_ = std::make_shared<rclcpp::Node>("mode_controller_test");
    controller_ = std::make_unique<ai_sapiens_sim2real::ModeController>(
      node_, *root_config_, &shared_data_, require_heartbeat);
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

  std::shared_ptr<rclcpp::Node> node_;
  ai_sapiens_sim2real::SharedControlData shared_data_;
  std::unique_ptr<ai_sapiens_sim2real::RootConfig> root_config_;
  std::unique_ptr<ai_sapiens_sim2real::ModeController> controller_;
};

}  // namespace mode_controller_test

using mode_controller_test::ModeControllerFixture;
using mode_controller_test::kMimicInputCode;
using mode_controller_test::kMimicSquatSelectorCode;
using mode_controller_test::kVelocityInputCode;

TEST(RejectedRequestGate, BlocksUntilExplicitlyRearmed)
{
  ai_sapiens_sim2real::RejectedRequestGate gate;
  EXPECT_FALSE(gate.blocked());
  gate.reject(kMimicInputCode);
  EXPECT_TRUE(gate.blocked());
  gate.reject(kMimicInputCode);
  EXPECT_TRUE(gate.blocked());
  gate.observe(true, kMimicInputCode);
  EXPECT_TRUE(gate.blocked());
  gate.observe(false, kVelocityInputCode);
  EXPECT_TRUE(gate.blocked());
  gate.observe(true, kMimicInputCode);
  EXPECT_TRUE(gate.blocked());
  gate.observe(true, kVelocityInputCode);
  EXPECT_FALSE(gate.blocked());
}

TEST(MimicBehavior, MissingMotionSourceIsRejectedExplicitly)
{
  const ai_sapiens_sim2real::MimicBehavior mimic;
  EXPECT_THROW(mimic.require_motion_source(), std::logic_error);
}

TEST(MotionSource, RootConfigRetainsSourcesAcrossConsumers)
{
  ai_sapiens_sim2real::RootConfig config(mode_controller_test::write_root_config());
  const auto first = config.policy_behaviors();
  const auto second = config.policy_behaviors();
  ASSERT_EQ(first.size(), second.size());
  for (size_t i = 0; i < first.size(); ++i) {
    if (first[i].mimic) {
      ASSERT_TRUE(second[i].mimic);
      ASSERT_NE(first[i].mimic->source, nullptr);
      EXPECT_EQ(first[i].mimic->source, second[i].mimic->source);
      EXPECT_EQ(&first[i].mimic->require_motion_source(), first[i].mimic->source.get());
    }
  }
}

TEST(RootConfig, PolicyTransitionsInheritAndOverrideIndividualFields)
{
  const auto path = mode_controller_test::write_root_config();
  auto document = YAML::LoadFile(path.string());
  auto save = [&]() {std::ofstream(path) << document;};
  auto expect = [&](const std::string & name, bool enabled, double duration) {
      const ai_sapiens_sim2real::RootConfig config(path);
      bool found = false;
      for (const auto & behavior : config.policy_behaviors()) {
        if (behavior.name == name) {
          found = true;
          EXPECT_EQ(behavior.action_transition.enabled, enabled);
          EXPECT_DOUBLE_EQ(behavior.action_transition.duration, duration);
        }
      }
      EXPECT_TRUE(found);
    };
  expect("velocity_policy", false, 0.2);
  expect("mimic_run", false, 0.2);
  document["policy_action_transition"] = YAML::Load("{enabled: true, duration: 0.15}");
  save();
  expect("velocity_policy", true, 0.15);
  expect("mimic_run", true, 0.15);
  document["state_behaviors"]["velocity_policy"]["policy_action_transition"] =
    YAML::Load("{duration: 0.4}");
  document["state_behaviors"]["mimic_run"]["policy_action_transition"] =
    YAML::Load("{enabled: false}");
  save();
  expect("velocity_policy", true, 0.4);
  expect("mimic_run", false, 0.15);
  document["policy_action_transition"]["enabled"] = false;
  document["state_behaviors"]["mimic_run"]["policy_action_transition"] =
    YAML::Load("{enabled: true, duration: 0.0}");
  save();
  expect("velocity_policy", false, 0.4);
  expect("mimic_run", true, 0.0);
}

TEST(RootConfig, RejectsInvalidPolicyTransitionOverrides)
{
  for (const auto & name : {"velocity_policy", "mimic_run"}) {
    for (const auto & value : {"{enabled: invalid}", "{duration: -0.1}",
        "{duration: .nan}", "{duration: .inf}", "{duration: invalid}", "false"})
    {
      const auto path = mode_controller_test::write_root_config();
      auto document = YAML::LoadFile(path.string());
      document["state_behaviors"][name]["policy_action_transition"] = YAML::Load(value);
      std::ofstream(path) << document;
      const ai_sapiens_sim2real::RootConfig config(path);
      try {
        config.policy_behaviors();
        FAIL() << "Expected invalid transition override to be rejected";
      } catch (const std::runtime_error & error) {
        EXPECT_NE(std::string(error.what()).find(
            std::string("state_behaviors.") + name + ".policy_action_transition"),
          std::string::npos);
      }
    }
  }
}

TEST(ModeController, WrongMimicTiltKeepsCurrentState)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.shared_data_.sensors.orientation = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitY()));
  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "ReadyPose");
}

TEST(ModeController, RejectedLevelMimicNeedsOperatorReleaseEvenAfterPoseRecovers)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.shared_data_.sensors.orientation = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitY()));
  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.update_controller();
  ASSERT_EQ(fixture.controller_->status_snapshot().active_state, "ReadyPose");
  fixture.shared_data_.sensors.orientation = Eigen::Quaternionf::Identity();
  for (int i = 0; i < 5; ++i) {
    fixture.update_controller();
    EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "ReadyPose");
  }
  // Changing only the selector must not rearm.
  fixture.shared_data_.teleop.selector_code = 201;
  fixture.update_controller();
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "ReadyPose");
  fixture.shared_data_.teleop.input_code = mode_controller_test::kReadyPoseInputCode;
  fixture.update_controller();
  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "MimicSquat");
}

TEST(ModeController, ReleasedMimicInputAllowsVelocityAndDamping)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.shared_data_.sensors.orientation = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitY()));
  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.update_controller();
  ASSERT_EQ(fixture.controller_->status_snapshot().active_state, "ReadyPose");
  fixture.shared_data_.sensors.orientation = Eigen::Quaternionf::Identity();
  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
  fixture.shared_data_.requests.damping = true;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Damping");
  fixture.shared_data_.teleop.input_code = mode_controller_test::kReadyPoseInputCode;
  fixture.update_controller();
  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
}

TEST(ModeController, VelocityCanRetryMimicAfterReturningToVelocityInput)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  fixture.shared_data_.sensors.orientation = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitY()));
  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.shared_data_.teleop.selector_code = kMimicSquatSelectorCode;
  fixture.update_controller();
  ASSERT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
  fixture.shared_data_.sensors.orientation = Eigen::Quaternionf::Identity();
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  fixture.shared_data_.teleop.input_code = kMimicInputCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "MimicSquat");
}

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

TEST(ModeController, EmergencyCommandUsesConfiguredGainsWithoutChangingState)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  auto & data = fixture.shared_data_;
  ASSERT_EQ(data.mode.active_state_name, "ReadyPose");
  ASSERT_FLOAT_EQ(data.output.command.damping[0], 1.0f);
  data.sensors.joint_pos[0] = 1.25f;
  data.output.feedforward[0] = 2.0f;
  data.output.position_limits[0] = std::make_pair(-0.5f, 0.5f);
  const auto transitions = data.mode.transition_count;

  fixture.controller_->apply_emergency_damping();

  EXPECT_EQ(data.mode.active_state_name, "ReadyPose");
  EXPECT_EQ(data.mode.transition_count, transitions);
  EXPECT_FLOAT_EQ(data.output.command.stiffness[0], 0.0f);
  EXPECT_FLOAT_EQ(data.output.command.damping[0], 3.5f);
  EXPECT_FLOAT_EQ(data.output.command.position[0], 1.25f);
  EXPECT_FLOAT_EQ(data.output.feedforward[0], 0.0f);
  EXPECT_FALSE(data.output.position_limits[0]);
  EXPECT_FALSE(data.output.has_published_command);
}

TEST(ModeController, ReadyPoseStartupUsesDampingGainsForEmergencyAndFailsafe)
{
  ModeControllerFixture fixture(true, true, "ReadyPose");
  auto & data = fixture.shared_data_;
  ASSERT_EQ(data.mode.active_state_name, "ReadyPose");
  fixture.controller_->reset();
  ASSERT_EQ(data.mode.active_state_name, "ReadyPose");
  ASSERT_FLOAT_EQ(data.output.command.stiffness[0], 1.0f);

  fixture.controller_->apply_emergency_damping();
  EXPECT_EQ(data.mode.active_state_name, "ReadyPose");
  EXPECT_FLOAT_EQ(data.output.command.stiffness[0], 0.0f);
  EXPECT_FLOAT_EQ(data.output.command.damping[0], 3.5f);

  data.requests.damping = true;
  fixture.update_controller();
  EXPECT_EQ(data.mode.active_state_name, "Damping");
  EXPECT_FLOAT_EQ(data.output.command.stiffness[0], 0.0f);
  EXPECT_FLOAT_EQ(data.output.command.damping[0], 3.5f);
}

TEST(ModeController, LatchedCommandPublishFaultRevokesApiAndKeepsDamping)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_mimic_through_service());
  fixture.shared_data_.output.is_command_publish_fault_latched = true;
  fixture.update_controller();
  const auto status = fixture.controller_->status_snapshot();
  EXPECT_EQ(status.active_state, "Damping");
  EXPECT_STREQ(status.authority, "MANUAL");
  EXPECT_STREQ(status.last_transition_reason, "command_publish_fault");

  fixture.shared_data_.teleop.input_code = kVelocityInputCode;
  fixture.update_controller();
  EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Damping");
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

TEST(ModeController, OptionalHeartbeatAllowsServiceWithNoHeartbeat)
{
  ModeControllerFixture fixture(false);
  fixture.release_startup_gate();
  fixture.shared_data_.teleop.api_mode_requested = true;
  fixture.update_controller();
  fixture.update_controller();

  const auto status = fixture.controller_->status_snapshot();
  EXPECT_STREQ(status.authority, "API");
  EXPECT_FALSE(status.api_heartbeat_valid);
  EXPECT_TRUE(status.api_request_available);
  EXPECT_TRUE(fixture.controller_->set_mode_by_name("MimicSquat").success);
}

TEST(ModeController, FreshNonNeutralApiCommandBlocksEntryButExpiredCommandDoesNot)
{
  ModeControllerFixture fixture;
  fixture.release_startup_gate();
  fixture.set_api_heartbeat_valid(true);
  auto & data = fixture.shared_data_;
  data.api.velocity_commands.x() = 0.3F;
  data.api.velocity_received_at = std::chrono::steady_clock::now() - std::chrono::milliseconds(650);
  data.teleop.api_mode_requested = true;
  fixture.update_controller();
  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");

  data.teleop.api_mode_requested = false;
  fixture.update_controller();
  data.api.velocity_received_at -= std::chrono::seconds(2);
  data.teleop.api_mode_requested = true;
  fixture.update_controller();
  fixture.update_controller();
  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "API");
  EXPECT_TRUE(data.mode.velocity_commands.isZero());
}

TEST(ModeController, StickOverrideSurvivesInvalidInputUntilValidNeutral)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_api());
  auto & data = fixture.shared_data_;
  data.api.velocity_commands.x() = 0.3F;
  data.api.velocity_received_at = std::chrono::steady_clock::now();
  fixture.update_controller();
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), 0.3F);

  data.teleop.velocity_command_normalized.y() = -0.4F;
  data.teleop.velocity_commands.y() = -0.02F;
  fixture.update_controller();
  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "API");
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), 0.0F);
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.y(), -0.02F);

  data.api.velocity_commands.x() = 0.6F;
  data.api.velocity_received_at = std::chrono::steady_clock::now();
  data.teleop.is_velocity_command_valid = false;
  data.teleop.velocity_commands.setZero();
  data.teleop.velocity_command_normalized.setZero();
  fixture.update_controller();
  fixture.update_controller();
  EXPECT_TRUE(data.mode.velocity_commands.isZero());

  data.teleop.is_velocity_command_valid = true;
  fixture.update_controller();
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), 0.6F);
}

TEST(ModeController, FreshCommandFromPreviousSessionCannotDriveAfterReentry)
{
  ModeControllerFixture fixture;
  ASSERT_TRUE(fixture.enter_api());
  auto & data = fixture.shared_data_;
  data.api.velocity_commands.x() = 0.03F;
  data.api.velocity_received_at = std::chrono::steady_clock::now();
  fixture.update_controller();
  ASSERT_FLOAT_EQ(data.mode.velocity_commands.x(), 0.03F);

  data.teleop.api_mode_requested = false;
  fixture.update_controller();
  ASSERT_STREQ(fixture.controller_->status_snapshot().authority, "MANUAL");
  ASSERT_TRUE(fixture.enter_api());
  EXPECT_TRUE(data.mode.velocity_commands.isZero());

  data.api.velocity_received_at = std::chrono::steady_clock::now();
  fixture.update_controller();
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), 0.03F);
}

TEST(ModeController, DisabledStickOverrideKeepsApiVelocityAndStopsOnTimeout)
{
  ModeControllerFixture fixture(true, false);
  ASSERT_TRUE(fixture.enter_api());
  auto & data = fixture.shared_data_;
  data.api.velocity_commands.x() = 0.3F;
  data.api.velocity_received_at = std::chrono::steady_clock::now();
  data.teleop.velocity_command_normalized.y() = 0.4F;
  data.teleop.velocity_commands.y() = 0.4F;
  fixture.update_controller();
  EXPECT_STREQ(fixture.controller_->status_snapshot().authority, "API");
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.x(), 0.3F);
  EXPECT_FLOAT_EQ(data.mode.velocity_commands.y(), 0.0F);

  data.api.velocity_received_at -= std::chrono::seconds(2);
  fixture.update_controller();
  EXPECT_TRUE(data.mode.velocity_commands.isZero());
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

TEST(PolicyEntryValidator, ConfiguredStartTimeSelectsEntryOrientation)
{
  const auto path = mode_controller_test::write_root_config();
  std::ofstream(path.string() + ".csv") <<
    "0,0,0,0,0,0,1,0\n0,0,0,0,0.70710678,0,0.70710678,0\n";
  std::ofstream(path, std::ios::app) << "    time_start: 0.02\n";
  ai_sapiens_sim2real::RootConfig config(path);
  const auto validator = ai_sapiens_sim2real::PolicyEntryValidator::from_config(config);
  ai_sapiens_sim2real::StateBehavior target;
  target.mimic = true;
  target.policy_name = "mimic_run";
  EXPECT_FALSE(validator.evaluate(target, Eigen::Quaternionf::Identity(), true).allowed());
  const Eigen::Quaternionf lying(Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitY()));
  EXPECT_TRUE(validator.evaluate(target, lying, true).allowed());
  std::filesystem::remove(path);
  std::filesystem::remove(path.string() + ".csv");
}

TEST(ModeController, LyingOrientationAllowsReadyPoseToVelocity)
{
  for (float pitch : {1.5707963f, -1.5707963f}) {
    ModeControllerFixture fixture;
    fixture.release_startup_gate();
    fixture.shared_data_.sensors.orientation = Eigen::Quaternionf(
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    fixture.shared_data_.sensors.compute_projected_gravity();
    fixture.shared_data_.teleop.input_code = kVelocityInputCode;
    fixture.update_controller();
    EXPECT_EQ(fixture.controller_->status_snapshot().active_state, "Velocity");
  }
}
