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
#include <cstdlib>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>

#include "ai_sapiens_sim2real/command_transition.hpp"
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_diagnostics.hpp"
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_recorder.hpp"
#include "ai_sapiens_sim2real/policy/diagnostics/policy_diagnostics.hpp"
#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"
#include "ros_log_capture.hpp"

using ai_sapiens_sim2real::ActionFaultRecorder;
using ai_sapiens_sim2real::ActionPipeline;
using ai_sapiens_sim2real::ActionProperties;
using ai_sapiens_sim2real::ActionRejection;
using ai_sapiens_sim2real::ActionSafetyLimits;
using ai_sapiens_sim2real::BoundaryCheckState;
using ai_sapiens_sim2real::CommandTransition;
using ai_sapiens_sim2real::FaultPolicyMetadata;
using ai_sapiens_sim2real::MotionReference;
using ai_sapiens_sim2real::PolicyDiagnostics;
using ai_sapiens_sim2real::PolicyRuntime;
using ai_sapiens_sim2real::PolicyUpdateResult;
using ai_sapiens_sim2real::SensorData;
using ai_sapiens_sim2real::SharedControlData;
using ai_sapiens_sim2real::Sim2RealConfig;
using ai_sapiens_sim2real::format_action_fault_actions;
using ai_sapiens_sim2real::format_action_fault_sensors;
using ai_sapiens_sim2real::validate_action;
using ai_sapiens_sim2real::test::RosLogCapture;

namespace
{
std::string normalized_whitespace(const std::string & value)
{
  std::istringstream input(value);
  std::string word, result;
  while (input >> word) {
    if (!result.empty()) {result += " ";}
    result += word;
  }
  return result;
}

class PolicyActionFault : public ::testing::Test
{
protected:
  virtual bool uses_motion_reference() const {return false;}
  virtual std::filesystem::path policy_directory() const {return COMMON_POLICY_TEST_DATA_DIR;}

  void SetUp() override
  {
    const auto node = std::make_shared<rclcpp::Node>("policy_action_fault_test");
    const auto directory = policy_directory();
    const Sim2RealConfig config(directory / "sim2real.yaml");
    data.resize(2, 2);
    data.sensors.joint_pos << .4f, -.7f;
    data.sensors.is_imu_usable = true;
    if (uses_motion_reference()) {
      const auto motion = std::filesystem::path(__FILE__).parent_path() / "data/common/motion.csv";
      reference = std::make_unique<MotionReference>(motion.string(), 50.f,
        std::vector<std::string>{"left_hip_roll_joint", "right_hip_roll_joint", "waist_yaw_joint"});
    }
    runtime = std::make_unique<PolicyRuntime>(node, "fault_test",
        (directory / "policy.onnx").string(), config, config.policy_joints(), &data,
        reference.get());
    runtime->enter();
  }

  PolicyUpdateResult tick(double seconds = .02)
  {
    return runtime->update(rclcpp::Duration::from_seconds(seconds));
  }

  SharedControlData data;
  std::unique_ptr<MotionReference> reference;
  std::unique_ptr<PolicyRuntime> runtime;
};

class MimicActionFault : public PolicyActionFault
{
protected:
  bool uses_motion_reference() const override {return true;}
};
}  // namespace

TEST_F(PolicyActionFault, HoldsOutputAndRetriesOnFiveSeparatePolicyTicks)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto good_action = data.policy.last_action;
  const auto good_target = runtime->target_command();
  auto output = good_target;
  CommandTransition transition(2);
  transition.begin(output, good_target, {0, 1}, 0, output);
  transition.update(.02, PolicyUpdateResult::TargetReady, good_target, output);

  data.sensors.joint_pos[0] = 20.0f;
  for (int attempt = 1; attempt <= 6; ++attempt) {
    if (attempt > 1) {
      ASSERT_EQ(tick(.005), PolicyUpdateResult::Skipped);
      EXPECT_FALSE(data.requests.action_limit_exceeded);
    }
    const auto result = tick(attempt == 1 ? .02 : .015);
    ASSERT_EQ(result, PolicyUpdateResult::TargetUnavailable);
    transition.update(.02, result, runtime->target_command(), output);
    EXPECT_EQ(runtime->target_command().position, good_target.position);
    EXPECT_EQ(output.position, good_target.position);
    EXPECT_EQ(data.policy.last_action, good_action);
    EXPECT_EQ(data.requests.action_limit_exceeded, attempt == 6);
    EXPECT_NEAR(data.policy.episode_time, (attempt + 1) * .02, 1e-6);
  }
}

TEST_F(PolicyActionFault, FreshSensorsRecoverAndResetTheFailureBudget)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  for (int failures : {1, 5}) {
    data.sensors.joint_pos[0] = 20.0f;
    for (int i = 0; i < failures; ++i) {
      ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
      EXPECT_FALSE(data.requests.action_limit_exceeded);
    }
    data.sensors.joint_pos[0] = .6f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_NEAR(runtime->target_command().position[0], .35f, 1e-6);
  }
  data.sensors.joint_pos[0] = 20.0f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
  EXPECT_FALSE(data.requests.action_limit_exceeded);
}

TEST_F(PolicyActionFault, ReentryResetsRetries)
{
  data.sensors.joint_pos[0] = 20.0f;
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetUnavailable);
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetUnavailable);
  EXPECT_FALSE(data.requests.action_limit_exceeded);
}

TEST_F(PolicyActionFault, NonfiniteActionsShareTheSixFailureBudget)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto target = runtime->target_command().position;
  const auto action = data.policy.last_action;
  for (int attempt = 1; attempt <= 6; ++attempt) {
    data.sensors.joint_pos[0] = attempt % 2 == 0 ? 20.0f :
      std::numeric_limits<float>::quiet_NaN();
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(data.requests.action_limit_exceeded, attempt == 6);
    EXPECT_EQ(runtime->target_command().position, target);
    EXPECT_EQ(data.policy.last_action, action);
  }
}

TEST_F(PolicyActionFault, RecoversAfterNonfiniteAction)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()})
  {
    data.sensors.joint_pos[0] = invalid;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    data.sensors.joint_pos[0] = .4f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
  }
}

TEST(ActionFaultDiagnostics, HighlightsJumpsWithPreviousValuesAndUnits)
{
  SensorData previous, current;
  previous.joint_pos = Eigen::VectorXf::Zero(1);
  previous.joint_vel = Eigen::VectorXf::Zero(1);
  current = previous;
  current.joint_pos[0] = 1;
  current.joint_vel[0] = 10;
  current.angular_velocity.x() = 10;
  const auto log = normalized_whitespace(
    format_action_fault_sensors(current, previous, {"knee"}, .02));
  EXPECT_NE(log.find("dt=0.020"), std::string::npos);
  EXPECT_NE(log.find("knee 57.296 0.000 +57.296 [JUMP!]"), std::string::npos);
  EXPECT_NE(log.find("knee 572.958 0.000 +572.958 [JUMP!]"), std::string::npos);
  EXPECT_NE(log.find("wx 572.958 0.000 +572.958 [JUMP!]"), std::string::npos);
  const auto slow = format_action_fault_sensors(current, previous, {"knee"}, .1);
  EXPECT_EQ(slow.find("[JUMP!]"), std::string::npos);
}

TEST(ActionFaultDiagnostics, WrapsYawAndHandlesMissingBaselineAndInvalidValues)
{
  constexpr float rad = 3.141592653589793f / 180;
  SensorData previous, current;
  previous.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(179 * rad, Eigen::Vector3f::UnitZ()));
  current.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(-179 * rad, Eigen::Vector3f::UnitZ()));
  const auto wrapped = normalized_whitespace(
    format_action_fault_sensors(current, previous, {}, .02));
  EXPECT_NE(wrapped.find("yaw -179.000 179.000 +2.000"), std::string::npos);
  EXPECT_EQ(wrapped.find("[JUMP!]"), std::string::npos);
  current.angular_velocity.y() = std::numeric_limits<float>::quiet_NaN();
  const auto initial = normalized_whitespace(
    format_action_fault_sensors(current, std::nullopt, {"absent"}, 0));
  EXPECT_NE(initial.find("n/a n/a"), std::string::npos);
  EXPECT_NE(initial.find("[NONFINITE]"), std::string::npos);
  EXPECT_NE(initial.find("absent [MISSING]"), std::string::npos);
}

namespace
{
class PolicyHistoryFault : public PolicyActionFault
{
protected:
  std::filesystem::path policy_directory() const override {return HISTORY_POLICY_TEST_DATA_DIR;}
};
}  // namespace

TEST_F(PolicyHistoryFault, FourRejectedSpikesRecoverOnTheFifthPolicyTick)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto normal_target = runtime->target_command().position;
  for (int i = 0; i < 4; ++i) {
    data.sensors.joint_pos[0] = 200.4f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_GT(data.policy.input[6], 200.0f);  // Keep failed input visible in diagnostics.
  }
  data.sensors.joint_pos[0] = .4f;
  // The model sums all four frames. Without rejection cleanup, the same
  // spike causes three more failures even though physical feedback is normal.
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_EQ(runtime->target_command().position, normal_target);
    for (size_t frame = 0; frame < 4; ++frame) {
      EXPECT_FLOAT_EQ(data.policy.input[frame * 2], .4f);
      EXPECT_FLOAT_EQ(data.policy.input[frame * 2 + 1], -.7f);
    }
  }
}

TEST_F(MimicActionFault, CapturesRejectedInputBeforeHistoryRepairAndRecoverySeparately)
{
  bool has_recorded_pair = false;
  // The production queue intentionally drops records on lock contention. Retry
  // the isolated scenario a bounded number of times; persistent loss still fails.
  for (int trial = 0; trial < 5; ++trial) {
    runtime->enter();
    std::string pattern = "/tmp/policy-fault-capture-XXXXXX";
    ASSERT_NE(::mkdtemp(pattern.data()), nullptr);
    ActionFaultRecorder::Options options;
    options.directory = pattern;
    auto recorder = std::make_shared<ActionFaultRecorder>(options,
      rclcpp::get_logger("capture_test"));
    const auto session = recorder->session_directory();
    runtime->set_fault_recorder(recorder);
    ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
    const auto accepted = data.policy.input;
    // Raw action exceeds 10 while the target stays inside the absolute limit.
    data.sensors.joint_pos[0] = 12.f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    const auto rejected = data.policy.input;
    data.sensors.joint_pos[0] = .4f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    const auto recovered = data.policy.input;
    runtime->set_fault_recorder(nullptr);
    recorder.reset();
    if (!std::filesystem::exists(session / "incident-1-attempt-1.yaml") ||
      !std::filesystem::exists(session / "incident-1-attempt-2-recovered.yaml"))
    {
      std::filesystem::remove_all(pattern);
      continue;
    }
    const auto failure = YAML::LoadFile((session / "incident-1-attempt-1.yaml").string());
    const auto recovery = YAML::LoadFile((session /
        "incident-1-attempt-2-recovered.yaml").string());
    const auto tensor = [](const YAML::Node & node) {
        return node["onnx_calls"][0]["inputs"][0]["raw_tensor"].as<std::vector<float>>();
      };
    EXPECT_EQ(tensor(failure["previous_accepted_inference"]), accepted);
    EXPECT_EQ(tensor(failure["current_inference"]), rejected);
    EXPECT_EQ(tensor(recovery["current_inference"]), recovered);
    EXPECT_EQ(recovery["current_inference"]["outcome"].as<std::string>(), "ACCEPTED");
    EXPECT_FALSE(recovery["previous_accepted_inference"]);
    EXPECT_NE(failure["diagnostics"].as<std::string>().find("[CLIP+DELTA]"), std::string::npos);
    EXPECT_EQ(failure["current_inference"]["action_reference"].as<std::vector<float>>(),
      failure["previous_accepted_inference"]["processed_action"].as<std::vector<float>>());
    EXPECT_NE(failure["diagnostics"].as<std::string>().find("change_limit="), std::string::npos);
    EXPECT_NE(failure["diagnostics"].as<std::string>().find("[JUMP!]"), std::string::npos);
    std::filesystem::remove_all(pattern);
    has_recorded_pair = true;
    break;
  }
  EXPECT_TRUE(has_recorded_pair);
}

TEST(PolicyDiagnostics, DefersCaptureErrorLogUntilFlush)
{
  std::string pattern = "/tmp/policy-diagnostics-flush-XXXXXX";
  ASSERT_NE(::mkdtemp(pattern.data()), nullptr);

  const auto logger = rclcpp::get_logger("deferred_capture_error_test");
  {
    ActionFaultRecorder::Options options;
    options.directory = pattern;
    auto recorder = std::make_shared<ActionFaultRecorder>(options, logger);
    FaultPolicyMetadata metadata;
    metadata.config_path = "/dev/null";
    PolicyDiagnostics diagnostics(std::move(metadata), logger);
    diagnostics.set_recorder(std::move(recorder));
    RosLogCapture logs(logger.get_name());

    diagnostics.capture_output({1.0f});
    EXPECT_TRUE(logs.take().empty());

    diagnostics.flush_diagnostics();
    EXPECT_NE(logs.take().find("Observation capture disabled"), std::string::npos);
  }

  std::filesystem::remove_all(pattern);
}

TEST(ActionFaultDiagnostics, ConsoleIncludesOnlyInvalidActionsAndJumpRows)
{
  const std::vector<float> values{7.f, 0.f, std::numeric_limits<float>::quiet_NaN()};
  const auto validation = validate_action(values, values, nullptr, ActionSafetyLimits{});
  const auto actions = format_action_fault_actions(values,
      {"limit_joint", "normal_joint", "nan_joint"}, validation, true);
  EXPECT_NE(actions.find("limit"), std::string::npos);
  EXPECT_NE(actions.find("nan"), std::string::npos);
  EXPECT_EQ(actions.find("normal"), std::string::npos);
  SensorData previous, current;
  previous.joint_pos = Eigen::VectorXf::Zero(2);
  previous.joint_vel = Eigen::VectorXf::Zero(2);
  current = previous;
  current.joint_pos[0] = 1.f;
  const auto sensors = format_action_fault_sensors(current, previous,
      {"jump_joint", "normal_joint"}, .02, true);
  EXPECT_NE(sensors.find("jump"), std::string::npos);
  EXPECT_EQ(sensors.find("normal"), std::string::npos);
  EXPECT_EQ(sensors.find("IMU"), std::string::npos);
  EXPECT_EQ(sensors.find("Joint velocity"), std::string::npos);
}

TEST_F(MimicActionFault, RejectsLargeChangeInsideAbsoluteLimitAndRetainsReference)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto accepted = runtime->target_command().position;
  const auto last_action = data.policy.last_action;
  data.sensors.joint_pos[0] = 12.f;  // raw 11.9; target 6.05; delta 5.8 rad.
  for (int attempt = 0; attempt < 5; ++attempt) {
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(runtime->target_command().position, accepted);
    EXPECT_EQ(data.policy.last_action, last_action);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
  }
  data.sensors.joint_pos[0] = .6f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], .35f, 1e-6);
  // Recovery resets the common retry budget; rejected targets never become its reference.
  data.sensors.joint_pos[0] = 12.2f;
  for (int attempt = 1; attempt <= 6; ++attempt) {
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(data.requests.action_limit_exceeded, attempt == 6);
  }
}

TEST_F(MimicActionFault, RejectsAtClipWhenDeltaCrosses90Degrees)
{
  constexpr float limit = 3.14159265f / 2.f;
  constexpr float target = 5.1f;  // raw 10, scale .5, offset .1
  data.output.has_published_command = true;
  data.sensors.joint_pos[0] = 10.1f;
  data.output.last_published.position = {target - limit, -.325f};
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetUnavailable);
  data.output.last_published.position[0] = std::nextafter(target - limit, target);
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  EXPECT_FLOAT_EQ(runtime->target_command().position[0], target);
}

TEST_F(MimicActionFault, EntryUsesPublishedCommandAndReentryReseedsIt)
{
  data.output.has_published_command = true;
  data.output.last_published.position = {6.05f, -.325f};
  data.output.command.position = {0.f, 0.f};  // may differ during transitions/position limiting
  data.sensors.joint_pos[0] = 12.f;
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  data.output.last_published.position[0] = .25f;
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetUnavailable);
}

TEST(ActionFaultDiagnostics, DeltaFaultIsShownEvenInsideAbsoluteLimit)
{
  const std::vector<float> values{3.55f, .3f}, reference{.25f, .2f};
  const auto validation = validate_action({10.f, 0.f}, values, &reference, ActionSafetyLimits{});
  const auto report = normalized_whitespace(format_action_fault_actions(
    values, {"left_knee_joint", "right_knee_joint"}, validation, true));
  EXPECT_NE(report.find("L.knee 3.550 0.250 +3.300 [CLIP+DELTA]"), std::string::npos);
  EXPECT_EQ(report.find("R.knee"), std::string::npos);
  EXPECT_EQ(report.find("[LIMIT]"), std::string::npos);
}

TEST_F(PolicyActionFault, WalkingAcceptsLargeDeltaButStillRejectsAbsoluteLimit)
{
  data.output.has_published_command = true;
  data.output.last_published.position = {-2.f, -.325f};
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  data.sensors.joint_pos[0] = 7.f;  // target 3.55; delta 3.3 rad is allowed for walking.
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 3.55f, 1e-6);
  data.sensors.joint_pos[0] = 20.f;
  EXPECT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
}


TEST_F(MimicActionFault, RequiresClipBoundaryAndDeltaOnTheSameJoint)
{
  // Large delta alone is accepted, even on an unclipped Mimic model.
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  data.sensors.joint_pos[0] = 7.f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 3.55f, 1e-6);

  // Reaching the boundary alone is also accepted; it does not clip the actual output.
  data.output.has_published_command = true;
  data.output.last_published.position = {6.05f, -.325f};
  data.sensors.joint_pos[0] = 12.f;
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 6.05f, 1e-6);

  // A large left delta cannot combine with saturation of the right joint.
  data.output.last_published.position = {.25f, 2.375f};
  data.sensors.joint_pos << 7.f, 10.1f;
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);

  data.sensors.joint_pos << -9.9f, -.7f;  // raw -10 and a large delta on the same joint.
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
}

TEST(ActionValidation, BasicFailuresSkipBoundaryChecksAndKeepTheirReasons)
{
  const std::vector<float> reference{0.f, 0.f, std::numeric_limits<float>::quiet_NaN()};
  const auto result = validate_action({14.f, 0.f, 0.f},
      {7.f, std::numeric_limits<float>::infinity(), 0.f}, &reference, ActionSafetyLimits{});
  EXPECT_FALSE(result.is_valid());
  EXPECT_EQ(result.boundary_check, BoundaryCheckState::SkippedInvalidAction);
  ASSERT_EQ(result.joints.size(), 3U);
  EXPECT_EQ(result.joints[0].rejection, ActionRejection::AbsoluteLimit);
  EXPECT_EQ(result.joints[1].rejection, ActionRejection::Nonfinite);
  EXPECT_EQ(result.joints[2].rejection, ActionRejection::InvalidReference);
}

TEST(ActionValidation, BoundaryAndDeltaMustOccurOnTheSameJoint)
{
  const std::vector<float> values{2.f, .1f}, reference{0.f, 0.f};
  auto result = validate_action({0.f, 10.f}, values, &reference, ActionSafetyLimits{});
  EXPECT_TRUE(result.is_valid());
  EXPECT_EQ(result.boundary_check, BoundaryCheckState::Completed);

  result = validate_action({10.f, 0.f}, values, &reference, ActionSafetyLimits{});
  EXPECT_FALSE(result.is_valid());
  EXPECT_EQ(result.joints[0].rejection, ActionRejection::BoundaryAndDelta);
  EXPECT_EQ(result.joints[1].rejection, ActionRejection::None);
}

TEST(ActionValidation, DisabledAndUnneededChecksAreDistinct)
{
  const auto walking = validate_action({10.f}, {2.f}, nullptr, ActionSafetyLimits{});
  EXPECT_TRUE(walking.is_valid());
  EXPECT_EQ(walking.boundary_check, BoundaryCheckState::Disabled);
  const std::vector<float> reference{1.9f};
  const auto mimic = validate_action({10.f}, {2.f}, &reference, ActionSafetyLimits{});
  EXPECT_TRUE(mimic.is_valid());
  EXPECT_EQ(mimic.boundary_check, BoundaryCheckState::NotNeeded);
}

namespace
{
class PolicyInferenceFault : public PolicyActionFault
{
protected:
  std::filesystem::path policy_directory() const override
  {
    return std::filesystem::path(COMMON_POLICY_TEST_DATA_DIR).parent_path() /
           "inference_error_policy";
  }
};
}  // namespace

TEST_F(PolicyInferenceFault, ExceptionsAndRejectedActionsShareTheRetryBudget)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto target = runtime->target_command().position;
  const auto feedback = data.policy.last_action;

  for (int attempt = 1; attempt <= 6; ++attempt) {
    data.sensors.joint_pos[0] = attempt % 2 == 0 ? 2.6f : 1.6f;
    EXPECT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(runtime->target_command().position, target);
    EXPECT_EQ(data.policy.last_action, feedback);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_EQ(data.requests.damping, attempt == 6);
  }
}

TEST_F(PolicyInferenceFault, FreshObservationRecoversFromExceptionsAndResetsRetries)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);

  for (int incident = 0; incident < 2; ++incident) {
    data.sensors.joint_pos[0] = 2.6f;

    for (int retry = 0; retry < 5; ++retry) {
      EXPECT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
      EXPECT_FALSE(data.requests.damping);
    }

    data.sensors.joint_pos[0] = .4f;
    EXPECT_EQ(tick(), PolicyUpdateResult::TargetReady);
    EXPECT_FALSE(data.requests.damping);
  }
}

TEST(ActionValidation, ConfiguredClippingCannotHideNonfiniteActions)
{
  ActionProperties properties;
  properties.scale = {1.f};
  properties.offset = {0.f};
  properties.clip = {std::make_pair(-1.f, 1.f)};
  ActionPipeline pipeline(properties);

  for (const float invalid : {std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
  {
    const auto validation = validate_action({invalid}, pipeline.process({invalid}),
      nullptr, ActionSafetyLimits{});
    EXPECT_FALSE(validation.is_valid());
    EXPECT_EQ(validation.joints[0].rejection, ActionRejection::Nonfinite);
  }

  EXPECT_EQ(pipeline.process({2.f}), std::vector<float>{1.f});
}
