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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>

#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/controllers/policy_controller.hpp"
#include "ai_sapiens_sim2real/policy/blinkmimic/blinkmimic_policy_runtime.hpp"
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_recorder.hpp"
#include "ai_sapiens_sim2real/policy/mimic/motion_source.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

using ai_sapiens_sim2real::ActionFaultRecorder;
using ai_sapiens_sim2real::BehaviorKind;
using ai_sapiens_sim2real::BlinkMimicMotion;
using ai_sapiens_sim2real::BlinkMimicObservation;
using ai_sapiens_sim2real::BlinkMimicPolicyRuntime;
using ai_sapiens_sim2real::MimicBehavior;
using ai_sapiens_sim2real::ModeDecision;
using ai_sapiens_sim2real::MotionSource;
using ai_sapiens_sim2real::PolicyController;
using ai_sapiens_sim2real::PolicyUpdateResult;
using ai_sapiens_sim2real::RootConfig;
using ai_sapiens_sim2real::SharedControlData;
using ai_sapiens_sim2real::Sim2RealConfig;

namespace
{
class BlinkMimic : public ::testing::Test
{
protected:
  const std::filesystem::path directory{BLINK_TEST_DATA_DIR};
  Sim2RealConfig config{directory / "sim2real.yaml"};
  SharedControlData data;
  MimicBehavior mimic;

  void SetUp() override
  {
    data.resize(2, 2);
    data.resize_observation(92);
    data.sensors.joint_pos << .4f, -.7f;
    data.sensors.joint_vel << .8f, -.9f;
    data.sensors.angular_velocity << .1f, .2f, .3f;
    data.sensors.orientation = Eigen::AngleAxisf(-.4f, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(.2f, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(-.1f, Eigen::Vector3f::UnitX());
    data.sensors.is_imu_usable = true;
    mimic.motion_file = directory / "motion.csv";
    mimic.source = std::make_shared<MotionSource>(mimic.motion_file.string(), 50.f,
        config.policy_joints());
    mimic.on_complete = "Velocity";
  }

  std::unique_ptr<BlinkMimicPolicyRuntime> runtime()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    auto runtime = BlinkMimicPolicyRuntime::create(std::make_shared<rclcpp::Node>("blink_test"),
      "MimicBlink", (directory / "policy.onnx").string(), config, mimic, config.policy_joints(),
        &data);
    runtime->enter();
    return runtime;
  }
};
}  // namespace

TEST_F(BlinkMimic, ReferenceDerivativesIncludeEndpoints)
{
  BlinkMimicMotion motion(mimic.source->make_cursor(), 50, config.policy_joints());
  ASSERT_EQ(motion.size(), 8U);
  for (size_t i = 0; i < motion.size(); ++i) {
    const float t = static_cast<float>(i) * .02f;
    const float derivative_t = i == 0 ? .01f : (i == 7 ? .13f : t);
    EXPECT_NEAR(motion.frame(i).linear_velocity.x(), 4 * derivative_t, 1e-5);
    EXPECT_NEAR(motion.frame(i).linear_velocity.y(), .3f, 1e-5);
    EXPECT_NEAR(motion.frame(i).joint_velocity[0], 2 * derivative_t, 1e-5);
    EXPECT_NEAR(motion.frame(i).joint_velocity[1], 2, 1e-5);
    EXPECT_TRUE(motion.frame(i).angular_velocity.allFinite());
  }
}

TEST_F(BlinkMimic, HistoryRejectsOnlyFailedSampleAndReentryResets)
{
  BlinkMimicObservation observation(config, mimic, config.policy_joints(), &data);
  observation.reset();
  const auto first = observation.compute();
  ASSERT_EQ(first.size(), 92U);
  for (size_t frame = 1; frame < 3; ++frame) {
    EXPECT_EQ(std::vector<float>(first.begin(), first.begin() + 18),
      std::vector<float>(first.begin() + frame * 18, first.begin() + (frame + 1) * 18));
  }
  observation.advance();
  data.sensors.joint_pos[0] = 20;
  observation.compute();
  observation.reject_latest();
  observation.advance();
  data.sensors.joint_pos[0] = .5f;
  data.policy.last_action = {.2f, -.3f};
  const auto recovered = observation.compute();
  EXPECT_FLOAT_EQ(recovered[0], first[0]);
  EXPECT_FLOAT_EQ(recovered[18], first[0]);
  EXPECT_NEAR(recovered[36], .4f, 1e-6);
  EXPECT_FLOAT_EQ(recovered[46], .2f);
  EXPECT_FLOAT_EQ(recovered[47], -.3f);
  data.sensors.joint_pos[0] = .4f;
  data.policy.last_action = {0, 0};
  observation.reset();
  EXPECT_EQ(observation.compute(), first);
}

TEST_F(BlinkMimic, RejectedInitialSampleDoesNotKeepYawAlignment)
{
  const auto orientation = data.sensors.orientation;
  for (const bool switch_motion : {false, true}) {
    for (const bool invalid_observation : {false, true}) {
      SCOPED_TRACE(::testing::Message() << "switch=" << switch_motion <<
        " invalid_observation=" << invalid_observation);
      BlinkMimicObservation actual(config, mimic, config.policy_joints(), &data);
      BlinkMimicObservation expected(config, mimic, config.policy_joints(), &data);
      data.sensors.orientation = orientation;
      data.sensors.joint_pos[0] = .4f;
      if (switch_motion) {
        actual.compute();
        expected.compute();
        const auto motion = actual.prepare_motion(mimic);
        actual.select_motion(motion);
        expected.select_motion(motion);
      }

      data.sensors.orientation = Eigen::AngleAxisf(1.f, Eigen::Vector3f::UnitZ()) *
        orientation;
      if (invalid_observation) {
        data.sensors.joint_pos[0] = std::numeric_limits<float>::quiet_NaN();
        EXPECT_THROW(actual.compute(), std::runtime_error);
      } else {
        actual.compute();
      }
      actual.reject_latest();
      actual.reject_latest();  // Rejection must be idempotent.
      actual.advance();
      expected.advance();

      data.sensors.orientation = orientation;
      data.sensors.joint_pos[0] = .4f;
      const auto recovered = actual.compute();
      const auto clean = expected.compute();
      ASSERT_EQ(recovered.size(), clean.size());
      for (size_t i = 0; i < clean.size(); ++i) {
        EXPECT_NEAR(recovered[i], clean[i], 1e-6) << "index=" << i;
      }
    }
  }
}

TEST_F(BlinkMimic, RejectedLaterSamplePreservesEstablishedYawAlignment)
{
  BlinkMimicObservation actual(config, mimic, config.policy_joints(), &data);
  BlinkMimicObservation expected(config, mimic, config.policy_joints(), &data);
  actual.compute();
  expected.compute();
  actual.advance();
  expected.advance();

  const auto orientation = data.sensors.orientation;
  data.sensors.orientation = Eigen::AngleAxisf(1.f, Eigen::Vector3f::UnitZ()) * orientation;
  actual.compute();
  actual.reject_latest();
  actual.advance();
  expected.advance();

  data.sensors.orientation = Eigen::AngleAxisf(.2f, Eigen::Vector3f::UnitZ()) * orientation;
  const auto recovered = actual.compute();
  const auto clean = expected.compute();
  ASSERT_EQ(recovered.size(), clean.size());
  for (size_t i = 0; i < clean.size(); ++i) {
    EXPECT_NEAR(recovered[i], clean[i], 1e-6) << "index=" << i;
  }
}

TEST_F(BlinkMimic, CompletionAndValidatedFeedbackUseCommonRuntime)
{
  auto policy = runtime();
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(i ? .02 : 0)),
      PolicyUpdateResult::TargetReady);
    EXPECT_TRUE(data.requests.state_name.empty());
    EXPECT_NEAR(data.policy.last_action[0], .3f, 1e-6);
    EXPECT_NEAR(policy->target_command().position[0], .25f, 1e-6);
  }
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
    PolicyUpdateResult::TargetUnavailable);
  EXPECT_EQ(data.requests.state_name, "Velocity");
  policy->enter();
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
}

TEST_F(BlinkMimic, RejectedActionAndObservationExceptionHoldLastAcceptedTarget)
{
  auto policy = runtime();
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
  const auto target = policy->target_command().position;
  const auto last = data.policy.last_action;
  data.sensors.joint_pos[0] = 20;
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
    PolicyUpdateResult::TargetUnavailable);
  EXPECT_EQ(policy->target_command().position, target);
  EXPECT_EQ(data.policy.last_action, last);
  data.sensors.joint_pos[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
    PolicyUpdateResult::TargetUnavailable);
  EXPECT_EQ(policy->target_command().position, target);
  data.sensors.joint_pos[0] = .4f;
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)), PolicyUpdateResult::TargetReady);
  EXPECT_FALSE(data.requests.damping);
  EXPECT_FALSE(data.requests.action_limit_exceeded);
  EXPECT_NEAR(data.policy.input[0], .3f, 1e-6);
}

TEST_F(BlinkMimic, SixUnusableSamplesRequestDamping)
{
  auto policy = runtime();
  data.sensors.is_imu_usable = false;
  for (int attempt = 0; attempt < 6; ++attempt) {
    EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(attempt ? .02 : 0)),
      PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(data.requests.damping, attempt == 5);
  }
}

TEST_F(BlinkMimic, ReferenceSwitchPreservesHistoryAndRestartsOnlyReference)
{
  BlinkMimicObservation observation(config, mimic, config.policy_joints(), &data);
  const auto start = observation.compute();
  observation.advance();
  data.sensors.joint_pos[0] = .5f;
  const auto before = observation.compute();
  data.policy.last_action = {.2f, -.3f};
  observation.select_motion(observation.prepare_motion(mimic));
  const auto after = observation.compute();
  EXPECT_EQ(std::vector<float>(after.begin(), after.begin() + 36),
    std::vector<float>(before.begin() + 18, before.begin() + 54));
  EXPECT_FLOAT_EQ(after[46], .2f);
  EXPECT_FLOAT_EQ(after[47], -.3f);
  EXPECT_EQ(std::vector<float>(after.begin() + 54, after.end()),
    std::vector<float>(start.begin() + 54, start.end()));
  observation.reject_latest();
  EXPECT_EQ(observation.compute(), after);
}

TEST_F(BlinkMimic, MotionSelectionPreservesFeedbackTimeAndFailureBudget)
{
  auto policy = runtime();
  mimic.on_complete = "Damping";
  policy->add_motion("second", mimic);
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)),
    PolicyUpdateResult::TargetReady);
  const auto action = data.policy.last_action;
  const auto time = data.policy.episode_time;
  const auto target = policy->target_command().position;
  policy->select_motion("second");
  EXPECT_EQ(data.policy.last_action, action);
  EXPECT_EQ(data.policy.episode_time, time);
  EXPECT_EQ(policy->target_command().position, target);

  data.sensors.is_imu_usable = false;
  for (int attempt = 0; attempt < 5; ++attempt) {
    EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
      PolicyUpdateResult::TargetUnavailable);
    EXPECT_FALSE(data.requests.damping);
  }
  policy->select_motion("MimicBlink");
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
    PolicyUpdateResult::TargetUnavailable);
  EXPECT_TRUE(data.requests.damping);
}

TEST_F(BlinkMimic, SelectedMotionOwnsCompletionAndPolicyReentryStillResets)
{
  auto policy = runtime();
  mimic.on_complete = "Damping";
  policy->add_motion("second", mimic);
  policy->select_motion("second");
  for (size_t i = 0; i < 8; ++i) {
    ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(i ? .02 : 0)),
      PolicyUpdateResult::TargetReady);
  }
  policy->update(rclcpp::Duration::from_seconds(.02));
  EXPECT_EQ(data.requests.state_name, "Damping");
  policy->select_motion("MimicBlink");
  policy->enter();
  EXPECT_EQ(data.policy.episode_time, 0);
  EXPECT_EQ(data.policy.last_action, std::vector<float>({0, 0}));
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)),
    PolicyUpdateResult::TargetReady);
  EXPECT_EQ(std::vector<float>(data.policy.input.begin(), data.policy.input.begin() + 18),
    std::vector<float>(data.policy.input.begin() + 18, data.policy.input.begin() + 36));
}

TEST_F(BlinkMimic, UnsupportedPlaybackContractFailsAtLoad)
{
  mimic.fps = 30;
  EXPECT_THROW(
    BlinkMimicObservation(config, mimic, config.policy_joints(), &data), std::runtime_error);
  mimic.fps = 50;
  mimic.time_start = .1f;
  EXPECT_THROW(
    BlinkMimicObservation(config, mimic, config.policy_joints(), &data), std::runtime_error);
}

TEST_F(BlinkMimic, MatchesUpstreamMotionGoalsAndTemporalHistory)
{
  const auto expected = YAML::LoadFile((directory / "expected.yaml").string())["observations"];
  BlinkMimicObservation observation(config, mimic, config.policy_joints(), &data);
  observation.reset();
  for (size_t frame = 0; frame < expected.size(); ++frame) {
    const auto actual = observation.compute();
    const auto values = expected[frame].as<std::vector<float>>();
    ASSERT_EQ(actual.size(), values.size());
    for (size_t index = 0; index < values.size(); ++index) {
      EXPECT_NEAR(actual[index], values[index], 2e-5) << "frame=" << frame << " index=" << index;
    }
    observation.advance();
  }
  EXPECT_FALSE(observation.has_frame());
}

TEST_F(BlinkMimic, YamlActionScaleControlsTargetsWithoutModelMetadata)
{
  const auto path = std::filesystem::temp_directory_path() / "blink-config-mismatch.yaml";
  auto changed = YAML::LoadFile((directory / "sim2real.yaml").string());
  changed["actions"]["joint_pos"]["scale"] = std::vector<float>{.6f, .5f};
  std::ofstream(path) << changed;
  const Sim2RealConfig altered(path);
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto policy = BlinkMimicPolicyRuntime::create(std::make_shared<rclcpp::Node>("yaml_action_test"),
    "MimicTest", (directory / "policy.onnx").string(), altered, mimic,
    config.policy_joints(), &data);
  policy->enter();
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(data.policy.last_action[0], .3f, 1e-6);
  EXPECT_NEAR(policy->target_command().position[0], .28f, 1e-6);
  std::filesystem::remove(path);
}

TEST(BlinkExportIntegration, LoadsPythonExportAndComputesValidatedTargets)
{
  const auto * path = std::getenv("BLINK_EXPORT_BUNDLE");
  if (!path) {GTEST_SKIP() << "Set BLINK_EXPORT_BUNDLE to an exported synthetic asset";}
  const std::filesystem::path directory(path);
  const Sim2RealConfig config(directory / "params/sim2real.yaml");
  MimicBehavior mimic;
  mimic.motion_file = directory / "params/motions/dance1.csv";
  mimic.source = std::make_shared<MotionSource>(mimic.motion_file.string(), 50.f,
    config.policy_joints());
  SharedControlData data;
  data.resize(config.policy_joints().size(), config.policy_joints().size());
  data.sensors.is_imu_usable = true;
  for (size_t j = 0; j < config.policy_joints().size(); ++j) {
    data.sensors.joint_pos[j] = config.joint_properties().default_position[j];
  }
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto runtime = BlinkMimicPolicyRuntime::create(
    std::make_shared<rclcpp::Node>("blink_export_test"),
    "MimicBlinkExport", (directory / "exported/policy.onnx").string(), config, mimic,
    config.policy_joints(), &data);
  runtime->enter();
  for (int tick = 0; tick < 3; ++tick) {
    EXPECT_EQ(runtime->update(rclcpp::Duration::from_seconds(tick ? .02 : 0)),
      PolicyUpdateResult::TargetReady);
  }
}

TEST_F(BlinkMimic, ControllerSharesOnlySamePolicyAndReentersAfterDamping)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto pattern = (std::filesystem::temp_directory_path() / "blink-controller-XXXXXX").string();
  ASSERT_NE(::mkdtemp(pattern.data()), nullptr);
  const std::filesystem::path temp(pattern);
  const auto other_model = temp / "other.onnx";
  std::filesystem::copy_file(directory / "policy.onnx", other_model);
  auto root =
    YAML::Load(
        R"(
robot_joint_order: [left_hip_roll_joint, right_hip_roll_joint]
teleop_input: {plugin: test, config: test.yaml}
teleop_conditions: {DampingRequested: {input_code: 1}}
authority:
  default_velocity_state: Velocity
  api_entry: {allowed_from_states: [Damping]}
state_machine:
  initial: Damping
  states:
    Damping: {run: damping}
    A: {run: first}
    B: {run: second}
    C: {run: other}
    Velocity: {run: velocity}
state_behaviors:
  damping: {kind: damping, damping: {values: [0.0, 0.0]}}
)");
  const std::filesystem::path common(COMMON_POLICY_TEST_DATA_DIR);
  auto velocity_config = YAML::LoadFile((common / "sim2real.yaml").string());
  velocity_config["observations"] = YAML::Load(
    "{joint_pos_rel: {history_length: 9}, velocity_commands: {}}");
  std::ofstream(temp / "velocity.yaml") << velocity_config;
  auto velocity = root["state_behaviors"]["velocity"];
  velocity["kind"] = "policy";
  velocity["policy_path"] = (common / "policy.onnx").string();
  velocity["sim2real_yaml_path"] = (temp / "velocity.yaml").string();
  for (const auto * name : {"first", "second", "other"}) {
    auto behavior = root["state_behaviors"][name];
    behavior["kind"] = "mimic";
    behavior["asset"] = "test/blink";
    behavior["policy_path"] = (std::string(name) == "other" ?
      other_model : directory / "policy.onnx").string();
    behavior["sim2real_yaml_path"] = (directory / "sim2real.yaml").string();
    behavior["motion_file"] = (directory / "motion.csv").string();
    behavior["on_complete"] = "Damping";
    behavior["action_transition"] = YAML::Load("{enabled: true, duration: 0.5}");
  }
  const auto path = temp / "root.yaml";
  std::ofstream(path) << root;
  const RootConfig root_config(path);
  data.joint_map.controller_joint_names = config.policy_joints();
  data.teleop.unavailable = false;
  PolicyController controller(std::make_shared<rclcpp::Node>("blink_controller_test"),
    root_config, &data);
  ModeDecision decision;
  decision.active_behavior_kind = BehaviorKind::Policy;
  decision.active_policy_name = "first";
  decision.transition_count = 1;
  const auto tick = [&]() {
      controller.update(decision, rclcpp::Time(0), rclcpp::Duration::from_seconds(.02));
    };
  tick();
  data.sensors.joint_pos[0] = .5f;
  tick();
  const auto time = data.policy.episode_time;
  decision.active_policy_name = "second";
  ++decision.transition_count;
  data.sensors.joint_pos[0] = .6f;
  tick();
  // A reference change within the same model must not start action blending.
  EXPECT_NEAR(data.output.command.position[0], .35f, 1e-6);
  EXPECT_NEAR(data.policy.episode_time, time + .02, 1e-6);
  EXPECT_NEAR(data.policy.input[18], .4f, 1e-6);

  for (const auto * name : {"other", "second"}) {
    decision.active_policy_name = name;
    ++decision.transition_count;
    tick();
    EXPECT_NEAR(data.policy.episode_time, .02, 1e-6);
  }
  decision.active_behavior_kind = BehaviorKind::Damping;
  ++decision.transition_count;
  tick();
  decision.active_behavior_kind = BehaviorKind::Policy;
  ++decision.transition_count;
  tick();
  EXPECT_NEAR(data.policy.episode_time, .02, 1e-6);
  EXPECT_EQ(std::vector<float>(data.policy.input.begin(), data.policy.input.begin() + 18),
    std::vector<float>(data.policy.input.begin() + 18, data.policy.input.begin() + 36));
  std::filesystem::remove_all(temp);
}

TEST_F(BlinkMimic, FaultCaptureIdentifiesReferenceBeforeAndAfterSwitch)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto runtime = BlinkMimicPolicyRuntime::create(
    std::make_shared<rclcpp::Node>("reference_fault_test"),
    "first", (directory / "policy.onnx").string(), config, mimic, config.policy_joints(), &data);
  auto second = mimic;
  second.motion_file = directory / "second.csv";  // Same prepared data, distinct diagnostic path.
  runtime->add_motion("second", second);

  for (int trial = 0; trial < 5; ++trial) {
    auto pattern = (std::filesystem::temp_directory_path() / "reference-fault-XXXXXX").string();
    ASSERT_NE(::mkdtemp(pattern.data()), nullptr);
    ActionFaultRecorder::Options options;
    options.directory = pattern;
    auto recorder = std::make_shared<ActionFaultRecorder>(
      options, rclcpp::get_logger("reference_test"));
    const auto file = recorder->session_directory() / "incident-1-attempt-1.yaml";
    runtime->set_fault_recorder(recorder);
    runtime->select_motion("first");
    runtime->set_active_behavior("MimicFirst", "first");
    runtime->enter();
    data.sensors.is_imu_usable = true;
    ASSERT_EQ(runtime->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
    ASSERT_EQ(
      runtime->update(rclcpp::Duration::from_seconds(.02)), PolicyUpdateResult::TargetReady);
    runtime->select_motion("second");
    runtime->set_active_behavior("MimicSecond", "second");
    data.sensors.is_imu_usable = false;
    ASSERT_EQ(
      runtime->update(rclcpp::Duration::from_seconds(.02)), PolicyUpdateResult::TargetUnavailable);
    runtime->select_motion("first");
    runtime->set_active_behavior("MimicFirst", "first");
    runtime->set_fault_recorder(nullptr);
    recorder.reset();
    if (!std::filesystem::exists(file)) {
      std::filesystem::remove_all(pattern);
      continue;  // The diagnostic queue intentionally drops on lock contention.
    }
    const auto record = YAML::LoadFile(file.string());
    const auto current = record["current_inference"];
    const auto previous = record["previous_accepted_inference"];
    EXPECT_EQ(current["active_state"].as<std::string>(), "MimicSecond");
    EXPECT_EQ(current["behavior"].as<std::string>(), "second");
    EXPECT_EQ(current["motion_file"].as<std::string>(), second.motion_file.string());
    EXPECT_EQ(current["motion_frame_zero_based"].as<size_t>(), 0u);
    EXPECT_EQ(current["onnx_calls"].size(), 0u);
    EXPECT_EQ(previous["active_state"].as<std::string>(), "MimicFirst");
    EXPECT_EQ(previous["motion_file"].as<std::string>(), mimic.motion_file.string());
    EXPECT_EQ(previous["motion_frame_zero_based"].as<size_t>(), 1u);
    std::filesystem::remove_all(pattern);
    return;
  }
  FAIL() << "No diagnostic file was queued in five trials";
}

TEST_F(BlinkMimic, YamlControlsPreprocessing)
{
  BlinkMimicObservation original(config, mimic, config.policy_joints(), &data);
  const auto baseline = original.compute();
  auto yaml = YAML::LoadFile((directory / "sim2real.yaml").string());
  auto terms = yaml["observations"]["blinkmimic"]["history"]["terms"];
  const auto first = terms.begin()->first.as<std::string>();
  terms[first]["scale"] = 2.f;
  terms[first]["clip"] = std::vector<float>{-.1f, .1f};
  const auto path = std::filesystem::temp_directory_path() / "blink-observation-config.yaml";
  std::ofstream(path) << yaml;
  const Sim2RealConfig altered(path);
  BlinkMimicObservation observation(altered, mimic, config.policy_joints(), &data);
  const auto actual = observation.compute();
  ASSERT_EQ(actual.size(), baseline.size());
  const auto slice = observation.layout()[0];
  for (size_t t = 0; t < slice.frames; ++t) {
    for (size_t j = 0; j < slice.width; ++j) {
      const auto index = slice.offset + t * slice.frame_stride + j;
      EXPECT_FLOAT_EQ(actual[index], 2.f * std::clamp(baseline[index], -.1f, .1f));
    }
  }
  std::filesystem::remove(path);
}

TEST_F(BlinkMimic, RejectsYamlHistorySizeThatDoesNotMatchOnnx)
{
  auto yaml = YAML::LoadFile((directory / "sim2real.yaml").string());
  for (auto term : yaml["observations"]["blinkmimic"]["history"]["terms"]) {
    term.second["history_length"] = 4;
  }
  const auto path = std::filesystem::temp_directory_path() / "blinkmimic-wrong-input-size.yaml";
  std::ofstream(path) << yaml;
  const Sim2RealConfig altered(path);
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  EXPECT_THROW(BlinkMimicPolicyRuntime::create(std::make_shared<rclcpp::Node>("wrong_size_test"),
    "MimicTest", (directory / "policy.onnx").string(), altered, mimic,
    config.policy_joints(), &data), std::runtime_error);
  std::filesystem::remove(path);
}
