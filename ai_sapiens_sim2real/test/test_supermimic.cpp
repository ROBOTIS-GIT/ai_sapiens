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
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_recorder.hpp"
#include "ai_sapiens_sim2real/policy/mimic/motion_source.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"
#include "ai_sapiens_sim2real/policy/supermimic/supermimic_policy_runtime.hpp"

using ai_sapiens_sim2real::ActionFaultRecorder;
using ai_sapiens_sim2real::BehaviorKind;
using ai_sapiens_sim2real::ConfiguredObservationGroup;
using ai_sapiens_sim2real::ConfiguredObservationTerm;
using ai_sapiens_sim2real::MimicBehavior;
using ai_sapiens_sim2real::ModeDecision;
using ai_sapiens_sim2real::MotionSource;
using ai_sapiens_sim2real::PolicyController;
using ai_sapiens_sim2real::PolicyUpdateResult;
using ai_sapiens_sim2real::RootConfig;
using ai_sapiens_sim2real::SharedControlData;
using ai_sapiens_sim2real::Sim2RealConfig;
using ai_sapiens_sim2real::SuperMimicObservation;
using ai_sapiens_sim2real::SuperMimicPolicyRuntime;

namespace
{
class SuperMimic : public ::testing::Test
{
protected:
  const std::filesystem::path directory{SUPERMIMIC_TEST_DATA_DIR};
  Sim2RealConfig config{directory / "sim2real.yaml"};
  SharedControlData data;
  MimicBehavior mimic;

  void SetUp() override
  {
    data.resize(2, 2);
    data.resize_observation(136);
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

  std::unique_ptr<SuperMimicPolicyRuntime> runtime()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    auto runtime =
      SuperMimicPolicyRuntime::create(std::make_shared<rclcpp::Node>("supermimic_test"),
      "MimicSuper", (directory / "policy.onnx").string(), config, mimic, config.policy_joints(),
        &data);
    runtime->enter();
    return runtime;
  }
};
}  // namespace

TEST_F(SuperMimic, RejectedInitialSampleDoesNotKeepYawAlignment)
{
  const auto orientation = data.sensors.orientation;
  for (const bool switch_motion : {false, true}) {
    for (const bool invalid_observation : {false, true}) {
      SCOPED_TRACE(::testing::Message() << "switch=" << switch_motion <<
        " invalid_observation=" << invalid_observation);
      SuperMimicObservation actual(config, mimic, config.policy_joints(), &data);
      SuperMimicObservation expected(config, mimic, config.policy_joints(), &data);
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

TEST_F(SuperMimic, RejectedLaterSamplePreservesEstablishedYawAlignment)
{
  SuperMimicObservation actual(config, mimic, config.policy_joints(), &data);
  SuperMimicObservation expected(config, mimic, config.policy_joints(), &data);
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

TEST_F(SuperMimic, SixUnusableSamplesRequestDamping)
{
  auto policy = runtime();
  data.sensors.is_imu_usable = false;
  for (int attempt = 0; attempt < 6; ++attempt) {
    EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(attempt ? .02 : 0)),
      PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(data.requests.damping, attempt == 5);
  }
}

TEST_F(SuperMimic, MotionSelectionPreservesFeedbackTimeAndFailureBudget)
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
  policy->select_motion("MimicSuper");
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
    PolicyUpdateResult::TargetUnavailable);
  EXPECT_TRUE(data.requests.damping);
}

TEST_F(SuperMimic, UnsupportedPlaybackContractFailsAtLoad)
{
  mimic.fps = 30;
  EXPECT_THROW(SuperMimicObservation(config, mimic, config.policy_joints(), &data),
    std::runtime_error);
  mimic.fps = 50;
  mimic.time_start = .1f;
  EXPECT_THROW(SuperMimicObservation(config, mimic, config.policy_joints(), &data),
    std::runtime_error);
}

TEST_F(SuperMimic, YamlActionScaleControlsTargetsWithoutModelMetadata)
{
  const auto path = std::filesystem::temp_directory_path() / "super-mimic-config-mismatch.yaml";
  auto changed = YAML::LoadFile((directory / "sim2real.yaml").string());
  changed["actions"]["joint_pos"]["scale"] = std::vector<float>{.6f, .5f};
  std::ofstream(path) << changed;
  const Sim2RealConfig altered(path);
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto policy = SuperMimicPolicyRuntime::create(std::make_shared<rclcpp::Node>("yaml_action_test"),
    "MimicTest", (directory / "policy.onnx").string(), altered, mimic,
    config.policy_joints(), &data);
  policy->enter();
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(data.policy.last_action[0], .3f, 1e-6);
  EXPECT_NEAR(policy->target_command().position[0], .28f, 1e-6);
  std::filesystem::remove(path);
}

TEST_F(SuperMimic, ControllerSharesOnlySamePolicyAndReentersAfterDamping)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto pattern = (std::filesystem::temp_directory_path() /
    "super-mimic-controller-XXXXXX").string();
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
  PolicyController controller(std::make_shared<rclcpp::Node>("supermimic_controller_test"),
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
  EXPECT_NEAR(data.policy.input[20], .4f, 1e-6);

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
  EXPECT_FLOAT_EQ(data.policy.input[18], data.policy.input[22]);
  std::filesystem::remove_all(temp);
}


TEST_F(SuperMimic, MatchesNativeGoalExtractorAndTermMajorHistory)
{
  const auto expected = YAML::LoadFile((directory / "expected.yaml").string())["observations"];
  SuperMimicObservation observation(config, mimic, config.policy_joints(), &data);
  for (size_t i = 0; i < expected.size(); ++i) {
    data.sensors.orientation = Eigen::AngleAxisf(-.4f + i * .002f, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(.2f, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(-.1f, Eigen::Vector3f::UnitX());
    data.sensors.joint_pos[0] = .4f + i * .001f;
    data.policy.last_action = {i * .002f, -static_cast<float>(i) * .001f};
    const auto actual = observation.compute();
    const auto values = expected[i].as<std::vector<float>>();
    ASSERT_EQ(actual.size(), values.size());
    for (size_t j = 0; j < values.size(); ++j) {
      EXPECT_NEAR(actual[j], values[j], 3e-5) << "frame=" << i << " index=" << j;
    }
    observation.advance();
  }
  EXPECT_FALSE(observation.has_frame());
}

TEST_F(SuperMimic, FailureHoldsTargetAndDiscardsOnlyFailedHistory)
{
  auto policy = runtime();
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
  const auto target = policy->target_command().position;
  const auto accepted = data.policy.last_action;
  data.sensors.joint_pos[0] = 20;
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
    PolicyUpdateResult::TargetUnavailable);
  EXPECT_EQ(policy->target_command().position, target);
  EXPECT_EQ(data.policy.last_action, accepted);
  data.sensors.joint_pos[0] = .5f;
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(data.policy.input[18], .3f, 1e-6);
  EXPECT_NEAR(data.policy.input[20], .3f, 1e-6);
  EXPECT_NEAR(data.policy.input[22], .4f, 1e-6);
  EXPECT_FALSE(data.requests.damping);
}

TEST_F(SuperMimic, MotionSwitchPreservesHistoryAndCompletesIntoSelectedState)
{
  auto policy = runtime();
  mimic.on_complete = "Damping";
  policy->add_motion("second", mimic);
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
  data.sensors.joint_pos[0] = .5f;
  policy->select_motion("second");
  for (size_t i = 0; i < 64; ++i) {
    ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)), PolicyUpdateResult::TargetReady);
    if (i == 0) {EXPECT_NEAR(data.policy.input[20], .3f, 1e-6);}
  }
  EXPECT_EQ(policy->update(rclcpp::Duration::from_seconds(.02)),
    PolicyUpdateResult::TargetUnavailable);
  EXPECT_EQ(data.requests.state_name, "Damping");
  policy->select_motion("MimicSuper");
  policy->enter();
  EXPECT_EQ(data.policy.episode_time, 0);
  EXPECT_EQ(data.policy.last_action, std::vector<float>({0, 0}));
  ASSERT_EQ(policy->update(rclcpp::Duration::from_seconds(0)), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(data.policy.input[18], .4f, 1e-6);
}

TEST_F(SuperMimic, VelocityEncoderUsesCommandsAndHasNoMotionCompletion)
{
  const Sim2RealConfig velocity_config(directory / "velocity/sim2real.yaml");
  SuperMimicObservation observation(velocity_config, std::nullopt, config.policy_joints(), &data);
  data.mode.velocity_commands << .2f, -.3f, .4f;
  const auto values = observation.compute();
  ASSERT_EQ(values.size(), 39U);
  EXPECT_FLOAT_EQ(values[36], .2f);
  EXPECT_FLOAT_EQ(values[37], -.3f);
  EXPECT_FLOAT_EQ(values[38], .4f);
  for (int i = 0; i < 100; ++i) {
    observation.advance();
                                                       }
  EXPECT_TRUE(observation.has_frame());
  EXPECT_THROW(SuperMimicObservation(velocity_config, mimic, config.policy_joints(), &data),
    std::runtime_error);
  EXPECT_THROW(SuperMimicObservation(config, std::nullopt, config.policy_joints(), &data),
    std::runtime_error);
}

TEST(SuperMimicExportIntegration, LoadsPythonExportAndComputesValidatedTargets)
{
  const auto * path = std::getenv("SUPERMIMIC_EXPORT_BUNDLE");
  if (!path) {GTEST_SKIP() << "Set SUPERMIMIC_EXPORT_BUNDLE to an exported synthetic asset";}
  const std::filesystem::path directory(path);
  const Sim2RealConfig config(directory / "params/sim2real.yaml");
  MimicBehavior mimic;
  mimic.motion_file = directory / "motion.csv";
  mimic.source = std::make_shared<MotionSource>(mimic.motion_file.string(), 50.f,
    config.policy_joints());
  SharedControlData data;
  data.resize(config.policy_joints().size(), config.policy_joints().size());
  data.sensors.is_imu_usable = true;
  for (size_t j = 0; j < config.policy_joints().size(); ++j) {
    data.sensors.joint_pos[j] = config.joint_properties().default_position[j];
  }
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto runtime =
    SuperMimicPolicyRuntime::create(std::make_shared<rclcpp::Node>("supermimic_export_test"),
    "MimicSuperExport", (directory / "exported/policy.onnx").string(), config, mimic,
    config.policy_joints(), &data);
  runtime->enter();
  for (int tick = 0; tick < 3; ++tick) {
    EXPECT_EQ(runtime->update(rclcpp::Duration::from_seconds(tick ? .02 : 0)),
      PolicyUpdateResult::TargetReady);
  }
}

TEST_F(SuperMimic, FaultCaptureIdentifiesReferenceBeforeAndAfterSwitch)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto runtime = SuperMimicPolicyRuntime::create(
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

TEST_F(SuperMimic, YamlControlsPreprocessing)
{
  SuperMimicObservation original(config, mimic, config.policy_joints(), &data);
  const auto baseline = original.compute();
  auto yaml = YAML::LoadFile((directory / "sim2real.yaml").string());
  auto terms = yaml["observations"]["supermimic"]["history"]["terms"];
  const auto first = terms.begin()->first.as<std::string>();
  terms[first]["scale"] = 2.f;
  terms[first]["clip"] = std::vector<float>{-.1f, .1f};
  const auto path = std::filesystem::temp_directory_path() / "super-observation-config.yaml";
  std::ofstream(path) << yaml;
  const Sim2RealConfig altered(path);
  SuperMimicObservation observation(altered, mimic, config.policy_joints(), &data);
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

TEST(ConfiguredObservationGroup, OrderedTermsAndLayoutDeterminePacking)
{
  const std::map<std::string, size_t> widths{{"a", 2}, {"b", 1}};
  auto yaml =
    YAML::Load(
        R"(
layout: term_major
terms:
  b: {scale: 1.0, clip: null, history_length: 2}
  a: {scale: [1.0, 2.0], clip: null, history_length: 2}
)");
  const ConfiguredObservationGroup group(yaml, widths);
  ASSERT_EQ(group.terms()[0].name, "b");
  ASSERT_EQ(group.terms()[1].name, "a");
  std::vector<float> output;
  group.append_frames(output, {1, 2, 3, 4, 5, 6});
  EXPECT_EQ(output, std::vector<float>({1, 4, 2, 3, 5, 6}));
  yaml["layout"] = "frame_major";
  const ConfiguredObservationGroup temporal(yaml, widths);
  output.clear();
  temporal.append_frames(output, {1, 2, 3, 4, 5, 6});
  EXPECT_EQ(output, std::vector<float>({1, 2, 3, 4, 5, 6}));
  yaml["terms"]["a"]["history_length"] = 3;
  EXPECT_THROW(ConfiguredObservationGroup(yaml, widths), std::runtime_error);
  yaml["terms"]["a"]["history_length"] = 2;
  yaml["terms"]["b"]["clip"] = std::vector<float>{1, -1};
  EXPECT_THROW(ConfiguredObservationGroup(yaml, widths), std::runtime_error);
}


TEST_F(SuperMimic, RejectsYamlHistorySizeThatDoesNotMatchOnnx)
{
  auto yaml = YAML::LoadFile((directory / "sim2real.yaml").string());
  for (auto term : yaml["observations"]["supermimic"]["history"]["terms"]) {
    term.second["history_length"] = 4;
  }
  const auto path = std::filesystem::temp_directory_path() / "supermimic-wrong-input-size.yaml";
  std::ofstream(path) << yaml;
  const Sim2RealConfig altered(path);
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  EXPECT_THROW(SuperMimicPolicyRuntime::create(std::make_shared<rclcpp::Node>("wrong_size_test"),
    "MimicTest", (directory / "policy.onnx").string(), altered, mimic,
    config.policy_joints(), &data), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(ConfiguredObservationGroup, PreprocessingUsesOnlyTheExplicitTermRange)
{
  const ConfiguredObservationTerm term{"joint_pos", 2, {2.f, 3.f}, {{-1.f, 1.f}}};
  std::vector<float> values{99.f, 2.f, -2.f, 88.f};
  term.apply(values, 1, 3);
  EXPECT_EQ(values, (std::vector<float>{99.f, 2.f, -3.f, 88.f}));

  const auto original = values;
  EXPECT_THROW(term.apply(values, 1, 2), std::invalid_argument);
  EXPECT_THROW(term.apply(values, 3, 2), std::invalid_argument);
  EXPECT_THROW(term.apply(values, 3, 5), std::invalid_argument);
  EXPECT_EQ(values, original);
}
