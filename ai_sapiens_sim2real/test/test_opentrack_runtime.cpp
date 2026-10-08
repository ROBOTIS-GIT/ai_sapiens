// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ai_sapiens_sim2real/command_transition.hpp"
#include "ai_sapiens_sim2real/command_publishers/joint_command_publisher.hpp"
#include "ai_sapiens_sim2real/policy/mimic_policy_runtime.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

using namespace ai_sapiens_sim2real;  // NOLINT
namespace fs = std::filesystem;

class OpenTrackRuntimeTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("opentrack_runtime_test");
    dir = fs::temp_directory_path() /
      ("opentrack_test_" + std::to_string(node->now().nanoseconds()));
    fs::create_directories(dir);
    data.resize(2, 2);
    // Robot order is deliberately different from policy/CSV order.
    data.sensors.joint_pos << 0.2f, 0.1f;
    std::ofstream csv(dir / "motion.csv");
    csv << "root_x,root_y,root_z,root_qw,root_qx,root_qy,root_qz,j1,j2,"
      "root_vx,root_vy,root_vz,root_wx,root_wy,root_wz,velocity:j1,velocity:j2,"
      "height:left_foot,height:right_foot,height:left_foot_top,height:right_foot_top,ref_root_height\n";
    for (int i = 0; i < 100; ++i) {
      csv << "0,0,0.8,1,0,0,0,1,2,0,0,0,0,0,0,0,0,0,0,0.05,0.05,0.8\n";
    }
  }
  void TearDown() override {fs::remove_all(dir);}

  std::unique_ptr<MimicPolicyRuntime> make(
    const std::string & model, int history = 0, float fps = 50.0f)
  {
    std::ofstream yaml(dir / "sim2real.yaml");
    yaml << "runtime_type: opentrack\nstep_dt: 0.02\njoint_vel_scale: 0.05\n"
      "policy_joints: [j1, j2]\nhistory_length: " << history << "\n"
      "joint_properties:\n"
      "  j1: {default_position: 0, stiffness: 20, damping: 2, position_limit: [-3, 3]}\n"
      "  j2: {default_position: 0, stiffness: 30, damping: 3, position_limit: [-3, 3]}\n"
      "commands: {}\nactions:\n  reference_residual: {scale: [1, 1], offset: [], clip: null}\n"
      "observations:\n";
    for (const auto * term : {"dif_joint_pos", "dif_joint_vel", "gvec_pelvis", "gyro_pelvis",
        "joint_pos", "joint_vel", "last_motor_targets", "ref_feet_height", "ref_root_height"})
    {
      yaml << "  " << term << ": {history_length: 1}\n";
    }
    yaml.close();
    MimicBehavior mimic;
    mimic.motion_file = dir / "motion.csv";
    mimic.fps = fps;
    mimic.on_complete = "Velocity";
    return MimicPolicyRuntime::create(node, model,
             (fs::path(OPENTRACK_FIXTURES) / (model + ".onnx")).string(),
      Sim2RealConfig(dir / "sim2real.yaml"), mimic, {"j2", "j1"}, &data);
  }
  PolicyUpdateResult step(PolicyRuntime & runtime, double dt = 0.02)
  {
    return runtime.update(rclcpp::Duration::from_seconds(dt));
  }
  rclcpp::Node::SharedPtr node;
  SharedControlData data;
  fs::path dir;
};

TEST_F(OpenTrackRuntimeTest, SpecialistUsesFinalPublishedTargetsAndResetsOnEntry)
{
  auto runtime = make("specialist");
  runtime->enter();
  ASSERT_EQ(step(*runtime, 0), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 1.1, 1e-5);
  EXPECT_NEAR(runtime->target_command().position[1], 2.2, 1e-5);
  data.output.command.position = {0.7f, 0.6f};
  // Verify the publication boundary, including position clipping.
  data.output.position_limits[1] = PositionLimit::value_type{-0.4f, 0.4f};
  JointCommandPublisher pub(node, &data.output, "/opentrack_test/commands", {"j2", "j1"}, true,
    true);
  pub.publish(node->now());
  ASSERT_EQ(data.output.published_command_count, 1u);
  runtime->command_published(data.output.last_published);
  ASSERT_EQ(step(*runtime), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 1.4, 1e-5);
  EXPECT_NEAR(runtime->target_command().position[1], 2.7, 1e-5);
  runtime->enter();
  ASSERT_EQ(step(*runtime, 0), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 1.1, 1e-5);
}

TEST_F(OpenTrackRuntimeTest, AdapterHistoryCommitsOnlyAfterPublication)
{
  auto runtime = make("adapter");
  runtime->enter();
  ASSERT_EQ(step(*runtime, 0), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 1.1, 1e-5);
  // No publication: unissued policy targets must not contaminate history.
  ASSERT_EQ(step(*runtime), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 1.1, 1e-5);
  JointCommand published;
  published.resize(2);
  published.position = {0.7f, 0.4f};
  runtime->command_published(published);
  // A skipped inference does not append another history frame.
  ASSERT_EQ(step(*runtime, 0), PolicyUpdateResult::Skipped);
  published.position = {0.9f, 0.8f};
  runtime->command_published(published);
  ASSERT_EQ(step(*runtime), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 1.4, 1e-5);
  EXPECT_NEAR(data.policy.input[14], 0.8, 1e-5);  // latest actually issued target
  runtime->enter();
  ASSERT_EQ(step(*runtime, 0), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(runtime->target_command().position[0], 1.1, 1e-5);
}

TEST_F(OpenTrackRuntimeTest, MotionClockAdvancesDuringCommandTransition)
{
  auto runtime = make("specialist");
  runtime->enter();
  CommandTransition transition(2);
  transition.begin(data.output.command, runtime->target_command(),
    runtime->controlled_joints(), 0.1, data.output.command);
  for (int i = 0; i < 3; ++i) {
    const auto result = step(*runtime, i == 0 ? 0.0 : 0.02);
    ASSERT_EQ(result, PolicyUpdateResult::TargetReady);
    transition.update(0.02, result, runtime->target_command(), data.output.command);
    EXPECT_NEAR(data.policy.episode_time, (i + 1) * 0.02, 1e-6);
  }
  // The motion progressed while the command still blends toward the policy.
  EXPECT_LT(data.output.command.stiffness[1], runtime->target_command().stiffness[0]);
}

TEST_F(OpenTrackRuntimeTest, RejectsInvalidTargetsAndNonfiniteSensors)
{
  auto rejected = make("rejected");
  rejected->enter();
  EXPECT_EQ(step(*rejected, 0), PolicyUpdateResult::TargetUnavailable);
  EXPECT_TRUE(data.requests.action_limit_exceeded);
  auto runtime = make("adapter");
  runtime->enter();
  data.sensors.joint_pos[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(step(*runtime, 0), PolicyUpdateResult::TargetUnavailable);
  EXPECT_TRUE(data.requests.damping);
}

TEST_F(OpenTrackRuntimeTest, RejectsMismatchedHistoryAndMotionFrequency)
{
  EXPECT_THROW(make("specialist", 4), std::runtime_error);
  EXPECT_THROW(make("adapter", 5), std::runtime_error);
  EXPECT_THROW(make("adapter", 4, 30), std::runtime_error);
}

TEST_F(OpenTrackRuntimeTest, VelocityDiagnosticIsNeverUsedAsAction)
{
  OnnxInference inference((fs::path(OPENTRACK_FIXTURES) / "velocity.onnx").string());
  std::unordered_map<std::string, std::vector<float>> inputs{
    {"obs", std::vector<float>(132)}, {"velocity_history", std::vector<float>(20 * 75)}};
  const auto action = inference.run(inputs);
  ASSERT_EQ(action.size(), 23u);
  for (const auto value : action) {
    EXPECT_FLOAT_EQ(value, 0.125f);
  }
}
