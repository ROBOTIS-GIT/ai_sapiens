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
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <yaml-cpp/yaml.h>
#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_recorder.hpp"

using namespace ai_sapiens_sim2real;

namespace
{
class ActionFaultFiles : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string pattern = "/tmp/action-fault-test-XXXXXX";
    ASSERT_NE(::mkdtemp(pattern.data()), nullptr);
    directory = pattern;
    options.directory = directory;
    metadata = std::make_shared<FaultPolicyMetadata>();
    metadata->state = "test";
    metadata->policy_joints = {"knee"};
    metadata->input_names = {"obs"};
    metadata->input_shapes = {{1, 2}};
    metadata->layouts["obs"] = {{"joint_pos", 0, 1, 2, 1}};
    metadata->action_limit = 6.283f;
  }
  void TearDown() override {std::filesystem::remove_all(directory);}
  ActionFaultRecord record() const
  {
    ActionFaultRecord result;
    result.metadata = metadata;
    result.incident = 1;
    result.attempt = 1;
    result.current.calls.push_back({{{"obs", {.123456789f, 200.f}}}, {800.f}});
    result.current.processed_action = {200.f};
    result.current.validation = validate_action(
      {800.f}, result.current.processed_action, nullptr, ActionSafetyLimits{});
    result.current.outcome = PolicyActionOutcome::Retry;
    return result;
  }
  std::string enqueue_when_available(
    ActionFaultRecorder & recorder,
    const ActionFaultRecord & value)
  {
    // Production enqueue deliberately drops on lock contention; file tests wait
    // for a queue slot without changing the real-time writer contract.
    for (int attempt = 0; attempt < 25; ++attempt) {
      auto path = recorder.enqueue(value);
      if (!path.empty()) {return path;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return {};
  }
  std::filesystem::path directory;
  ActionFaultRecorder::Options options;
  std::shared_ptr<FaultPolicyMetadata> metadata;
  rclcpp::Logger logger{rclcpp::get_logger("fault_file_test")};
};
}  // namespace

TEST_F(ActionFaultFiles, RunsWithoutRecordsDoNotCreateDirectories)
{
  options.directory = directory / "not-created" / "action_faults";
  for (int run = 0; run < 3; ++run) {
    ActionFaultRecorder recorder(options, logger);
    EXPECT_FALSE(std::filesystem::exists(recorder.session_directory()));
  }
  EXPECT_FALSE(std::filesystem::exists(directory / "not-created"));
}

TEST_F(ActionFaultFiles, RoundTripsActualTensorsWithCaptureTimestampAboveDiagnostics)
{
  std::string path;
  {
    ActionFaultRecorder recorder(options, logger);
    EXPECT_FALSE(std::filesystem::exists(recorder.session_directory()));
    auto r = record();
    r.current.unix_time_ns = 1793545445123456789LL;
    r.current.calls.push_back({{{"obs", {0.f, std::numeric_limits<float>::quiet_NaN()}}},
        {std::numeric_limits<float>::infinity()}});
    path = enqueue_when_available(recorder, r);
    ASSERT_FALSE(path.empty());
  }  // Drains the queue on orderly shutdown.
  const auto file_permissions = std::filesystem::status(path).permissions();
  const auto directory_permissions = std::filesystem::status(
    std::filesystem::path(path).parent_path()).permissions();
  using Perms = std::filesystem::perms;
  EXPECT_NE(file_permissions & Perms::others_read, Perms::none);
  EXPECT_EQ(file_permissions & Perms::others_write, Perms::none);
  EXPECT_NE(directory_permissions & Perms::others_read, Perms::none);
  EXPECT_NE(directory_permissions & Perms::others_exec, Perms::none);
  EXPECT_EQ(directory_permissions & Perms::others_write, Perms::none);
  const auto file = YAML::LoadFile(path);
  EXPECT_EQ(file.begin()->first.as<std::string>(), "timestamp");
  EXPECT_EQ(file["timestamp"].as<std::string>(), "2026-11-02T00:04:05.123+09:00");
  EXPECT_EQ(file["timestamp_unix"].as<std::string>(), "1793545445.123456789");
  EXPECT_EQ(file["current_inference"]["unix_time_ns"].as<int64_t>(),
    1793545445123456789LL);
  EXPECT_NE(file["diagnostics"].as<std::string>().find("200.000 [LIMIT]"), std::string::npos);
  const auto calls = file["current_inference"]["onnx_calls"];
  ASSERT_EQ(calls.size(), 2U);
  const auto input = calls[0]["inputs"][0];
  EXPECT_EQ(input["shape"].as<std::vector<int64_t>>(), std::vector<int64_t>({1, 2}));
  EXPECT_EQ(input["raw_tensor"].as<std::vector<float>>(), std::vector<float>({.123456789f, 200.f}));
  EXPECT_EQ(input["terms"]["joint_pos"]["knee"].as<std::vector<float>>(),
    input["raw_tensor"].as<std::vector<float>>());
  EXPECT_TRUE(std::isnan(calls[1]["inputs"][0]["raw_tensor"][1].as<float>()));
  EXPECT_TRUE(std::isinf(calls[1]["raw_output"][0].as<float>()));
}

TEST_F(ActionFaultFiles, RestartsUseUniqueDirectoriesAndNeverOverwriteExistingFile)
{
  std::filesystem::path first, second, collision;
  {
    ActionFaultRecorder recorder(options, logger);
    first = recorder.session_directory();
    ASSERT_FALSE(std::filesystem::exists(first));
    std::filesystem::create_directory(first);  // Simulate a pre-existing path collision.
    collision = first / "incident-1-attempt-1.yaml";
    std::ofstream(collision) << "preserve this exact content";
    ASSERT_FALSE(enqueue_when_available(recorder, record()).empty());
  }
  std::ifstream old(collision);
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(old), {}), "preserve this exact content");
  {
    ActionFaultRecorder recorder(options, logger);
    second = recorder.session_directory();
    ASSERT_FALSE(enqueue_when_available(recorder, record()).empty());
  }
  EXPECT_NE(first, second);
  EXPECT_TRUE(std::filesystem::exists(second / "incident-1-attempt-1.yaml"));
}

TEST_F(ActionFaultFiles, QuotaCountsOldRecordsWithoutDeletingThem)
{
  const auto old = directory / "previous.yaml";
  std::ofstream(old) << std::string(1024, 'x');
  options.max_storage_bytes = 100;
  std::filesystem::path session;
  {
    ActionFaultRecorder recorder(options, logger);
    session = recorder.session_directory();
    ASSERT_FALSE(enqueue_when_available(recorder, record()).empty());
  }
  EXPECT_EQ(std::filesystem::file_size(old), 1024U);
  EXPECT_FALSE(std::filesystem::exists(session));
}

TEST_F(ActionFaultFiles, OversizedQueueRecordIsDroppedWithoutCreatingAFile)
{
  options.max_queue_bytes = 1;
  std::filesystem::path session;
  {
    ActionFaultRecorder recorder(options, logger);
    session = recorder.session_directory();
    EXPECT_TRUE(recorder.enqueue(record()).empty());
  }
  EXPECT_FALSE(std::filesystem::exists(session));
}

TEST_F(ActionFaultFiles, SharedPolicySnapshotsKeepTheirOwnReferenceContext)
{
  std::string path;
  {
    ActionFaultRecorder recorder(options, logger);
    auto value = record();
    value.current.context = {"MimicDance2", "dance2", "/motions/dance2.csv", 7};
    value.previous_accepted = value.current;
    value.previous_accepted->context = {"MimicDance1", "dance1", "/motions/dance1.csv", 49};
    path = enqueue_when_available(recorder, value);
    ASSERT_FALSE(path.empty());
    value.current.context = {"MimicDance3", "dance3", "/motions/dance3.csv", 0};
  }
  const auto file = YAML::LoadFile(path);
  const auto current = file["current_inference"];
  const auto previous = file["previous_accepted_inference"];
  EXPECT_EQ(current["active_state"].as<std::string>(), "MimicDance2");
  EXPECT_EQ(current["behavior"].as<std::string>(), "dance2");
  EXPECT_EQ(current["motion_file"].as<std::string>(), "/motions/dance2.csv");
  EXPECT_EQ(current["motion_frame_zero_based"].as<size_t>(), 7u);
  EXPECT_EQ(previous["active_state"].as<std::string>(), "MimicDance1");
  EXPECT_EQ(previous["behavior"].as<std::string>(), "dance1");
  EXPECT_EQ(previous["motion_file"].as<std::string>(), "/motions/dance1.csv");
  EXPECT_EQ(previous["motion_frame_zero_based"].as<size_t>(), 49u);
  EXPECT_NE(file["diagnostics"].as<std::string>().find("Frame (zero-based): 7"),
    std::string::npos);
}
