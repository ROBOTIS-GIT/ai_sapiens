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
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_manager.hpp"

using ai_sapiens_sim2real::ObservationContext;
using ai_sapiens_sim2real::ObservationManager;
using ai_sapiens_sim2real::ObservationRegistry;
using ai_sapiens_sim2real::PolicyJointContext;
using ai_sapiens_sim2real::SharedControlData;

namespace
{
YAML::Node history_config()
{
  ObservationRegistry::register_observation("history_signal",
    [](const ObservationContext & context, const YAML::Node &) {
      return std::vector<float>{context.shared.policy.phase_state.phase};
    });
  return YAML::Load(
        R"(
history_signal: {history_length: 2}
velocity_commands: {history_length: 2}
)");
}
}  // namespace

TEST(ObservationHistory, PreservesDefaultAndOptInOrderingAcrossWrapAndReset)
{
  for (int mode = 0; mode < 3; ++mode) {
    SCOPED_TRACE(mode);
    auto config = history_config();
    if (mode != 0) {config["use_gym_history"] = mode == 2;}
    const bool frame_major = mode == 2;
    SharedControlData shared;
    PolicyJointContext joints;
    ObservationManager manager(config, &shared, joints, nullptr);
    EXPECT_EQ(manager.get_observation_size(), 8u);
    EXPECT_EQ(manager.get_term_names(),
      (std::vector<std::string>{"history_signal", "velocity_commands"}));
    EXPECT_EQ(manager.compute(), std::vector<float>(8, 0));
    for (int step = 1; step <= 4; ++step) {
      shared.policy.phase_state.phase = static_cast<float>(step);
      shared.mode.velocity_commands.x() = step * 10.0f;
      const float previous = static_cast<float>(step - 1);
      const float current = static_cast<float>(step);
      const std::vector<float> expected = frame_major ?
        std::vector<float>{previous, previous * 10, 0, 0, current, current * 10, 0, 0} :
      std::vector<float>{previous, current, previous * 10, 0, 0, current * 10, 0, 0};
      EXPECT_EQ(manager.compute(), expected);
    }
    manager.reset();
    EXPECT_EQ(manager.compute(), frame_major ?
      (std::vector<float>{4, 40, 0, 0, 4, 40, 0, 0}) :
      (std::vector<float>{4, 4, 40, 0, 0, 40, 0, 0}));
  }
}

TEST(ObservationHistory, FrameMajorPreservesClippingAndScaling)
{
  auto config = history_config();
  config["use_gym_history"] = true;
  config["history_signal"]["clip"] = YAML::Load("[-1, 1]");
  config["history_signal"]["scale"] = 2;
  config["velocity_commands"]["scale"] = 2;
  config["velocity_commands"]["clip"] = YAML::Load("[-3, 3]");
  config["velocity_commands"]["scale_first"] = true;
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationManager manager(config, &shared, joints, nullptr);
  shared.policy.phase_state.phase = 3;
  shared.mode.velocity_commands.x() = 2;
  EXPECT_EQ(manager.compute(), (std::vector<float>{0, 0, 0, 0, 2, 3, 0, 0}));
  shared.policy.phase_state.phase = -3;
  shared.mode.velocity_commands.x() = -2;
  EXPECT_EQ(manager.compute(), (std::vector<float>{2, 3, 0, 0, -2, -3, 0, 0}));
}

TEST(ObservationHistory, EqualHistoryLengthIsRequiredOnlyForFrameMajor)
{
  auto config = history_config();
  config["velocity_commands"]["history_length"] = 3;
  SharedControlData shared;
  PolicyJointContext joints;
  EXPECT_NO_THROW(ObservationManager(config, &shared, joints, nullptr));
  config["use_gym_history"] = true;
  EXPECT_THROW(ObservationManager(config, &shared, joints, nullptr), std::runtime_error);
  config["use_gym_history"] = "invalid";
  EXPECT_THROW(ObservationManager(config, &shared, joints, nullptr), YAML::Exception);
}

TEST(ObservationHistory, EmptyAndSingleFrameHistoriesAreSupported)
{
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationManager empty(YAML::Load("use_gym_history: true"), &shared, joints, nullptr);
  EXPECT_EQ(empty.get_observation_size(), 0u);
  empty.reset();
  EXPECT_TRUE(empty.compute().empty());
  auto config = history_config();
  config["use_gym_history"] = true;
  config["history_signal"]["history_length"] = 1;
  config["velocity_commands"]["history_length"] = 1;
  ObservationManager manager(config, &shared, joints, nullptr);
  shared.policy.phase_state.phase = .5f;
  shared.mode.velocity_commands.x() = 2;
  EXPECT_EQ(manager.compute(), (std::vector<float>{.5f, 2, 0, 0}));
}

TEST(ObservationHistory, RejectedSamplesAreReplacedWithoutRescalingInEitherLayout)
{
  for (bool frame_major : {false, true}) {
    for (int history_length : {1, 4}) {
      SCOPED_TRACE(frame_major);
      SCOPED_TRACE(history_length);
      auto config = history_config();
      config["use_gym_history"] = frame_major;
      config["history_signal"]["history_length"] = history_length;
      config["velocity_commands"]["history_length"] = history_length;
      config["history_signal"]["scale"] = 2;
      config["velocity_commands"]["scale"] = 3;
      SharedControlData shared;
      PolicyJointContext joints;
      shared.policy.phase_state.phase = .5f;
      shared.mode.velocity_commands.x() = 2.f;
      ObservationManager manager(config, &shared, joints, nullptr);
      const auto good = manager.compute();
      const auto layout = manager.layout();
      ASSERT_EQ(layout.size(), 2u);

      for (int failure = 0; failure < 5; ++failure) {
        shared.policy.phase_state.phase = 100.f;
        shared.mode.velocity_commands.x() = 100.f;
        const auto failed = manager.compute();
        EXPECT_NE(failed, good);
        manager.reject_latest();
      }

      shared.policy.phase_state.phase = .5f;
      shared.mode.velocity_commands.x() = 2.f;
      const auto recovered = manager.compute();
      EXPECT_EQ(recovered, good);
      std::vector<int> visits(recovered.size(), 0);

      for (const auto & slice : layout) {
        for (size_t frame = 0; frame < slice.frames; ++frame) {
          for (size_t channel = 0; channel < slice.width; ++channel) {
            const auto index = slice.offset + frame * slice.frame_stride +
              channel * slice.element_stride;
            ASSERT_LT(index, recovered.size());
            ++visits[index];
            const float expected = slice.name == "history_signal" ? 1.f :
              channel == 0 ? 6.f : 0.f;
            EXPECT_FLOAT_EQ(recovered[index], expected);
          }
        }
      }

      EXPECT_EQ(visits, std::vector<int>(recovered.size(), 1));
    }
  }
}
