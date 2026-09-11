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

#include <limits>

#include "ai_sapiens_sim2real/policy/torso_orientation.hpp"

#include "ai_sapiens_sim2real/policy/motion_playback.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"

using ai_sapiens_sim2real::MotionPlayback;
using ai_sapiens_sim2real::MotionReference;

TEST(MotionPlayback, OpenEndedWindowNeverCompletes)
{
  MotionPlayback playback;  // time_start=0, time_end=inf
  EXPECT_FLOAT_EQ(playback.seek_time(0.0f).value(), 0.0f);
  EXPECT_FLOAT_EQ(playback.seek_time(100.0f).value(), 100.0f);
}

TEST(MotionPlayback, CompletesAtWindowEndFromZeroStart)
{
  MotionPlayback playback;
  playback.time_end = 5.0f;
  EXPECT_FLOAT_EQ(playback.seek_time(4.0f).value(), 4.0f);
  EXPECT_FLOAT_EQ(playback.seek_time(5.0f).value(), 5.0f);  // boundary still plays
  EXPECT_FALSE(playback.seek_time(5.01f).has_value());
}

TEST(MotionPlayback, NonZeroStartSeeksAndCompletesInAbsoluteMotionTime)
{
  MotionPlayback playback;
  playback.time_start = 3.0f;
  playback.time_end = 10.0f;  // play absolute motion times [3, 10]

  EXPECT_FLOAT_EQ(playback.seek_time(0.0f).value(), 3.0f);   // starts at time_start
  EXPECT_FLOAT_EQ(playback.seek_time(6.0f).value(), 9.0f);   // 3 + 6
  EXPECT_FLOAT_EQ(playback.seek_time(7.0f).value(), 10.0f);  // 3 + 7 == time_end, still plays
  EXPECT_FALSE(playback.seek_time(7.5f).has_value());        // 3 + 7.5 > time_end -> complete
}

TEST(MotionReference, RejectsInvalidFpsBeforeLoadingCsv)
{
  const std::vector<std::string> fallback_joint_order{"joint"};

  EXPECT_THROW(MotionReference("unused.csv", 0.0f, fallback_joint_order), std::runtime_error);
  EXPECT_THROW(MotionReference("unused.csv", -50.0f, fallback_joint_order), std::runtime_error);
  EXPECT_THROW(
    MotionReference("unused.csv", std::numeric_limits<float>::infinity(), fallback_joint_order),
    std::runtime_error);
}

using ai_sapiens_sim2real::initial_pelvis_alignment;
using ai_sapiens_sim2real::pelvis_orientation_observation;

TEST(PelvisOrientation, InitialHeadingIsRemovedButLaterDisturbanceIsObserved)
{
  const Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
  const Eigen::Quaternionf quarter(Eigen::AngleAxisf(1.57079632679f,Eigen::Vector3f::UnitZ()));
  const auto aligned = initial_pelvis_alignment(quarter,identity);
  const auto initial = pelvis_orientation_observation(quarter,identity,aligned);
  const std::array<float,6> expected_identity{1,0,0,1,0,0};
  for (size_t i=0;i<6;++i) EXPECT_NEAR(initial[i],expected_identity[i],1e-6);
  const auto disturbed = pelvis_orientation_observation(quarter*quarter,identity,aligned);
  const std::array<float,6> expected_disturbed{0,1,-1,0,0,0};
  for (size_t i=0;i<6;++i) EXPECT_NEAR(disturbed[i],expected_disturbed[i],1e-6);
}
TEST(PelvisOrientation, RollAndQuaternionSignUseRowMajorEncoding)
{
  const Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf roll(Eigen::AngleAxisf(1.57079632679f,Eigen::Vector3f::UnitX()));
  roll.coeffs() *= -2.0f;
  const auto result = pelvis_orientation_observation(roll,identity,identity);
  const std::array<float,6> expected{1,0,0,0,0,-1};
  for (size_t i=0;i<6;++i) EXPECT_NEAR(result[i],expected[i],1e-6);
}
