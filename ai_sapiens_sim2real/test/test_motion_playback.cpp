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
