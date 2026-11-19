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
#include <unistd.h>

#include <cstdio>
#include <limits>

#include "ai_sapiens_sim2real/policy/mimic/motion_playback.hpp"
#include "ai_sapiens_sim2real/policy/mimic/motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/mimic/motion_source.hpp"

using ai_sapiens_sim2real::MotionPlayback;
using ai_sapiens_sim2real::MotionReference;

TEST(MotionSource, LoadsOnceSharesFramesAndKeepsCursorsIndependent)
{
  char path[] = "/tmp/motion_source_test_XXXXXX";
  const int fd = mkstemp(path);
  ASSERT_GE(fd, 0);
  close(fd);
  struct Cleanup
  {
    const char * path;
    ~Cleanup() {std::remove(path);}
  } cleanup{path};
  std::ofstream(path) << "0,0,0,0,0,0,1,0\n0,0,0,0,0,0,1,1\n";
  ai_sapiens_sim2real::MotionSource source(path, 50, {"joint"});
  auto first = source.make_cursor();
  std::ofstream(path) << "not a motion\n";  // Subsequent cursor must not re-read.
  auto second = source.make_cursor();
  EXPECT_EQ(&first->joint_order(), &second->joint_order());
  first->seek(first->duration());
  EXPECT_FLOAT_EQ(first->joint_pos()[0], 1);
  EXPECT_FLOAT_EQ(second->joint_pos()[0], 0);
}

TEST(MotionPlayback, OpenEndedWindowNeverCompletes)
{
  MotionPlayback playback;  // time_start=0, time_end=inf
  EXPECT_FLOAT_EQ(playback.seek_time(0.0).value(), 0.0f);
  EXPECT_FLOAT_EQ(playback.seek_time(100.0).value(), 100.0f);
}

TEST(MotionPlayback, CompletesAtWindowEndFromZeroStart)
{
  MotionPlayback playback;
  playback.time_end = 5.0f;
  EXPECT_FLOAT_EQ(playback.seek_time(4.0).value(), 4.0f);
  EXPECT_FLOAT_EQ(playback.seek_time(5.0).value(), 5.0f);  // boundary still plays
  EXPECT_FALSE(playback.seek_time(5.0200000000000005).has_value());
}

TEST(MotionPlayback, NonZeroStartSeeksAndCompletesInAbsoluteMotionTime)
{
  MotionPlayback playback;
  playback.time_start = 3.0f;
  playback.time_end = 10.0f;  // play absolute motion times [3, 10]

  EXPECT_FLOAT_EQ(playback.seek_time(0.0).value(), 3.0f);   // starts at time_start
  EXPECT_FLOAT_EQ(playback.seek_time(6.0).value(), 9.0f);   // 3 + 6
  EXPECT_FLOAT_EQ(playback.seek_time(7.0).value(), 10.0f);  // 3 + 7 == time_end, still plays
  EXPECT_FALSE(playback.seek_time(7.5).has_value());        // 3 + 7.5 > time_end -> complete
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

TEST(MotionPlayback, IncludesEveryFrameExactlyOnceAndThenCompletes)
{
  for (int count : {1, 2, 3, 299, 300, 2659}) {
    MotionPlayback playback;
    playback.time_end = (count - 1) / 50.0;
    double episode_time = 0.0;
    for (int i = 0; i < count; ++i) {
      const auto time = playback.seek_time(episode_time);
      ASSERT_TRUE(time.has_value());
      EXPECT_NEAR(*time, i / 50.0, 1e-10);
      episode_time += playback.step_dt;
    }
    EXPECT_FALSE(playback.seek_time(episode_time));
  }
}

TEST(MotionPlayback, NonZeroWindowAndFloatEndpointsDoNotLoseLastFrame)
{
  MotionPlayback playback;
  playback.time_start = 0.2f;
  playback.time_end = 53.16f;
  const auto endpoint = playback.seek_time(52.96);
  ASSERT_TRUE(endpoint);
  EXPECT_DOUBLE_EQ(*endpoint, playback.time_end);
  EXPECT_FALSE(playback.seek_time(52.980000000000004));
  playback.reset();
  playback.time_start = 0.0;
  playback.time_end = 0.025;
  EXPECT_DOUBLE_EQ(*playback.seek_time(0.02), 0.02);
  EXPECT_DOUBLE_EQ(*playback.seek_time(0.04), 0.025);
  EXPECT_FALSE(playback.seek_time(0.06));
}

TEST(MotionReference, CentralVelocitiesAndNonlinearFrameValuesMatchTraining)
{
  char path[] = "/tmp/motion_central_test_XXXXXX";
  const int fd = mkstemp(path);
  ASSERT_GE(fd, 0);
  close(fd);
  struct Cleanup {const char * path; ~Cleanup() {std::remove(path);}} cleanup{path};
  for (int count : {1, 2, 3, 299, 300, 2659}) {
    std::ofstream out(path);
    for (int i = 0; i < count; ++i) {
      out << "0,0,0,0,0,0,1," << i * i << "\n";
    }
    out.close();
    MotionReference reference(path, 50, {"joint"});
    EXPECT_NEAR(reference.duration(), (count - 1) / 50.0, 1e-12);
    MotionPlayback playback;
    playback.time_end = reference.duration();
    double episode_time = 0.0;
    for (int i = 0; i < count; ++i) {
      const auto time = playback.seek_time(episode_time);
      ASSERT_TRUE(time);
      reference.seek(*time);
      episode_time += playback.step_dt;
      EXPECT_FLOAT_EQ(reference.joint_pos()[0], static_cast<float>(i * i));
      const float expected_velocity = count == 1 ? 0.0f :
        (i == 0 ? 50.0f : (i == count - 1 ? (2 * i - 1) * 50.0f : i * 100.0f));
      EXPECT_NEAR(reference.joint_vel()[0], expected_velocity, 0.05f);
    }
    EXPECT_FALSE(playback.seek_time(episode_time));
    if (count >= 3) {
      reference.seek(0.03);
      EXPECT_NEAR(reference.joint_pos()[0], 2.5f, 1e-5f);
    }
  }
}

TEST(MotionPlayback, ResetRearmsEndpointAndStayDoesNotRepeatIt)
{
  MotionPlayback playback;
  playback.time_end = 0.04;
  EXPECT_DOUBLE_EQ(*playback.seek_time(0.0), 0.0);
  EXPECT_DOUBLE_EQ(*playback.seek_time(0.02), 0.02);
  EXPECT_DOUBLE_EQ(*playback.seek_time(0.04), 0.04);
  EXPECT_FALSE(playback.seek_time(0.06));
  EXPECT_FALSE(playback.seek_time(0.06));
  playback.reset();
  EXPECT_DOUBLE_EQ(*playback.seek_time(0.0), 0.0);
  EXPECT_DOUBLE_EQ(*playback.seek_time(0.04), 0.04);
  EXPECT_FALSE(playback.seek_time(0.06));
}

TEST(MotionReference, DifferentMotionRatesUseEpisodeTimeAndIncludeEndpoint)
{
  char path[] = "/tmp/motion_rates_test_XXXXXX";
  const int fd = mkstemp(path);
  ASSERT_GE(fd, 0);
  close(fd);
  struct Cleanup {const char * path; ~Cleanup() {std::remove(path);}} cleanup{path};

  for (float fps : {30.0f, 60.0f, 100.0f}) {
    SCOPED_TRACE(fps);
    std::ofstream out(path);
    for (int frame = 0; frame < 5; ++frame) {
      out << "0,0,0,0,0,0,1," << frame << "\n";
    }
    out.close();

    MotionReference reference(path, fps, {"joint"});
    MotionPlayback playback;
    playback.time_end = reference.duration();
    double episode_time = 0.0;
    int endpoint_count = 0;

    for (int tick = 0; tick < 20; ++tick) {
      const auto time = playback.seek_time(episode_time);
      if (!time) {
        break;
      }

      reference.seek(*time);
      EXPECT_NEAR(reference.joint_pos()[0], std::min(episode_time * fps, 4.0), 1e-5);
      EXPECT_NEAR(reference.joint_vel()[0], fps, 1e-4);
      if (*time == playback.time_end) {
        ++endpoint_count;
      }
      episode_time += playback.step_dt;
    }

    EXPECT_EQ(endpoint_count, 1);
    EXPECT_FALSE(playback.seek_time(episode_time));
  }
}
