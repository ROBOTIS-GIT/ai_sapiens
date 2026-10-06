// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "ai_sapiens_depth/depth_processor.hpp"
using ai_sapiens_depth::DepthConfig;
using ai_sapiens_depth::DepthHistory;
using ai_sapiens_depth::preprocess;
TEST(Depth, ClampCropAndReplicatedGaussian)
{
  DepthConfig c;
  std::vector<float> raw(64 * 36, c.near);
  raw[18 * 64 + 16] = c.far;
  raw[17 * 64 + 16] = c.far;  // Outside the crop: must not blur into it.
  const auto out = preprocess(raw, c, false);
  const double side = std::exp(-0.5) / (1 + 2 * std::exp(-0.5));
  EXPECT_NEAR(out[0], (1 - side) * (1 - side), 1e-7);
  EXPECT_NEAR(out[1], side * (1 - side), 1e-7);
  EXPECT_NEAR(out[32], side * (1 - side), 1e-7);
  EXPECT_NEAR(out[2], 0, 1e-7);
  EXPECT_EQ(out.size(), 576U);
}
TEST(Depth, InvalidSimulationAndRealHoles)
{
  DepthConfig c;
  std::vector<float> raw(64 * 36, std::numeric_limits<float>::quiet_NaN());
  auto sim = preprocess(raw, c, false);
  EXPECT_FLOAT_EQ(sim[0], 1);
  auto real = preprocess(raw, c, true);
  EXPECT_FLOAT_EQ(real[0], 1);
  raw[0] = c.near;
  real = preprocess(raw, c, true);
  EXPECT_NEAR(real.back(), 0, 1e-7);
  sim = preprocess(raw, c, false);
  EXPECT_FLOAT_EQ(sim.back(), 1);
}
TEST(Depth, HistoryTimestampStrideDelayAndDuplicates)
{
  DepthHistory history(DepthConfig{});
  for (int i = 0; i <= 40; ++i) {
    ASSERT_TRUE(history.push(std::vector<float>(576, i / 100.0f), 10 + i * 0.02));
                                                                                                         }
  const auto zero = history.sample(0), delayed = history.sample(1);
  for (int i = 0; i < 8; ++i) {
    EXPECT_NEAR(zero.data[i * 576], (5 + i * 5) / 100.0, 1e-7);
    EXPECT_NEAR(delayed.data[i * 576], (4 + i * 5) / 100.0, 1e-7);
  }
  EXPECT_FALSE(history.push(std::vector<float>(576, 1), 10.8));
  EXPECT_NEAR(history.sample().data.back(), 0.4, 1e-7);
}
TEST(Depth, WarmFillRateChangeGapAndReset)
{
  DepthHistory history(DepthConfig{});
  history.push(std::vector<float>(576, 0.2), 2.0);
  for (double stamp : history.sample().frame_stamps) {
    EXPECT_DOUBLE_EQ(stamp, 2.0);
                                                                                   }
  for (int i = 1; i <= 30; ++i) {
    history.push(std::vector<float>(576, i / 100.0f), 2 + i / 30.0);
                                                                                           }
  const auto out = history.sample();
  for (int i = 0; i < 8; ++i) {
    EXPECT_NEAR(out.frame_stamps[i], 2.3 + i * 0.1, 1e-8);
                                                                                 }
  history.push(std::vector<float>(576, 0.9), 5.0);
  EXPECT_FLOAT_EQ(history.sample().data.front(), 0.9f);
  history.push(std::vector<float>(576, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(history.sample().frame_stamps.front(), 1.0);
  EXPECT_THROW(history.push(std::vector<float>(576, NAN), 1.02), std::runtime_error);
}
TEST(Depth, SimulationHistoryMatchesFrameStrideDespiteTimestampJitter)
{
  // A short interval makes timestamp - 20 ms skip the previous frame.
  // Exercise all eight history slots, both delays, warm-fill and eviction.
  for (double epoch : {10.0, 1791268000.0}) {
    DepthHistory history(DepthConfig{}, DepthHistory::Sampling::FrameIndex);
    double previous_latest = -1;
    for (int tick = 0; tick < 100; ++tick) {
      const double stamp = epoch + tick * 0.02 + (tick % 2 == 0 ? 0.002 : 0.0);
      ASSERT_TRUE(history.push(std::vector<float>(576, tick / 100.0f), stamp));
      for (int delay = 0; delay <= 1; ++delay) {
        const auto out = history.sample(delay);
        for (int frame = 0; frame < 8; ++frame) {
          const int selected = std::max(0, tick - delay - (7 - frame) * 5);
          ASSERT_FLOAT_EQ(out.data[frame * 576], selected / 100.0f)
            << "tick=" << tick << " delay=" << delay << " frame=" << frame;
          EXPECT_DOUBLE_EQ(out.frame_stamps[frame],
            epoch + selected * 0.02 + (selected % 2 == 0 ? 0.002 : 0.0));
        }
      }
      // Alternating zero/one-frame delay may repeat a frame, never go backwards.
      const auto latest = history.sample(tick % 2).frame_stamps.back();
      EXPECT_GE(latest, previous_latest);
      previous_latest = latest;
    }
  }
}
TEST(Depth, SimulationHistoryRejectsDuplicateAndResetsOnGapOrClockReset)
{
  DepthHistory history(DepthConfig{}, DepthHistory::Sampling::FrameIndex);
  ASSERT_TRUE(history.push(std::vector<float>(576, 0.1f), 2.0));
  ASSERT_TRUE(history.push(std::vector<float>(576, 0.2f), 2.02));
  EXPECT_FALSE(history.push(std::vector<float>(576, 0.9f), 2.02));
  EXPECT_FLOAT_EQ(history.sample(1).data.back(), 0.1f);
  ASSERT_TRUE(history.push(std::vector<float>(576, 0.3f), 3.0));
  for (float value : history.sample(1).data) {
    EXPECT_FLOAT_EQ(value, 0.3f);
  }
  ASSERT_TRUE(history.push(std::vector<float>(576, 0.4f), 0.0));
  for (double stamp : history.sample(1).frame_stamps) {
    EXPECT_DOUBLE_EQ(stamp, 0.0);
  }
}
TEST(Depth, RectifiedMetricImageEndianAndCoverage)
{
  DepthConfig c;
  sensor_msgs::msg::Image image;
  image.header.frame_id = "depth"; image.width = 64; image.height = 36;
  image.encoding = "16UC1"; image.step = 132; image.is_bigendian = true;
  image.data.resize(image.step * image.height);
  for (unsigned y = 0; y < image.height; ++y) {
    for (unsigned x = 0; x < image.width; ++x) {
      image.data[y * image.step + 2 * x] = 0x03;
      image.data[y * image.step + 2 * x + 1] = 0xe8;
    }
  }
  sensor_msgs::msg::CameraInfo info;
  info.header = image.header; info.width = 64; info.height = 36;
  info.p[0] = 64 / (2 * std::tan(87 * std::acos(-1.0) / 360));
  info.p[5] = 36 / (2 * std::tan(58 * std::acos(-1.0) / 360));
  info.p[2] = 31.5; info.p[6] = 17.5;
  auto metric = ai_sapiens_depth::metric_image(image, info, c, 0.001);
  EXPECT_FLOAT_EQ(metric[0], 1.0); EXPECT_FLOAT_EQ(metric.back(), 1.0);
  image.is_bigendian = false;
  metric = ai_sapiens_depth::metric_image(image, info, c, 0.001);
  EXPECT_NEAR(metric[0], 59.395, 1e-5);
  info.p[0] *= 2;
  EXPECT_THROW(ai_sapiens_depth::metric_image(image, info, c, 0.001), std::runtime_error);
  image.data.clear();
  EXPECT_THROW(ai_sapiens_depth::metric_image(image, info, c, 0.001), std::runtime_error);
}
