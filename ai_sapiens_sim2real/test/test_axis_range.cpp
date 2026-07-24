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

#include "ai_sapiens_sim2real/axis_range.hpp"

using ai_sapiens_sim2real::AxisRange;
using ai_sapiens_sim2real::clamp_to_axis;
using ai_sapiens_sim2real::normalize_axis_to_unit;
using ai_sapiens_sim2real::scale_unit_to_axis;

TEST(AxisRange, NormalizePreservesZeroAndMapsBoundsToUnit)
{
  const AxisRange declared{-2.0, 4.0};  // asymmetric
  EXPECT_FLOAT_EQ(normalize_axis_to_unit(0.0, declared), 0.0f);
  EXPECT_FLOAT_EQ(normalize_axis_to_unit(4.0, declared), 1.0f);   // positive bound
  EXPECT_FLOAT_EQ(normalize_axis_to_unit(-2.0, declared), -1.0f);  // negative bound
  EXPECT_FLOAT_EQ(normalize_axis_to_unit(2.0, declared), 0.5f);
}

TEST(AxisRange, ScalePreservesZeroAndMapsUnitToBounds)
{
  const AxisRange active{-1.0, 3.0};  // asymmetric
  EXPECT_FLOAT_EQ(scale_unit_to_axis(0.0f, active), 0.0f);
  EXPECT_FLOAT_EQ(scale_unit_to_axis(1.0f, active), 3.0f);
  EXPECT_FLOAT_EQ(scale_unit_to_axis(-1.0f, active), -1.0f);
  EXPECT_FLOAT_EQ(scale_unit_to_axis(0.5f, active), 1.5f);
}

// normalize then scale reproduces a direct declared->active proportional map,
// independently per side, even when both ranges are asymmetric.
TEST(AxisRange, NormalizeThenScaleMapsDeclaredOntoActive)
{
  const AxisRange declared{-2.0, 4.0};
  const AxisRange active{-1.0, 3.0};

  EXPECT_FLOAT_EQ(scale_unit_to_axis(normalize_axis_to_unit(2.0, declared), active), 1.5f);
  EXPECT_FLOAT_EQ(scale_unit_to_axis(normalize_axis_to_unit(-1.0, declared), active), -0.5f);
}

TEST(AxisRange, ClampBoundsValueToRange)
{
  const AxisRange active{-1.0, 2.0};
  EXPECT_FLOAT_EQ(clamp_to_axis(0.5, active), 0.5f);
  EXPECT_FLOAT_EQ(clamp_to_axis(5.0, active), 2.0f);
  EXPECT_FLOAT_EQ(clamp_to_axis(-3.0, active), -1.0f);
}
