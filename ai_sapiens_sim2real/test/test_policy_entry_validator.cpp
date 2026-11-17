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
#include "ai_sapiens_sim2real/policy/policy_entry_validator.hpp"

using namespace ai_sapiens_sim2real;

TEST(PolicyEntryValidator, NonMimicEntryDoesNotCheckOrientation)
{
  PolicyEntryValidator validator({});
  StateBehavior target;
  target.kind = BehaviorKind::Policy;
  target.mimic = false;
  for (float degrees : {0.0f, 45.1f, 90.0f, -90.0f, 180.0f}) {
    const Eigen::Quaternionf orientation(Eigen::AngleAxisf(
      degrees * 3.14159265358979323846f / 180.0f, Eigen::Vector3f::UnitY()));
    EXPECT_TRUE(validator.evaluate(target, orientation, true).allowed());
  }
  EXPECT_TRUE(validator.evaluate(target, Eigen::Quaternionf(0, 0, 0, 0), true).allowed());
  EXPECT_EQ(validator.evaluate(target, Eigen::Quaternionf::Identity(), false).reason,
    EntryRejection::ImuUnusable);
}

TEST(PolicyEntryValidator, MimicStillRequiresMatchingReferenceOrientation)
{
  const auto supine = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitY()));
  PolicyEntryValidator validator({{"get_up", supine}});
  StateBehavior target;
  target.mimic = true;
  target.policy_name = "get_up";
  EXPECT_TRUE(validator.evaluate(
      target, supine, true).allowed());
  EXPECT_FALSE(validator.evaluate(
      target, Eigen::Quaternionf::Identity(), true).allowed());
}

TEST(PolicyEntryValidator, DistinguishesRejectionReasonsWithoutAssets)
{
  PolicyEntryValidator validator({{"motion", Eigen::Quaternionf::Identity()}});
  StateBehavior behavior;
  EXPECT_TRUE(validator.evaluate(behavior, Eigen::Quaternionf::Identity(), true).allowed());
  EXPECT_EQ(validator.evaluate(behavior, Eigen::Quaternionf::Identity(), false).reason,
    EntryRejection::ImuUnusable);
  behavior.mimic = true;
  behavior.policy_name = "missing";
  EXPECT_EQ(validator.evaluate(behavior, Eigen::Quaternionf::Identity(), true).reason,
    EntryRejection::MissingReference);
  behavior.policy_name = "motion";
  EXPECT_EQ(validator.evaluate(behavior, Eigen::Quaternionf(0, 0, 0, 0), true).reason,
    EntryRejection::InvalidOrientation);
  const auto tilted = Eigen::Quaternionf(Eigen::AngleAxisf(1.0f, Eigen::Vector3f::UnitX()));
  const auto mismatch = validator.evaluate(behavior, tilted, true);
  EXPECT_EQ(mismatch.reason, EntryRejection::TiltMismatch);
  EXPECT_GT(mismatch.tilt_error_deg, mismatch.tilt_limit_deg);
  EXPECT_TRUE(validator.evaluate(behavior, Eigen::Quaternionf::Identity(), true).allowed());
  behavior.kind = BehaviorKind::Damping;
  behavior.mimic = false;
  EXPECT_TRUE(validator.evaluate(behavior, Eigen::Quaternionf::Identity(), false).allowed());
}
