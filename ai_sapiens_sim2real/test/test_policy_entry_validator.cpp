#include <gtest/gtest.h>
#include "ai_sapiens_sim2real/policy/policy_entry_validator.hpp"

using namespace ai_sapiens_sim2real;

TEST(PolicyEntryValidator, PostureToNonMimicRequiresUprightOrientation)
{
  PolicyEntryValidator validator({});
  StateBehavior target;
  const auto pitch = [](float degrees) {
      return Eigen::Quaternionf(Eigen::AngleAxisf(
        degrees * 3.14159265358979323846f / 180.0f, Eigen::Vector3f::UnitY()));
    };
  for (float degrees : {0.0f, 36.2f, 44.9f}) {
    EXPECT_TRUE(validator.evaluate_transition(
        BehaviorKind::Posture, target, pitch(degrees), true).allowed());
  }
  for (float degrees : {45.1f, 90.0f, -90.0f, 180.0f}) {
    const auto result = validator.evaluate_transition(
      BehaviorKind::Posture, target, pitch(degrees), true);
    EXPECT_EQ(result.reason, EntryRejection::TiltMismatch);
    EXPECT_FLOAT_EQ(result.tilt_limit_deg, 45.0f);
  }
  const auto sideways = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitX()));
  EXPECT_FALSE(validator.evaluate_transition(
      BehaviorKind::Posture, target, sideways, true).allowed());
  EXPECT_EQ(validator.evaluate_transition(
      BehaviorKind::Posture, target, pitch(0), false).reason, EntryRejection::ImuUnusable);
  EXPECT_EQ(validator.evaluate_transition(
      BehaviorKind::Posture, target, Eigen::Quaternionf(0, 0, 0, 0), true).reason,
    EntryRejection::InvalidOrientation);
  EXPECT_TRUE(validator.evaluate_transition(
      BehaviorKind::Policy, target, pitch(60), true).allowed());
}

TEST(PolicyEntryValidator, PostureToMimicStillUsesReferenceInsteadOfUpright)
{
  const auto supine = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.57079632679f, Eigen::Vector3f::UnitY()));
  PolicyEntryValidator validator({{"get_up", supine}});
  StateBehavior target;
  target.mimic = true;
  target.policy_name = "get_up";
  EXPECT_TRUE(validator.evaluate_transition(
      BehaviorKind::Posture, target, supine, true).allowed());
  EXPECT_FALSE(validator.evaluate_transition(
      BehaviorKind::Posture, target, Eigen::Quaternionf::Identity(), true).allowed());
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
