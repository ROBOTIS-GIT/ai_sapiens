#include <gtest/gtest.h>
#include "ai_sapiens_sim2real/policy/mimic_heading_alignment.hpp"

using ai_sapiens_sim2real::mimic_heading_alignment;
TEST(MimicHeadingAlignment, StandingSupineProneAndNonzeroWaist)
{
  for (float pitch : {0.0f, 1.5707963f, -1.5707963f, 1.569f, 1.572f}) {
    for (float yaw : {0.0f, 0.7f, -2.8f}) {
      const Eigen::Quaternionf reference = Eigen::AngleAxisf(-0.8f, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitZ());
      const Eigen::Quaternionf expected(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
      Eigen::Quaternionf actual = expected * reference;
      EXPECT_LT(mimic_heading_alignment(reference, actual).angularDistance(expected), 1e-5f);
      actual.coeffs() *= -1;
      EXPECT_LT(mimic_heading_alignment(reference, actual).angularDistance(expected), 1e-5f);
    }
  }
}
TEST(MimicHeadingAlignment, DoesNotCancelTiltError)
{
  const Eigen::Quaternionf reference(Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitY()));
  const Eigen::Quaternionf actual = Eigen::AngleAxisf(0.6f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(0.12f, Eigen::Vector3f::UnitX()) * reference;
  const auto alignment = mimic_heading_alignment(reference, actual);
  EXPECT_NEAR(alignment.x(), 0, 1e-6);
  EXPECT_NEAR(alignment.y(), 0, 1e-6);
  EXPECT_GT((alignment * reference).angularDistance(actual), 0.1f);
}
TEST(MimicHeadingAlignment, OpposingTiltIsFinite)
{
  const Eigen::Quaternionf actual(0,1,0,0);
  EXPECT_TRUE(mimic_heading_alignment(Eigen::Quaternionf::Identity(),actual).isApprox(Eigen::Quaternionf::Identity()));
}
