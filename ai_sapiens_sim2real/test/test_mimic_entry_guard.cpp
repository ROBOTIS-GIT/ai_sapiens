#include <gtest/gtest.h>
#include "ai_sapiens_sim2real/policy/mimic_entry_guard.hpp"

using ai_sapiens_sim2real::check_mimic_entry_tilt;
namespace
{
Eigen::Quaternionf rotation(float degrees, const Eigen::Vector3f & axis)
{
  return Eigen::Quaternionf(Eigen::AngleAxisf(degrees * 3.14159265358979323846f / 180, axis));
}
}

TEST(MimicEntryGuard, IgnoresHeadingForStandingSupineProneAndSide)
{
  for (const auto & pose : {Eigen::Quaternionf::Identity(),
    rotation(90, Eigen::Vector3f::UnitY()), rotation(-90, Eigen::Vector3f::UnitY()),
    rotation(90, Eigen::Vector3f::UnitX())})
  {
    EXPECT_TRUE(check_mimic_entry_tilt(pose, rotation(137, Eigen::Vector3f::UnitZ()) * pose).allowed);
  }
}

TEST(MimicEntryGuard, RejectsWrongPostureAndChecksRelativeTilt)
{
  const auto upright = Eigen::Quaternionf::Identity();
  EXPECT_TRUE(check_mimic_entry_tilt(upright, rotation(34.9f, Eigen::Vector3f::UnitX())).allowed);
  EXPECT_FALSE(check_mimic_entry_tilt(upright, rotation(35.1f, Eigen::Vector3f::UnitX())).allowed);
  EXPECT_FALSE(check_mimic_entry_tilt(rotation(-30, Eigen::Vector3f::UnitX()),
    rotation(30, Eigen::Vector3f::UnitX())).allowed);
  EXPECT_FALSE(check_mimic_entry_tilt(rotation(90, Eigen::Vector3f::UnitY()),
    rotation(-90, Eigen::Vector3f::UnitY())).allowed);
}

TEST(MimicEntryGuard, NormalizesAndRejectsInvalidQuaternions)
{
  const auto upright = Eigen::Quaternionf::Identity();
  EXPECT_TRUE(check_mimic_entry_tilt(upright, Eigen::Quaternionf(-2, 0, 0, 0)).allowed);
  EXPECT_FALSE(check_mimic_entry_tilt(upright, Eigen::Quaternionf(0, 0, 0, 0)).allowed);
  EXPECT_FALSE(check_mimic_entry_tilt(upright,
    Eigen::Quaternionf(std::numeric_limits<float>::quiet_NaN(), 0, 0, 0)).allowed);
}
