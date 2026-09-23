#ifndef AI_SAPIENS_SIM2REAL__POLICY__MIMIC_ENTRY_GUARD_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MIMIC_ENTRY_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Geometry>

namespace ai_sapiens_sim2real
{

// Both orientations must map the same robot root frame into a gravity-aligned
// world frame. Independent world headings cancel in projected gravity.
struct MimicEntryCheck
{
  bool allowed;
  float tilt_error_deg;
};

inline constexpr float kMimicEntryTiltLimitDeg = 35.0f;

inline MimicEntryCheck check_gravity_alignment(
  const Eigen::Quaternionf & reference, const Eigen::Quaternionf & actual,
  float tilt_limit_deg)
{
  for (const auto * q : {&reference, &actual}) {
    if (!q->coeffs().allFinite() || !std::isfinite(q->squaredNorm()) ||
      q->squaredNorm() < 1e-12f)
    {
      return {false, std::numeric_limits<float>::quiet_NaN()};
    }
  }
  const Eigen::Vector3f gravity = -Eigen::Vector3f::UnitZ();
  const auto expected = reference.normalized().conjugate() * gravity;
  const auto measured = actual.normalized().conjugate() * gravity;
  const float error = std::acos(std::clamp(expected.dot(measured), -1.0f, 1.0f)) *
    (180.0f / 3.14159265358979323846f);
  return {error <= tilt_limit_deg, error};
}

inline MimicEntryCheck check_mimic_entry_tilt(
  const Eigen::Quaternionf & reference, const Eigen::Quaternionf & actual)
{
  return check_gravity_alignment(reference, actual, kMimicEntryTiltLimitDeg);
}

}  // namespace ai_sapiens_sim2real
#endif
