#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_VALIDATION_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__IMU_VALIDATION_HPP_

#include <cmath>
#include <Eigen/Geometry>

namespace ai_sapiens_sim2real
{
// Allow modest unit-length drift, not arbitrary vectors disguised as rotations.
inline bool valid_imu_sample(
  const Eigen::Quaternionf & orientation, const Eigen::Vector3f & angular_velocity,
  bool orientation_available, bool angular_velocity_available)
{
  if (!orientation_available || !angular_velocity_available || !orientation.coeffs().allFinite() ||
    !angular_velocity.allFinite())
  {
    return false;
  }
  const float norm = orientation.norm();
  return std::isfinite(norm) && norm >= 0.9f && norm <= 1.1f;
}
}  // namespace ai_sapiens_sim2real
#endif
