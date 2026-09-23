#ifndef AI_SAPIENS_SIM2REAL__POLICY__POLICY_ENTRY_VALIDATOR_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__POLICY_ENTRY_VALIDATOR_HPP_

#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <Eigen/Geometry>
#include "ai_sapiens_sim2real/config/root_sections/state_behaviors_config.hpp"
#include "ai_sapiens_sim2real/policy/mimic_entry_guard.hpp"

namespace ai_sapiens_sim2real
{
class RootConfig;

inline constexpr float kPostureToPolicyTiltLimitDeg = 45.0f;

enum class EntryRejection
{
  None, TransitionNotAllowed, MissingReference, ImuUnusable, InvalidOrientation, TiltMismatch
};

struct PolicyEntryResult
{
  EntryRejection reason{EntryRejection::None};
  float tilt_error_deg{std::numeric_limits<float>::quiet_NaN()};
  float tilt_limit_deg{kMimicEntryTiltLimitDeg};
  bool allowed() const {return reason == EntryRejection::None;}
};

inline const char * entry_rejection_name(EntryRejection reason)
{
  switch (reason) {
    case EntryRejection::None: return "none";
    case EntryRejection::TransitionNotAllowed: return "transition_not_allowed";
    case EntryRejection::MissingReference: return "missing_reference";
    case EntryRejection::ImuUnusable: return "imu_unusable";
    case EntryRejection::InvalidOrientation: return "invalid_orientation";
    case EntryRejection::TiltMismatch: return "tilt_mismatch";
  }
  return "unknown";
}

// Owns immutable entry metadata, not playback. Evaluation does no I/O/logging.
class PolicyEntryValidator
{
public:
  using References = std::unordered_map<std::string, Eigen::Quaternionf>;
  explicit PolicyEntryValidator(References references) : references_(std::move(references)) {}
  static PolicyEntryValidator from_config(const RootConfig & config);
  PolicyEntryResult evaluate_transition(
    BehaviorKind source_kind, const StateBehavior & target,
    const Eigen::Quaternionf & orientation, bool imu_usable) const
  {
    const auto entry = evaluate(target, orientation, imu_usable);
    if (!entry.allowed() || source_kind != BehaviorKind::Posture ||
      target.kind != BehaviorKind::Policy || target.mimic)
    {
      return entry;
    }

    // Non-mimic policies started from posture control require an upright root.
    // Policy-to-policy handoffs retain their existing entry checks.
    const auto check = check_gravity_alignment(
      Eigen::Quaternionf::Identity(), orientation, kPostureToPolicyTiltLimitDeg);
    if (!std::isfinite(check.tilt_error_deg)) {
      return {EntryRejection::InvalidOrientation};
    }
    return {check.allowed ? EntryRejection::None : EntryRejection::TiltMismatch,
      check.tilt_error_deg, kPostureToPolicyTiltLimitDeg};
  }

  PolicyEntryResult evaluate(
    const StateBehavior & behavior, const Eigen::Quaternionf & orientation,
    bool imu_usable) const
  {
    if (behavior.kind == BehaviorKind::Policy && !imu_usable) {
      return {EntryRejection::ImuUnusable};
    }
    if (!behavior.mimic) {
      return {};
    }
    const auto reference = references_.find(behavior.policy_name);
    if (reference == references_.end()) {
      return {EntryRejection::MissingReference};
    }
    const auto check = check_mimic_entry_tilt(reference->second, orientation);
    if (!std::isfinite(check.tilt_error_deg)) {
      return {EntryRejection::InvalidOrientation};
    }
    return {check.allowed ? EntryRejection::None : EntryRejection::TiltMismatch,
      check.tilt_error_deg};
  }

private:
  References references_;
};
}  // namespace ai_sapiens_sim2real
#endif
