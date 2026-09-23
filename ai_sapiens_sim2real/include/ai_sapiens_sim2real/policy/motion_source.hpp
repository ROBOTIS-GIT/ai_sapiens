#ifndef AI_SAPIENS_SIM2REAL__POLICY__MOTION_SOURCE_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MOTION_SOURCE_HPP_

#include <memory>
#include <mutex>
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"

namespace ai_sapiens_sim2real
{
// Configuration-scoped asset: one parse/derivative computation, independent
// playback cursors sharing read-only frame data. No process-global cache.
class MotionSource
{
public:
  MotionSource(std::string file, float fps, std::vector<std::string> joints)
  : file_(std::move(file)), fps_(fps), joints_(std::move(joints)) {}

  std::shared_ptr<MotionReference> make_cursor() const
  {
    std::call_once(loaded_, [this]() {
        reference_ = std::make_unique<const MotionReference>(file_, fps_, joints_);
      });
    return std::make_shared<MotionReference>(*reference_);
  }

private:
  std::string file_;
  float fps_;
  std::vector<std::string> joints_;
  mutable std::once_flag loaded_;
  mutable std::unique_ptr<const MotionReference> reference_;
};
}  // namespace ai_sapiens_sim2real
#endif
