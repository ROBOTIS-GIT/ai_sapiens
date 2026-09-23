#include "ai_sapiens_sim2real/policy/policy_entry_validator.hpp"

#include "ai_sapiens_sim2real/config/root_config.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/motion_source.hpp"

namespace ai_sapiens_sim2real
{
PolicyEntryValidator PolicyEntryValidator::from_config(const RootConfig & config)
{
  References references;
  for (const auto & behavior : config.policy_behaviors()) {
    if (!behavior.mimic) {
      continue;
    }
    const auto & mimic = *behavior.mimic;
    auto motion = mimic.source->make_cursor();
    motion->seek(mimic.time_start);
    references.emplace(behavior.name, motion->root_quaternion());
  }
  return PolicyEntryValidator(std::move(references));
}
}  // namespace ai_sapiens_sim2real
