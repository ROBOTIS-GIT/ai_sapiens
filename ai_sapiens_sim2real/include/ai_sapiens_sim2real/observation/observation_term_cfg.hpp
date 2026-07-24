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
//
// Author: Woojin Wie

#ifndef AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_TERM_CFG_HPP_
#define AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_TERM_CFG_HPP_

#include <algorithm>
#include <deque>
#include <functional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

// Forward declaration
class MotionReference;
struct PolicyJointContext;
struct SharedControlData;

struct ObservationContext
{
  const SharedControlData & shared;
  const PolicyJointContext & joints;
  const MotionReference * reference_motion{nullptr};
};

/**
 * @brief Configuration and history buffer for a single observation term.
 *
 * Handles clip/scale processing and maintains a history buffer for
 * temporal observations.
 */
struct ObservationTermCfg
{
  /// Observation function type: reads from the active policy observation context.
  using ObsFunc =
    std::function<std::vector<float>(const ObservationContext &, const YAML::Node &)>;

  // Static configuration parsed from YAML.
  std::string name;
  YAML::Node params;
  std::vector<float> clip;  ///< [min, max] for clipping
  std::vector<float> scale;  ///< Per-element scale factors
  int history_length = 1;
  bool scale_first = false;  ///< false = clip first (isaaclab default)
  ObsFunc func;

  /**
   * @brief Reset the history buffer with initial observation.
   * @param obs Initial observation to fill history
   */
  void reset(const std::vector<float> & obs)
  {
    buffer_.clear();
    for (int i = 0; i < history_length; ++i) {
      add(obs);
    }
  }

  /**
   * @brief Add new observation to history with clip/scale processing.
   * @param obs Raw observation values
   */
  void add(std::vector<float> obs)
  {
    // Apply clip and scale in correct order
    for (size_t j = 0; j < obs.size(); ++j) {
      if (scale_first) {
        // Scale first, then clip
        if (!scale.empty() && j < scale.size()) {
          obs[j] *= scale[j];
        }
        if (!clip.empty() && clip.size() >= 2) {
          obs[j] = std::clamp(obs[j], clip[0], clip[1]);
        }
      } else {
        // Clip first, then scale (isaaclab default)
        if (!clip.empty() && clip.size() >= 2) {
          obs[j] = std::clamp(obs[j], clip[0], clip[1]);
        }
        if (!scale.empty() && j < scale.size()) {
          obs[j] *= scale[j];
        }
      }
    }

    buffer_.push_back(obs);

    // Maintain history length
    while (static_cast<int>(buffer_.size()) > history_length) {
      buffer_.pop_front();
    }
  }

  /**
   * @brief Get specific history entry.
   * @param n History index (0 = oldest)
   * @return Observation at history position n
   */
  const std::vector<float> & get(int n) const
  {
    return buffer_[n];
  }

  /**
   * @brief Get concatenated history (oldest to newest).
   * @return All history entries concatenated
   */
  std::vector<float> get() const
  {
    std::vector<float> result;
    result.reserve(size());
    append_to(result);
    return result;
  }

  void append_to(std::vector<float> & output) const
  {
    for (const auto & entry : buffer_) {
      output.insert(output.end(), entry.begin(), entry.end());
    }
  }

  /**
   * @brief Get total size of concatenated history.
   * @return Total number of floats in all history entries
   */
  size_t size() const
  {
    size_t total = 0;
    for (const auto & entry : buffer_) {
      total += entry.size();
    }

    return total;
  }

  /**
   * @brief Get size of single observation (without history).
   * @return Size of one observation entry
   */
  size_t single_size() const
  {
    if (buffer_.empty()) {
      return 0U;
    }

    return buffer_.front().size();
  }

private:
  /// Circular buffer for history (oldest at front, newest at back).
  std::deque<std::vector<float>> buffer_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_TERM_CFG_HPP_
