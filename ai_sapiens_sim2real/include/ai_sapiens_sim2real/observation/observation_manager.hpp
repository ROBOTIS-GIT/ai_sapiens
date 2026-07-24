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

#ifndef AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_MANAGER_HPP_
#define AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_MANAGER_HPP_

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/observation/observation_term_cfg.hpp"
#include "ai_sapiens_sim2real/observation/observations.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

/**
 * @brief Dynamically manages observations based on YAML configuration.
 *
 * Parses sim2real.yaml observations section, creates ObservationTermCfg for each,
 * and computes the full observation vector in YAML order with clip/scale/history.
 */
class ObservationManager
{
public:
  /**
   * @brief Construct observation manager from YAML config.
   * @param obs_config The "observations" section from sim2real.yaml
   * @param shared_data Pointer to shared state data
   * @param joint_context Active policy joint order and controller mapping
   * @param reference_motion Reference motion for motion observations, or nullptr
   */
  ObservationManager(
    const YAML::Node & obs_config,
    const SharedControlData * shared_data,
    const PolicyJointContext & joint_context,
    const MotionReference * reference_motion);

  /**
   * @brief Reset all observation history buffers.
   *
   * Should be called when entering active state.
   */
  void reset();

  /**
   * @brief Compute full observation vector.
   *
   * Called in RT loop. Computes each observation, applies clip/scale,
   * and concatenates in YAML order.
   *
   * @return Concatenated observation vector
   */
  std::vector<float> compute();

  /**
   * @brief Get total observation size.
   * @return Total number of floats in observation vector
   */
  size_t get_observation_size() const;

  /**
   * @brief Get number of observation terms.
   * @return Number of registered observations
   */
  size_t get_num_terms() const
  {
    return terms_.size();
  }

  /**
   * @brief Get observation term names in order.
   * @return Vector of observation names
   */
  std::vector<std::string> get_term_names() const;

private:
  // Config parsing creates ordered terms; compute() reuses obs_buffer_ in that order.
  void parse_config(const YAML::Node & obs_config);
  ObservationTermCfg parse_observation_term(
    const std::string & term_name,
    const YAML::Node & term_config,
    bool global_scale_first) const;
  ObservationTermCfg::ObsFunc observation_function_for(const std::string & term_name) const;
  static bool is_reserved_config_key(const std::string & term_name);
  static void read_term_params(ObservationTermCfg & term, const YAML::Node & term_config);
  static void read_term_clip(ObservationTermCfg & term, const YAML::Node & term_config);
  static void read_term_scale(ObservationTermCfg & term, const YAML::Node & term_config);
  static void read_term_history_length(ObservationTermCfg & term, const YAML::Node & term_config);
  static void read_term_scale_order(
    ObservationTermCfg & term,
    const YAML::Node & term_config,
    bool global_scale_first);
  void initialize_term_history(ObservationTermCfg & term) const;
  ObservationContext make_observation_context() const;

  const SharedControlData * shared_data_;
  PolicyJointContext joint_context_;
  const MotionReference * reference_motion_;
  std::vector<ObservationTermCfg> terms_;
  std::vector<float> obs_buffer_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_MANAGER_HPP_
