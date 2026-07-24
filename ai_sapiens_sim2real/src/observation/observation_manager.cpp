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

#include "ai_sapiens_sim2real/observation/observation_manager.hpp"

namespace ai_sapiens_sim2real
{

ObservationManager::ObservationManager(
  const YAML::Node & obs_config,
  const SharedControlData * shared_data,
  const PolicyJointContext & joint_context,
  const MotionReference * reference_motion)
: shared_data_(shared_data)
  , joint_context_(joint_context)
  , reference_motion_(reference_motion)
{
  if (shared_data_ == nullptr) {
    throw std::runtime_error("ObservationManager requires shared_data");
  }

  parse_config(obs_config);
}

void ObservationManager::reset()
{
  const auto context = make_observation_context();
  for (auto & term : terms_) {
    auto obs = term.func(context, term.params);
    term.reset(obs);
  }
}

std::vector<float> ObservationManager::compute()
{
  obs_buffer_.clear();
  const auto context = make_observation_context();

  for (auto & term : terms_) {
    // Get raw observation from the active policy context.
    auto obs = term.func(context, term.params);

    // Add to history with clip/scale processing
    term.add(obs);

    // Append concatenated history to output
    term.append_to(obs_buffer_);
  }

  return obs_buffer_;
}

size_t ObservationManager::get_observation_size() const
{
  size_t total = 0;
  for (const auto & term : terms_) {
    total += term.size();
  }

  return total;
}

std::vector<std::string> ObservationManager::get_term_names() const
{
  std::vector<std::string> names;
  for (const auto & term : terms_) {
    names.push_back(term.name);
  }

  return names;
}

void ObservationManager::parse_config(const YAML::Node & obs_config)
{
  if (!obs_config.IsDefined() || obs_config.IsNull()) {
    throw std::runtime_error("Observations config is null or undefined");
  }
  if (!obs_config.IsMap()) {
    throw std::runtime_error("Observations config must be a map");
  }

  bool should_scale_before_clip_by_default = false;
  const auto global_scale_first = obs_config["scale_first"];
  if (global_scale_first && !global_scale_first.IsNull()) {
    should_scale_before_clip_by_default = global_scale_first.as<bool>();
  }

  // Iterate in YAML order (preserves observation order)
  for (auto it = obs_config.begin(); it != obs_config.end(); ++it) {
    std::string term_name = it->first.as<std::string>();
    if (is_reserved_config_key(term_name)) {
      continue;
    }

    terms_.push_back(parse_observation_term(
        term_name, YAML::Clone(it->second), should_scale_before_clip_by_default));

    const auto & term = terms_.back();
    std::cout << "[ObservationManager] Registered: " << term.name << " (size="
              << term.single_size() << " x " << term.history_length << " history)" << std::endl;
  }

  // Pre-allocate output buffer
  obs_buffer_.reserve(get_observation_size());

  std::cout << "[ObservationManager] Total observation size: " << get_observation_size()
            << std::endl;
}

ObservationTermCfg ObservationManager::parse_observation_term(
  const std::string & term_name,
  const YAML::Node & term_config,
  bool global_scale_first) const
{
  // Parse static YAML fields before initializing history from the current state.
  ObservationTermCfg term;
  term.name = term_name;
  term.func = observation_function_for(term_name);

  read_term_params(term, term_config);
  read_term_clip(term, term_config);
  read_term_scale(term, term_config);
  read_term_history_length(term, term_config);
  read_term_scale_order(term, term_config, global_scale_first);
  initialize_term_history(term);
  return term;
}

ObservationTermCfg::ObsFunc ObservationManager::observation_function_for(
  const std::string & term_name) const
{
  auto & registry = ObservationRegistry::get_registry();
  const auto func_it = registry.find(term_name);
  if (func_it == registry.end()) {
    throw std::runtime_error("Observation '" + term_name + "' is not registered");
  }
  if (!func_it->second) {
    throw std::runtime_error("Observation function is null for '" + term_name + "'");
  }

  return func_it->second;
}

bool ObservationManager::is_reserved_config_key(const std::string & term_name)
{
  return term_name == "scale_first" || term_name == "use_gym_history";
}

void ObservationManager::read_term_params(
  ObservationTermCfg & term,
  const YAML::Node & term_config)
{
  const auto params = term_config["params"];
  if (!params || params.IsNull()) {
    return;
  }

  // Clone params so term storage does not depend on the original YAML node lifetime.
  term.params = YAML::Clone(params);
}

void ObservationManager::read_term_clip(
  ObservationTermCfg & term,
  const YAML::Node & term_config)
{
  const auto clip = term_config["clip"];
  if (!clip || clip.IsNull()) {
    return;
  }

  if (!clip.IsSequence()) {
    throw std::runtime_error("Observation '" + term.name + "' clip must be [min, max]");
  }

  term.clip = clip.as<std::vector<float>>();
  if (term.clip.size() != 2U) {
    throw std::runtime_error("Observation '" + term.name +
        "' clip must contain exactly [min, max]");
  }
}

void ObservationManager::read_term_scale(
  ObservationTermCfg & term,
  const YAML::Node & term_config)
{
  const auto scale = term_config["scale"];
  if (!scale || scale.IsNull()) {
    return;
  }

  if (scale.IsScalar()) {
    term.scale = {scale.as<float>()};
    return;
  }

  if (!scale.IsSequence()) {
    throw std::runtime_error("Observation '" + term.name + "' scale must be a number or sequence");
  }

  term.scale = scale.as<std::vector<float>>();
}

void ObservationManager::read_term_history_length(
  ObservationTermCfg & term,
  const YAML::Node & term_config)
{
  const auto history_length = term_config["history_length"];
  if (!history_length || history_length.IsNull()) {
    return;
  }

  term.history_length = history_length.as<int>();
  if (term.history_length < 1) {
    throw std::runtime_error("Observation '" + term.name + "' history_length must be >= 1");
  }
}

void ObservationManager::read_term_scale_order(
  ObservationTermCfg & term,
  const YAML::Node & term_config,
  bool global_scale_first)
{
  term.scale_first = global_scale_first;

  const auto scale_first = term_config["scale_first"];
  if (!scale_first || scale_first.IsNull()) {
    return;
  }

  term.scale_first = scale_first.as<bool>();
}

void ObservationManager::initialize_term_history(ObservationTermCfg & term) const
{
  // Initialize history buffer with the current observation.
  auto initial_obs = term.func(make_observation_context(), term.params);
  if (initial_obs.empty()) {
    throw std::runtime_error("Observation '" + term.name + "' returned an empty vector");
  }
  const size_t observation_size = initial_obs.size();
  if (term.scale.size() == 1 && observation_size > 1) {
    term.scale.assign(observation_size, term.scale[0]);
  } else if (!term.scale.empty() && term.scale.size() != observation_size) {
    throw std::runtime_error(
      "Observation '" + term.name + "' scale size " +
      std::to_string(term.scale.size()) +
      " does not match observation size " + std::to_string(observation_size));
  }

  term.reset(initial_obs);
}

ObservationContext ObservationManager::make_observation_context() const
{
  return ObservationContext{*shared_data_, joint_context_, reference_motion_};
}

}  // namespace ai_sapiens_sim2real
