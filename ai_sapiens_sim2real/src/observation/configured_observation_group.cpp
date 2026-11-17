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
// Author: Kiwoong Park

#include "ai_sapiens_sim2real/observation/configured_observation_group.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace ai_sapiens_sim2real
{
namespace
{
void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error("Observation configuration: " + message);}
}
}

void ConfiguredObservationTerm::apply(
  std::vector<float> & values, size_t begin, size_t end) const
{
  if (begin > end || end > values.size() || end - begin != width) {
    throw std::invalid_argument(name + " observation term range/width mismatch");
  }

  for (size_t j = 0; j < width; ++j) {
    auto & value = values[begin + j];
    if (!std::isfinite(value)) {throw std::runtime_error(name + " is nonfinite");}
    if (clip) {value = std::clamp(value, clip->first, clip->second);}
    value *= scale[j];
    if (!std::isfinite(value)) {throw std::runtime_error(name + " preprocessing is nonfinite");}
  }
}

ConfiguredObservationGroup::ConfiguredObservationGroup(
  const YAML::Node & node, const std::map<std::string, size_t> & supported_terms)
{
  require(node.IsMap(), "group is missing; regenerate sim2real.yaml");
  const auto order = node["layout"].as<std::string>();
  require(order == "frame_major" || order == "term_major", "unknown layout " + order);
  is_term_major_ = order == "term_major";
  const auto terms = node["terms"];
  require(terms.IsMap() && terms.size() > 0, "terms must be a nonempty ordered map");
  std::set<std::string> seen;
  for (const auto & item : terms) {
    const auto name = item.first.as<std::string>();
    require(supported_terms.count(name) && seen.insert(name).second, "unknown/duplicate term " + name);
    const auto cfg = item.second;
    const int history = cfg["history_length"] ? cfg["history_length"].as<int>() : 1;
    require(history > 0 && history <= 1024, name + " invalid history length");
    if (terms_.empty()) {history_length_ = static_cast<size_t>(history);}
    require(static_cast<size_t>(history) == history_length_, "group terms need equal history lengths");
    ConfiguredObservationTerm term{name, supported_terms.at(name), {}, std::nullopt};
    const auto scale = cfg["scale"];
    term.scale = scale.IsScalar() ? std::vector<float>(term.width, scale.as<float>()) :
      scale.as<std::vector<float>>();
    require(term.scale.size() == term.width && std::all_of(term.scale.begin(), term.scale.end(),
        [](float v) {return std::isfinite(v);}), name + " invalid scale");
    const auto clip = cfg["clip"];
    if (clip && !clip.IsNull()) {
      const auto limits = clip.as<std::vector<float>>();
      require(limits.size() == 2 && std::isfinite(limits[0]) && std::isfinite(limits[1]) &&
        limits[0] <= limits[1], name + " invalid clip");
      term.clip = {limits[0], limits[1]};
    }
    width_ += term.width;
    terms_.push_back(std::move(term));
  }
}

void ConfiguredObservationGroup::append_frames(
  std::vector<float> & output, const std::vector<float> & frames) const
{
  require(width_ > 0 && frames.size() % width_ == 0, "invalid frame buffer");
  if (!is_term_major_) {
    output.insert(output.end(), frames.begin(), frames.end());
    return;
  }
  size_t source = 0;
  for (const auto & term : terms_) {
    for (size_t frame = 0; frame < frames.size() / width_; ++frame) {
      const auto start = frames.begin() + frame * width_ + source;
      output.insert(output.end(), start, start + term.width);
    }
    source += term.width;
  }
}

std::vector<ObservationSlice> ConfiguredObservationGroup::layout(size_t offset, size_t frames) const
{
  std::vector<ObservationSlice> result;
  for (const auto & term : terms_) {
    result.push_back({term.name, offset, term.width, frames, is_term_major_ ? term.width : width_});
    offset += term.width * (is_term_major_ ? frames : 1);
  }
  return result;
}

}  // namespace ai_sapiens_sim2real
