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

#include "ai_sapiens_sim2real/config/root_sections/selectors_config.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "ai_sapiens_sim2real/config/config_utils.hpp"

namespace ai_sapiens_sim2real
{

SelectorsConfig::SelectorsConfig(std::unordered_map<std::string, Selector> selectors)
: selectors_(std::move(selectors))
{
}

SelectorsConfig SelectorsConfig::from_yaml(const YAML::Node & config)
{
  return SelectorsConfig(read_selectors(config["selectors"]));
}

const Selector & SelectorsConfig::require_selector(const std::string & selector_name) const
{
  const auto selector = selectors_.find(selector_name);
  if (selector == selectors_.end()) {
    throw std::runtime_error("Unknown selector: " + selector_name);
  }

  return selector->second;
}

std::optional<std::string> SelectorsConfig::find_target_state_for_selector_code(
  const std::string & selector_name,
  uint16_t selector_code) const
{
  const auto & selector = require_selector(selector_name);

  const auto target_state = selector.find(selector_code);
  if (target_state == selector.end()) {
    return std::nullopt;
  }

  return target_state->second;
}

bool SelectorsConfig::has_selector(const std::string & selector_name) const
{
  return selectors_.find(selector_name) != selectors_.end();
}

void SelectorsConfig::for_each_selector(
  const std::function<void(const std::string &, const Selector &)> & visitor) const
{
  for (const auto & [name, selector] : selectors_) {
    visitor(name, selector);
  }
}

std::unordered_map<std::string, Selector> SelectorsConfig::read_selectors(
  const YAML::Node & selectors_node)
{
  if (yaml_node_is_missing(selectors_node)) {
    return {};
  }

  require_yaml_map(selectors_node, "selectors");

  std::unordered_map<std::string, Selector> selectors;
  selectors.reserve(selectors_node.size());

  for (const auto & selector_entry : selectors_node) {
    const auto selector_name = selector_entry.first.as<std::string>();
    selectors[selector_name] = read_selector(selector_entry.second, selector_name);
  }

  return selectors;
}

Selector SelectorsConfig::read_selector(
  const YAML::Node & selector_node,
  const std::string & selector_name)
{
  const std::string table_path = "selectors." + selector_name + ".table";
  return read_selector_table(selector_node["table"], table_path);
}

Selector SelectorsConfig::read_selector_table(
  const YAML::Node & table_node,
  const std::string & table_path)
{
  require_yaml_map(table_node, table_path);

  Selector selector;

  for (const auto & yaml_entry : table_node) {
    const auto selector_code = yaml_entry.first.as<uint16_t>();
    const auto target_state = yaml_entry.second.as<std::string>();

    const bool added = selector.emplace(selector_code, target_state).second;
    if (!added) {
      throw std::runtime_error(
        table_path + " has a duplicate key '" + std::to_string(selector_code) + "'");
    }
  }

  return selector;
}

}  // namespace ai_sapiens_sim2real
