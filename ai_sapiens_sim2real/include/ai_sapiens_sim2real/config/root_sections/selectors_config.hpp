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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__SELECTORS_CONFIG_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__SELECTORS_CONFIG_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

using Selector = std::unordered_map<uint16_t, std::string>;

class SelectorsConfig
{
public:
  SelectorsConfig() = default;
  explicit SelectorsConfig(std::unordered_map<std::string, Selector> selectors);

  static SelectorsConfig from_yaml(const YAML::Node & config);

  std::optional<std::string> find_target_state_for_selector_code(
    const std::string & selector_name,
    uint16_t selector_code) const;
  bool has_selector(const std::string & selector_name) const;
  void for_each_selector(
    const std::function<void(const std::string &, const Selector &)> & visitor) const;

private:
  static std::unordered_map<std::string, Selector> read_selectors(
    const YAML::Node & selectors_node);
  static Selector read_selector(
    const YAML::Node & selector_node,
    const std::string & selector_name);
  static Selector read_selector_table(
    const YAML::Node & table_node,
    const std::string & table_path);

  const Selector & require_selector(const std::string & selector_name) const;

  std::unordered_map<std::string, Selector> selectors_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__ROOT_SECTIONS__SELECTORS_CONFIG_HPP_
