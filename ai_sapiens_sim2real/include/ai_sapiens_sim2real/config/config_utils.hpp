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

#ifndef AI_SAPIENS_SIM2REAL__CONFIG__CONFIG_UTILS_HPP_
#define AI_SAPIENS_SIM2REAL__CONFIG__CONFIG_UTILS_HPP_

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

namespace ai_sapiens_sim2real
{

// Generic YAML loading, path, and validation helpers shared by the config
// classes (RootConfig, Sim2RealConfig). Nothing here is specific to either
// config schema.

std::string make_path_list_string(const std::vector<std::filesystem::path> & paths);
YAML::Node load_yaml_file(const std::filesystem::path & path, const std::string & context);

// Runs func, re-throwing any exception with the YAML file path prepended so
// errors point at the offending file.
template<typename Func>
decltype(auto) with_yaml_file_context(
  const std::filesystem::path & path,
  const std::string & context,
  Func && func)
{
  try {
    return std::forward<Func>(func)();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(context + " in YAML file '" + path.string() + "': " + e.what());
  } catch (const std::exception & e) {
    throw std::runtime_error(context + " in YAML file '" + path.string() + "': " + e.what());
  }
}

void require_existing_file(const std::filesystem::path & path, const std::string & context);
std::filesystem::path resolve_config_path(
  const std::filesystem::path & config_dir,
  const std::string & path);

bool yaml_node_is_missing(const YAML::Node & node);
void require_yaml_node(const YAML::Node & node, const std::string & path);
void require_yaml_map(const YAML::Node & node, const std::string & path);
void require_yaml_sequence(const YAML::Node & node, const std::string & path);
std::string read_required_yaml_string(const YAML::Node & node, const std::string & path);
std::vector<std::string> read_required_yaml_string_sequence(
  const YAML::Node & node,
  const std::string & path);

// Reads a non-empty sequence of strings under key, validating the entries are
// unique and non-empty (joint-order lists in both config schemas).
std::vector<std::string> read_string_sequence(const YAML::Node & node, const std::string & key);
void validate_unique_names(const std::vector<std::string> & names, const std::string & label);

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__CONFIG__CONFIG_UTILS_HPP_
