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

#include "ai_sapiens_sim2real/config/config_utils.hpp"

#include <sstream>
#include <unordered_set>

namespace ai_sapiens_sim2real
{

std::string make_path_list_string(const std::vector<std::filesystem::path> & paths)
{
  std::ostringstream stream;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }

    stream << paths[i].string();
  }

  return stream.str();
}

YAML::Node load_yaml_file(const std::filesystem::path & path, const std::string & context)
{
  try {
    return YAML::LoadFile(path.string());
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
      context + " failed to load YAML file '" + path.string() + "': " + e.what());
  }
}

void require_existing_file(const std::filesystem::path & path, const std::string & context)
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error(context + " missing file: " + path.string());
  }
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error(context + " is not a regular file: " + path.string());
  }
}

std::filesystem::path resolve_config_path(
  const std::filesystem::path & config_dir,
  const std::string & path)
{
  std::filesystem::path p(path);
  if (p.is_absolute()) {
    return p;
  }

  return config_dir / p;
}

bool yaml_node_is_missing(const YAML::Node & node)
{
  return !node || node.IsNull();
}

void require_yaml_node(const YAML::Node & node, const std::string & path)
{
  if (yaml_node_is_missing(node)) {
    throw std::runtime_error(path + " is required");
  }
}

void require_yaml_map(const YAML::Node & node, const std::string & path)
{
  require_yaml_node(node, path);
  if (!node.IsMap()) {
    throw std::runtime_error(path + " must be a map");
  }
}

void require_yaml_sequence(const YAML::Node & node, const std::string & path)
{
  require_yaml_node(node, path);
  if (!node.IsSequence()) {
    throw std::runtime_error(path + " must be a sequence");
  }
}

std::string read_required_yaml_string(const YAML::Node & node, const std::string & path)
{
  require_yaml_node(node, path);
  return node.as<std::string>();
}

std::vector<std::string> read_required_yaml_string_sequence(
  const YAML::Node & node,
  const std::string & path)
{
  require_yaml_sequence(node, path);

  auto values = node.as<std::vector<std::string>>();
  if (values.empty()) {
    throw std::runtime_error(path + " must not be empty");
  }

  return values;
}

std::vector<std::string> read_string_sequence(const YAML::Node & node, const std::string & key)
{
  return read_required_yaml_string_sequence(node[key], key);
}

void validate_unique_names(const std::vector<std::string> & names, const std::string & label)
{
  std::unordered_set<std::string> seen;
  for (const auto & name : names) {
    if (name.empty()) {
      throw std::runtime_error(label + " contains an empty joint name");
    }
    if (!seen.insert(name).second) {
      throw std::runtime_error(label + " contains duplicate joint name '" + name + "'");
    }
  }
}

}  // namespace ai_sapiens_sim2real
