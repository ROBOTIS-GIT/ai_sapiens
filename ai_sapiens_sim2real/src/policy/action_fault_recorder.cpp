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

#include "ai_sapiens_sim2real/policy/action_fault_recorder.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <sys/stat.h>
#include <utility>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>
#include "ai_sapiens_sim2real/policy/action_fault_diagnostics.hpp"

namespace ai_sapiens_sim2real
{
namespace
{
size_t snapshot_bytes(const FaultSnapshot & snapshot)
{
  size_t bytes = sizeof(snapshot) +
    snapshot.processed_action.size() * sizeof(float) +
    snapshot.validation.joints.size() * sizeof(JointActionCheck) +
    snapshot.error.size();
  for (const auto & call : snapshot.calls) {
    bytes += sizeof(call) + call.output.size() * sizeof(float);
    for (const auto & [name, values] : call.inputs) {
      bytes += name.size() + sizeof(values) + values.size() * sizeof(float);
    }
  }
  const auto sensor_bytes = [](const SensorData & s) {
      return static_cast<size_t>(s.joint_pos.size() + s.joint_vel.size()) * sizeof(float);
    };
  bytes += sensor_bytes(snapshot.sensors);
  if (snapshot.previous_sensors) {bytes += sensor_bytes(*snapshot.previous_sensors);}
  return bytes;
}

std::string channel_label(
  const ObservationSlice & slice, size_t index,
  const std::vector<std::string> & joints)
{
  const bool is_joint = slice.name.find("joint_pos") != std::string::npos ||
    slice.name.find("joint_vel") != std::string::npos || slice.name == "last_action" ||
    slice.name == "actions" || slice.name == "last_target";
  if (is_joint && slice.width == joints.size()) {return joints.at(index);}
  if (slice.name == "motion_command" && slice.width == 2 * joints.size()) {
    return std::string(index < joints.size() ? "position/" : "velocity/") +
           joints.at(index % joints.size());
  }
  if ((slice.name == "base_ang_vel" || slice.name == "projected_gravity") && slice.width == 3) {
    return std::vector<std::string>{"x", "y", "z"}.at(index);
  }
  if (slice.name == "velocity_commands" && slice.width == 3) {
    return std::vector<std::string>{"linear_x", "linear_y", "angular_z"}.at(index);
  }
  return std::to_string(index);
}

void emit_action_validation(YAML::Emitter & out, const ActionValidation & validation)
{
  out << YAML::Key << "action_boundary_check" << YAML::Value
      << boundary_check_label(validation.boundary_check)
      << YAML::Key << "action_reference" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const auto & joint : validation.joints) {
    if (joint.reference) {out << *joint.reference;}
  }
  out << YAML::EndSeq << YAML::Key << "action_raw_boundary_reached" << YAML::Value
      << YAML::Flow << YAML::BeginSeq;
  for (const auto & joint : validation.joints) {
    out << joint.is_at_raw_action_boundary;
  }
  out << YAML::EndSeq << YAML::Key << "action_rejection_reasons" << YAML::Value
      << YAML::Flow << YAML::BeginSeq;
  for (const auto & joint : validation.joints) {
    out << action_rejection_label(joint.rejection);
  }
  out << YAML::EndSeq;
}

void emit_observation_term(
  YAML::Emitter & out, const ObservationSlice & slice, const std::vector<float> & values,
  const std::vector<std::string> & policy_joints)
{
  out << YAML::Key << slice.name << YAML::Value << YAML::BeginMap;

  for (size_t channel = 0; channel < slice.width; ++channel) {
    std::vector<float> history;

    for (size_t frame = 0; frame < slice.frames; ++frame) {
      history.push_back(values.at(slice.offset + frame * slice.frame_stride +
        channel * slice.element_stride));
    }

    out << YAML::Key << channel_label(slice, channel, policy_joints)
        << YAML::Value << YAML::Flow << history;
  }

  out << YAML::EndMap;
}

void emit_inference_input(
  YAML::Emitter & out, const std::string & name, const std::vector<int64_t> & shape,
  const std::vector<float> & values, const FaultPolicyMetadata & metadata)
{
  out << YAML::BeginMap << YAML::Key << "name" << YAML::Value << name
      << YAML::Key << "shape" << YAML::Value << YAML::Flow << shape
      << YAML::Key << "terms" << YAML::Value << YAML::BeginMap;
  const auto layout = metadata.layouts.find(name);

  if (layout != metadata.layouts.end()) {
    for (const auto & slice : layout->second) {
      emit_observation_term(out, slice, values, metadata.policy_joints);
    }
  }

  out << YAML::EndMap
      << YAML::Key << "raw_tensor" << YAML::Value << YAML::Flow << values << YAML::EndMap;
}

void emit_inference_call(
  YAML::Emitter & out, const FaultInferenceCall & call, const FaultPolicyMetadata & metadata)
{
  out << YAML::BeginMap << YAML::Key << "inputs" << YAML::Value << YAML::BeginSeq;

  // Preserve ONNX input order, not unordered_map iteration order.
  for (size_t i = 0; i < metadata.input_names.size(); ++i) {
    const auto & name = metadata.input_names[i];
    emit_inference_input(out, name, metadata.input_shapes.at(i), call.inputs.at(name), metadata);
  }

  out << YAML::EndSeq << YAML::Key << "raw_output" << YAML::Value << YAML::Flow
      << call.output << YAML::EndMap;
}

void emit_snapshot(
  YAML::Emitter & out, const FaultSnapshot & snapshot,
  const FaultPolicyMetadata & metadata)
{
  out << YAML::BeginMap
      << YAML::Key << "unix_time_ns" << YAML::Value << snapshot.unix_time_ns
      << YAML::Key << "episode_time" << YAML::Value << snapshot.episode_time
      << YAML::Key << "outcome" << YAML::Value << action_outcome_label(snapshot.outcome)
      << YAML::Key << "consecutive_failures" << YAML::Value << snapshot.consecutive_failures
      << YAML::Key << "inference_error" << YAML::Value << snapshot.error
      << YAML::Key << "processed_action" << YAML::Value << YAML::Flow << snapshot.processed_action;
  emit_action_validation(out, snapshot.validation);
  out << YAML::Key << "onnx_calls" << YAML::Value << YAML::BeginSeq;
  for (const auto & call : snapshot.calls) {
    emit_inference_call(out, call, metadata);
  }
  out << YAML::EndSeq << YAML::EndMap;
}

std::string format_unix_timestamp(int64_t unix_time_ns)
{
  // Format integers directly so epoch-sized doubles cannot lose nanoseconds.
  const bool is_negative = unix_time_ns < 0;
  const uint64_t magnitude = is_negative ?
    static_cast<uint64_t>(-(unix_time_ns + 1)) + 1 : static_cast<uint64_t>(unix_time_ns);
  std::ostringstream out;
  out << (is_negative ? "-" : "") << magnitude / 1000000000
      << '.' << std::setfill('0') << std::setw(9) << magnitude % 1000000000;
  return out.str();
}

std::string format_capture_timestamp(int64_t unix_time_ns)
{
  // Explicit Korea time: container TZ is often UTC. Use the captured inference
  // time, not the later background write time, and retain unix_time_ns below.
  constexpr int64_t nanoseconds_per_second = 1000000000;
  auto seconds = unix_time_ns / nanoseconds_per_second;
  auto remainder = unix_time_ns % nanoseconds_per_second;
  if (remainder < 0) {--seconds; remainder += nanoseconds_per_second;}
  const std::time_t korea_time = static_cast<std::time_t>(seconds + 9 * 60 * 60);
  std::tm calendar{};
  if (!gmtime_r(&korea_time, &calendar)) {
    throw std::runtime_error("cannot format inference timestamp");
  }
  std::ostringstream out;
  out << std::put_time(&calendar, "%Y-%m-%dT%H:%M:%S")
      << '.' << std::setfill('0') << std::setw(3) << remainder / 1000000 << "+09:00";
  return out.str();
}

std::string format_record_summary(const ActionFaultRecord & record)
{
  const auto & m = *record.metadata;
  const auto & s = record.current;
  std::ostringstream diagnostics;
  diagnostics << "ACTION DIAGNOSTICS | " << action_outcome_label(s.outcome) << " | incident=" <<
    record.incident
              << " attempt=" << record.attempt << "\nPolicy: " << m.state
              << "\nlimit=" << m.action_limit << " rad, change_limit=" << m.action_change_limit
              << " rad (" << boundary_check_label(s.validation.boundary_check)
              << "), AND |raw action| >= " << m.raw_action_boundary << ", dt=" << m.step_dt
              << " s"
              << "\n\nActions [rad, policy coordinates]:\n"
              << "  joint             processed   previous      delta flag\n"
              << format_action_fault_actions(s.processed_action, m.policy_joints, s.validation)
              << "\n" << format_action_fault_sensors(s.sensors, s.previous_sensors,
                m.controller_joints, s.sensor_interval);
  return diagnostics.str();
}

std::string serialize(const ActionFaultRecord & record)
{
  const auto & m = *record.metadata;
  const auto & s = record.current;
  YAML::Emitter out;
  out.SetFloatPrecision(std::numeric_limits<float>::max_digits10);
  out.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  out << YAML::BeginMap
      << YAML::Key << "timestamp" << YAML::Value << YAML::DoubleQuoted
      << format_capture_timestamp(s.unix_time_ns)
      << YAML::Key << "timestamp_unix" << YAML::Value << YAML::DoubleQuoted
      << format_unix_timestamp(s.unix_time_ns)
      << YAML::Key << "diagnostics" << YAML::Value << YAML::Literal << format_record_summary(record)
      << YAML::Key << "observation_notes" << YAML::Value
      << "Actual ONNX inputs after scaling/clipping; term rows are oldest-to-newest history. "
    "Policy coordinates, not physical sensor units. "
    "raw_tensor preserves flat order and float32 precision; .nan/.inf are YAML floats."
      << YAML::Key << "current_inference" << YAML::Value;
  emit_snapshot(out, s, m);
  if (record.previous_accepted) {
    out << YAML::Key << "previous_accepted_inference" << YAML::Value;
    emit_snapshot(out, *record.previous_accepted, m);
  }
  out << YAML::Key << "policy" << YAML::Value << YAML::BeginMap
      << YAML::Key << "state" << YAML::Value << m.state
      << YAML::Key << "model_path" << YAML::Value << m.model
      << YAML::Key << "config_path" << YAML::Value << m.config_path
      << YAML::Key << "config_at_startup" << YAML::Value << YAML::Literal << m.config_yaml
      << YAML::Key << "joint_order" << YAML::Value << m.policy_joints << YAML::EndMap
      << YAML::EndMap;
  if (!out.good()) {throw std::runtime_error(out.GetLastError());}
  return std::string(out.c_str()) + "\n";
}

size_t existing_storage_bytes(const std::filesystem::path & directory)
{
  size_t bytes = 0;

  if (!std::filesystem::exists(directory)) {
    return bytes;
  }

  // Count previous runs without deleting old evidence to make room.
  for (const auto & entry : std::filesystem::recursive_directory_iterator(directory)) {
    if (entry.is_regular_file()) {bytes += entry.file_size();}
  }

  return bytes;
}

std::filesystem::path choose_session_directory(const std::filesystem::path & directory)
{
  const auto now = std::time(nullptr);
  std::tm utc{};
  if (!gmtime_r(&now, &utc)) {
    throw std::runtime_error("cannot format action fault session time");
  }

  constexpr char suffix_characters[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  std::random_device random;
  std::uniform_int_distribution<size_t> suffix_index(0, sizeof(suffix_characters) - 2);
  std::filesystem::path session_directory;

  do {
    std::ostringstream name;
    name << "run-" << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << '-';
    for (int i = 0; i < 6; ++i) {
      name << suffix_characters[suffix_index(random)];
    }
    session_directory = directory / name.str();
  } while (std::filesystem::exists(session_directory));

  return session_directory;
}

}  // namespace

std::filesystem::path ActionFaultRecorder::default_directory()
{
  return std::filesystem::path(AI_SAPIENS_SIM2REAL_SOURCE_DIR) / "logs/action_faults";
}

ActionFaultRecorder::ActionFaultRecorder(Options options, rclcpp::Logger logger)
: options_(std::move(options)), logger_(std::move(logger))
{
  if (options_.directory.empty()) {options_.directory = default_directory();}
  options_.directory = std::filesystem::absolute(options_.directory);
  stored_bytes_ = existing_storage_bytes(options_.directory);
  // Choose the path now; only the first record creates a directory.
  session_directory_ = choose_session_directory(options_.directory);
  worker_ = std::thread(&ActionFaultRecorder::write_loop, this);
}

void ActionFaultRecorder::create_session_directory()
{
  if (is_session_created_) {return;}
  std::filesystem::create_directories(options_.directory);
  // Never reuse an existing directory, even in the unlikely event of a name collision.
  if (!std::filesystem::create_directory(session_directory_)) {
    throw std::runtime_error("action fault session directory already exists");
  }
  // Containers commonly run as root; allow the bind-mount's host user to read
  // diagnostics while keeping write access restricted to the creating user.
  std::filesystem::permissions(session_directory_, std::filesystem::perms::owner_all |
    std::filesystem::perms::group_read | std::filesystem::perms::group_exec |
    std::filesystem::perms::others_read | std::filesystem::perms::others_exec);
  is_session_created_ = true;
  RCLCPP_INFO(logger_, "Action fault files: %s (exclusive creation, no automatic deletion)",
    session_directory_.c_str());
}

ActionFaultRecorder::~ActionFaultRecorder()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_stopping_ = true;
  }
  ready_.notify_one();
  if (worker_.joinable()) {worker_.join();}
}

std::string ActionFaultRecorder::enqueue(ActionFaultRecord record) noexcept
{
  try {
    if (!is_enabled_) {return {};}
    record.queued_bytes = snapshot_bytes(record.current) + sizeof(record);
    if (record.previous_accepted) {
      record.queued_bytes += snapshot_bytes(*record.previous_accepted);
    }
    const auto suffix = record.current.outcome ==
      PolicyActionOutcome::Accepted ? "-recovered.yaml" : ".yaml";
    record.path = session_directory_ / ("incident-" + std::to_string(record.incident) +
      "-attempt-" + std::to_string(record.attempt) + suffix);
    const auto path = record.path.string();
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || is_stopping_ || queue_.size() >= options_.max_queue_records ||
      record.queued_bytes > options_.max_queue_bytes - queued_bytes_)
    {
      ++dropped_records_;
      return {};
    }
    queue_.push_back(std::move(record));
    queued_bytes_ += queue_.back().queued_bytes;
    ready_.notify_one();
    return path;
  } catch (...) {
    ++dropped_records_;
    return {};
  }
}

void ActionFaultRecorder::write_loop() noexcept
{
  for (;; ) {
    ActionFaultRecord record;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      const auto has_pending_work = [this] {return is_stopping_ || !queue_.empty();};
      ready_.wait_for(lock, std::chrono::seconds(1), has_pending_work);
      const auto dropped = dropped_records_.exchange(0);
      if (dropped) {
        RCLCPP_WARN(logger_, "Action fault recorder dropped %zu record(s): queue unavailable",
            dropped);
      }
      if (queue_.empty()) {
        if (is_stopping_) {return;}
        continue;
      }
      record = std::move(queue_.front());
      queue_.pop_front();
      queued_bytes_ -= record.queued_bytes;
    }
    if (!is_enabled_) {continue;}
    try {
      write_record(record);
    } catch (const std::exception & e) {
      is_enabled_ = false;
      RCLCPP_ERROR(logger_, "Action fault file recording stopped: %s; existing files preserved. "
        "Failed path: %s", e.what(), record.path.c_str());
    } catch (...) {
      is_enabled_ = false;
      RCLCPP_ERROR(logger_, "Action fault file recording stopped: unknown writer error");
    }
  }
}

void ActionFaultRecorder::write_record(const ActionFaultRecord & record)
{
  const auto text = serialize(record);
  if (stored_bytes_ > options_.max_storage_bytes ||
    text.size() > options_.max_storage_bytes - stored_bytes_)
  {
    throw std::runtime_error("storage quota reached (no files removed)");
  }
  create_session_directory();
  const int fd = ::open(record.path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (fd < 0) {throw std::runtime_error(std::strerror(errno));}
  if (::fchmod(fd, 0644) != 0) {
    const auto error = std::string(std::strerror(errno));
    ::close(fd);
    throw std::runtime_error("cannot make diagnostic file readable: " + error);
  }
  size_t offset = 0;
  while (offset < text.size()) {
    const auto count = ::write(fd, text.data() + offset, text.size() - offset);
    if (count < 0 && errno == EINTR) {continue;}
    if (count <= 0) {
      const auto error = std::string(std::strerror(errno));
      ::close(fd);
      throw std::runtime_error("partial file retained: " + error);
    }
    offset += static_cast<size_t>(count);
  }
  const int sync_result = ::fsync(fd);
  const int close_result = ::close(fd);
  if (sync_result != 0 || close_result != 0) {throw std::runtime_error("file flush failed");}
  stored_bytes_ += text.size();
}

}  // namespace ai_sapiens_sim2real
