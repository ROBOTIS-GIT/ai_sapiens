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

#include "ai_sapiens_sim2real/sensor_handles/joint_states_diagnostics.hpp"

#include <cinttypes>
#include <utility>

#include "ai_sapiens_sim2real/exception_diagnostics.hpp"

namespace ai_sapiens_sim2real
{

JointStatesDiagnostics::JointStatesDiagnostics(
  const rclcpp::Node::SharedPtr & node, const std::string & topic,
  const std::vector<std::string> & joint_names)
: logger_(node->get_logger()), topic_(topic), joint_names_(joint_names)
{
  timer_ = node->create_wall_timer(
    std::chrono::milliseconds(10),
    CALLBACK_WITH_EXCEPTION_LOG(this, JointStatesDiagnostics::flush));
}

void JointStatesDiagnostics::record_ready()
{
  is_ready_.store(true);
}

void JointStatesDiagnostics::record_incomplete(
  const std::vector<JointSampleStatus> & joint_status,
  size_t complete_joints)
{
  const auto now = Clock::now();

  if (now < next_warning_at_) {
    return;
  }

  const auto sample = capture_incomplete_sample(joint_status, complete_joints, now);

  if (store_pending_warning(sample)) {
    next_warning_at_ = now + std::chrono::seconds(1);
  }
}

JointStatesDiagnostics::IncompleteSample JointStatesDiagnostics::capture_incomplete_sample(
  const std::vector<JointSampleStatus> & joint_status,
  size_t complete_joints, Clock::time_point now) const
{
  IncompleteSample sample;
  sample.complete_joints = complete_joints;
  sample.detected_at = now;
  sample.unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();

  for (size_t i = 0; i < joint_status.size(); ++i) {
    if (joint_status[i] == JointSampleStatus::Valid) {
      continue;
    }

    if (sample.missing_count == kMaxMissingJointsToLog) {
      sample.has_more_missing_joints = true;
      break;
    }

    sample.missing_joints[sample.missing_count++] = {i, joint_status[i]};
  }

  return sample;
}

bool JointStatesDiagnostics::store_pending_warning(const IncompleteSample & sample)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);

  if (!lock.owns_lock()) {
    return false;
  }

  // Coalesce persistent malformed samples if the node executor cannot keep up.
  pending_warning_ = sample;
  return true;
}

std::optional<JointStatesDiagnostics::IncompleteSample>
JointStatesDiagnostics::take_pending_warning()
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);

  if (!lock.owns_lock()) {
    return std::nullopt;
  }

  return std::exchange(pending_warning_, std::nullopt);
}

std::string JointStatesDiagnostics::build_missing_joint_summary(
  const IncompleteSample & sample) const
{
  std::string summary;

  for (size_t i = 0; i < sample.missing_count; ++i) {
    if (!summary.empty()) {
      summary += ", ";
    }

    const auto & missing = sample.missing_joints[i];
    summary += joint_names_[missing.joint_index];

    if (missing.status == JointSampleStatus::MissingValues) {
      summary += "(missing position/velocity)";
    } else if (missing.status == JointSampleStatus::InvalidValues) {
      summary += "(invalid position/velocity)";
    }
  }

  if (sample.has_more_missing_joints) {
    summary += ", ...";
  }

  return summary;
}

void JointStatesDiagnostics::report_incomplete(const IncompleteSample & sample)
{
  const auto summary = build_missing_joint_summary(sample);
  const double delay_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - sample.detected_at).count();
  RCLCPP_WARN(logger_,
    "Ignoring incomplete JointState message on %s: %zu/%zu joints complete. "
    "Missing/incomplete: %s detected_at=%" PRId64 ".%09" PRId64 " log_delay=%.1fms",
    topic_.c_str(), sample.complete_joints, joint_names_.size(), summary.c_str(),
    sample.unix_ns / 1'000'000'000,
    sample.unix_ns % 1'000'000'000, delay_ms);
}

void JointStatesDiagnostics::flush()
{
  const auto sample = take_pending_warning();

  if (sample) {
    report_incomplete(*sample);
  }

  if (!has_logged_ready_ && is_ready_.load()) {
    RCLCPP_INFO(logger_, "[JointStatesSensorHandle] all %zu joints received", joint_names_.size());
    has_logged_ready_ = true;
  }
}

}  // namespace ai_sapiens_sim2real
