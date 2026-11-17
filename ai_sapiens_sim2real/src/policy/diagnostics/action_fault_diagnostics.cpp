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

#include "ai_sapiens_sim2real/policy/diagnostics/action_fault_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace ai_sapiens_sim2real
{
namespace
{
constexpr double kDegreesPerRadian = 180.0 / 3.141592653589793;
// Heuristics for highlighting only: equivalent to 20 deg and 500 deg/s
// changes over 20 ms. Scale by the actual interval between observations.
constexpr double kAngleChangeRate = 1000.0;  // deg/s
constexpr double kVelocityChangeRate = 25000.0;  // deg/s^2

Eigen::Vector3d roll_pitch_yaw(const Eigen::Quaternionf & orientation)
{
  if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-6f) {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }

  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix().cast<double>();

  return Eigen::Vector3d(
    std::atan2(rotation(2, 1), rotation(2, 2)),
    std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0))),
    std::atan2(rotation(1, 0), rotation(0, 0))) * kDegreesPerRadian;
}

enum class SensorFlag {None, Missing, Nonfinite, PreviousNonfinite, Jump};

struct SensorRow
{
  std::string name;
  std::optional<double> current, previous, delta;
  SensorFlag flag{SensorFlag::None};
};

struct SensorTable
{
  std::string title, name_label;
  std::vector<SensorRow> rows;
};

struct TableWidths
{
  size_t name{5};
  int current{10}, previous{10}, delta{10};
};

SensorRow make_sensor_row(
  std::string name, std::optional<double> current,
  std::optional<double> previous, double jump_limit, bool wrap_angle = false)
{
  SensorRow row{std::move(name), current, previous, std::nullopt};

  if (!current) {
    row.flag = SensorFlag::Missing;
    return row;
  }

  if (!std::isfinite(*current)) {row.flag = SensorFlag::Nonfinite;}
  if (previous) {
    row.delta = wrap_angle ? std::remainder(*current - *previous, 360.0) : *current - *previous;

    if (row.flag == SensorFlag::None && !std::isfinite(*previous)) {
      row.flag = SensorFlag::PreviousNonfinite;
    }

    if (std::isfinite(*row.delta) && std::isfinite(jump_limit) &&
      std::abs(*row.delta) > jump_limit)
    {
      row.flag = SensorFlag::Jump;
    }
  }

  return row;
}

std::string format_sensor_number(std::optional<double> value, bool show_sign = false)
{
  if (!value) {return "n/a";}

  std::ostringstream text;
  text << std::fixed << std::setprecision(3);

  if (show_sign) {text << std::showpos;}
  text << *value;

  return text.str();
}

TableWidths measure_table(const SensorTable & table)
{
  TableWidths widths;

  for (const auto & row : table.rows) {
    widths.name = std::max(widths.name, row.name.size());
    widths.current = std::max(
      widths.current, static_cast<int>(format_sensor_number(row.current).size()));
    widths.previous = std::max(
      widths.previous, static_cast<int>(format_sensor_number(row.previous).size()));
    widths.delta = std::max(
      widths.delta, static_cast<int>(format_sensor_number(row.delta, true).size()));
  }

  return widths;
}

const char * sensor_flag_label(SensorFlag flag)
{
  switch (flag) {
    case SensorFlag::Missing: return "[MISSING]";
    case SensorFlag::Nonfinite: return "[NONFINITE]";
    case SensorFlag::PreviousNonfinite: return "[PREV_NONFINITE]";
    case SensorFlag::Jump: return "[JUMP!]";
    case SensorFlag::None: return "";
  }

  return "";
}

void append_sensor_table(std::ostream & out, const SensorTable & table, bool jumps_only)
{
  const auto widths = measure_table(table);
  bool has_header = false;

  for (const auto & row : table.rows) {
    if (jumps_only && row.flag != SensorFlag::Jump) {continue;}

    if (!has_header) {
      out << "\n" << table.title << "\n  " << std::left << std::setw(widths.name) <<
        table.name_label
          << " " << std::right << std::setw(widths.current) << "current"
          << " " << std::setw(widths.previous) << "prev" << " " << std::setw(widths.delta) <<
        "delta" << " flag\n";
      has_header = true;
    }

    out << "  " << std::left << std::setw(widths.name) << row.name;

    if (row.flag != SensorFlag::Missing) {
      out << " " << std::right << std::setw(widths.current) << format_sensor_number(row.current)
          << " " << std::setw(widths.previous) << format_sensor_number(row.previous)
          << " " << std::setw(widths.delta) << format_sensor_number(row.delta, true);
    }

    if (row.flag != SensorFlag::None) {out << " " << sensor_flag_label(row.flag);}
    out << "\n";
  }
}

SensorTable make_orientation_table(
  const SensorData & current,
  const std::optional<SensorData> & previous, double interval)
{
  SensorTable table{"IMU RPY [deg, ZYX; wrapped delta]", "axis", {}};
  const auto angles = roll_pitch_yaw(current.orientation);
  const auto old =
    previous ? std::optional<Eigen::Vector3d>(roll_pitch_yaw(previous->orientation)) :
    std::nullopt;
  const char * names[] = {"roll", "pitch", "yaw"};

  for (size_t i = 0; i < 3; ++i) {
    table.rows.push_back(make_sensor_row(names[i], angles[i],
      old ? std::optional<double>((*old)[i]) : std::nullopt, kAngleChangeRate * interval, true));
  }

  return table;
}

SensorTable make_gyro_table(
  const SensorData & current,
  const std::optional<SensorData> & previous, double interval)
{
  SensorTable table{"IMU gyro [deg/s, body axes]", "axis", {}};
  const char * names[] = {"wx", "wy", "wz"};

  for (size_t i = 0; i < 3; ++i) {
    table.rows.push_back(make_sensor_row(names[i], current.angular_velocity[i] * kDegreesPerRadian,
      previous ? std::optional<double>(previous->angular_velocity[i] *
          kDegreesPerRadian) : std::nullopt,
      kVelocityChangeRate * interval));
  }

  return table;
}

enum class JointQuantity {Position, Velocity};

SensorTable make_joint_table(
  const SensorData & current,
  const std::optional<SensorData> & previous, const std::vector<std::string> & names,
  double interval, JointQuantity quantity)
{
  const bool is_velocity = quantity == JointQuantity::Velocity;
  SensorTable table{is_velocity ? "Joint velocity [deg/s]" : "Joint position [deg]", "joint", {}};
  const auto & values = is_velocity ? current.joint_vel : current.joint_pos;
  const auto * old =
    previous ? &(is_velocity ? previous->joint_vel : previous->joint_pos) : nullptr;
  const double limit = (is_velocity ? kVelocityChangeRate : kAngleChangeRate) * interval;

  for (size_t i = 0; i < names.size(); ++i) {
    const auto value = i < static_cast<size_t>(values.size()) ?
      std::optional<double>(values[i] * kDegreesPerRadian) : std::nullopt;
    const auto old_value = old && i < static_cast<size_t>(old->size()) ?
      std::optional<double>((*old)[i] * kDegreesPerRadian) : std::nullopt;
    table.rows.push_back(make_sensor_row(compact_joint_name(names[i]), value, old_value, limit));
  }

  return table;
}
}  // namespace

std::string compact_joint_name(const std::string & name)
{
  std::string result = name;

  if (result.compare(0, 5, "left_") == 0) {
    result.replace(0, 5, "L.");
  } else if (result.compare(0, 6, "right_") == 0) {result.replace(0, 6, "R.");}
  if (result.size() >= 6 && result.compare(result.size() - 6, 6, "_joint") == 0) {
    result.resize(result.size() - 6);
  }

  return result;
}

std::string format_action_fault_sensors(
  const SensorData & current, const std::optional<SensorData> & previous,
  const std::vector<std::string> & joint_names, double elapsed_seconds, bool jumps_only)
{
  const bool has_interval = previous && std::isfinite(elapsed_seconds) && elapsed_seconds > 0.0;
  const double interval = has_interval ? elapsed_seconds : std::numeric_limits<double>::quiet_NaN();
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);

  if (!jumps_only) {
    out << "Sensors: dt=" << interval
        << " s; IMU usable=" << (current.is_imu_usable ? "true" : "false")
        << "\nJump thresholds (diagnostic):\n  angle=" << kAngleChangeRate * interval
        << " deg, velocity=" << kVelocityChangeRate * interval << " deg/s\n";
  }

  append_sensor_table(out, make_orientation_table(current, previous, interval), jumps_only);
  append_sensor_table(out, make_gyro_table(current, previous, interval), jumps_only);
  append_sensor_table(out, make_joint_table(current, previous, joint_names, interval,
    JointQuantity::Position), jumps_only);
  append_sensor_table(out, make_joint_table(current, previous, joint_names, interval,
    JointQuantity::Velocity), jumps_only);

  return out.str();
}

const char * action_outcome_label(PolicyActionOutcome outcome)
{
  switch (outcome) {
    case PolicyActionOutcome::Accepted: return "ACCEPTED";
    case PolicyActionOutcome::Retry: return "HOLD; retry next tick";
    case PolicyActionOutcome::Damping: return "DAMPING";
  }

  return "unknown";
}

const char * action_rejection_label(ActionRejection reason)
{
  switch (reason) {
    case ActionRejection::None: return "";
    case ActionRejection::Nonfinite: return "[NONFINITE]";
    case ActionRejection::AbsoluteLimit: return "[LIMIT]";
    case ActionRejection::InvalidReference: return "[REFERENCE_NONFINITE]";
    case ActionRejection::BoundaryAndDelta: return "[CLIP+DELTA]";
  }

  return "";
}

const char * boundary_check_label(BoundaryCheckState state)
{
  switch (state) {
    case BoundaryCheckState::Disabled: return "disabled";
    case BoundaryCheckState::NotNeeded: return "not_needed";
    case BoundaryCheckState::Completed: return "completed";
    case BoundaryCheckState::SkippedInvalidAction: return "skipped_invalid_action";
  }

  return "unknown";
}

std::string format_action_fault_actions(
  const std::vector<float> & processed, const std::vector<std::string> & joints,
  const ActionValidation & validation, bool invalid_only)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);

  for (size_t i = 0; i < processed.size(); ++i) {
    const auto * check = i < validation.joints.size() ? &validation.joints[i] : nullptr;

    if (invalid_only && (!check || check->rejection == ActionRejection::None)) {continue;}

    out << "  " << std::left << std::setw(16) << compact_joint_name(joints.at(i))
        << " " << std::right << std::setw(10) << processed[i];

    if (check && check->reference) {
      out << " " << std::setw(10) << *check->reference
          << " " << std::showpos << std::setw(10) << check->delta << std::noshowpos;
    }

    if (!check) {
      out << " [NOT CHECKED]";
    } else if (check->rejection != ActionRejection::None) {
      out << " " << action_rejection_label(check->rejection);
    }

    out << "\n";
  }

  return out.str();
}

}  // namespace ai_sapiens_sim2real
