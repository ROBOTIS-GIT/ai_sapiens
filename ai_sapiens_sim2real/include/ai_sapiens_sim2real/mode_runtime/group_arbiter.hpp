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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__GROUP_ARBITER_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__GROUP_ARBITER_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace ai_sapiens_sim2real
{
// Device-neutral, allocation-free arbiter. Edges belong to physical sources,
// not the selected output. Mimic consumes edges; a held join switch retries each tick.
class GroupArbiter
{
public:
  enum class Command {Hold, Damping, ReadyPose, Locomotion, Mimic};
  enum class Event
  {
    None, Joined, Left, IndividualLost, GroupLost,
    JoinGroupUnavailable, JoinRobotNotLocomotion, JoinIndividualNotLocomotion,
    JoinGroupNotLocomotion, JoinIndividualVelocityStale, JoinGroupVelocityStale,
    JoinIndividualMoving, JoinGroupMoving
  };
  struct Input
  {
    bool valid{false};
    bool velocity_fresh{false};
    bool join{false};
    Command command{Command::Hold};
    uint16_t selector{0};
    std::array<float, 3> velocity{};
  };
  struct Output
  {
    Event event{Event::None};
    bool participating{false};
    bool exited{false};
    bool request{false};
    Command command{Command::Hold};
    uint16_t selector{0};
    std::array<float, 3> velocity{};
  };

  Output update(const Input & local, const Input & group, bool locomotion, bool posture = false)
  {
    const bool was_participating = participating_;
    const auto edges = read_switch_edges(local, group);
    const auto join_rejection = check_join_conditions(local, group, locomotion);

    update_participation(local, group, join_rejection);

    Output out;
    out.participating = participating_;
    out.exited = was_participating && !participating_;
    out.event = participation_event(local, group, was_participating, edges, join_rejection);
    if (local.valid) {
      out.velocity = combined_velocity(local, group);
      select_command(local, group, locomotion, posture, was_participating, edges, out);
      out.request = out.command != Command::Hold;
    }

    // Consume physical edges even when rejected. Never replay a mimic later.
    previous_local_ = local;
    previous_group_ = group;
    return out;
  }

private:
  struct SwitchEdges
  {
    bool local_mimic;
    bool group_mimic;
    bool local_locomotion;
    bool join;
    bool leave;
  };

  SwitchEdges read_switch_edges(const Input & local, const Input & group) const
  {
    const bool local_continuous = local.valid && previous_local_.valid;
    return {
      edge(local, previous_local_, Command::Mimic),
      edge(group, previous_group_, Command::Mimic),
      edge(local, previous_local_, Command::Locomotion),
      local_continuous && local.join && !previous_local_.join,
      local_continuous && !local.join && previous_local_.join};
  }

  static Event check_join_conditions(const Input & local, const Input & group, bool locomotion)
  {
    // Join only from actual locomotion, with two fresh, individually zero inputs.
    // Held mimic switches are allowed; they do not represent executing motions.
    if (!group.valid) {
      return Event::JoinGroupUnavailable;
    }
    if (!locomotion) {
      return Event::JoinRobotNotLocomotion;
    }
    if (!joinable(local.command)) {
      return Event::JoinIndividualNotLocomotion;
    }
    if (!joinable(group.command)) {
      return Event::JoinGroupNotLocomotion;
    }
    if (!local.velocity_fresh) {
      return Event::JoinIndividualVelocityStale;
    }
    if (!group.velocity_fresh) {
      return Event::JoinGroupVelocityStale;
    }
    if (!neutral(local)) {
      return Event::JoinIndividualMoving;
    }
    if (!neutral(group)) {
      return Event::JoinGroupMoving;
    }
    return Event::None;
  }

  void update_participation(const Input & local, const Input & group, Event join_rejection)
  {
    if (!local.valid || !group.valid || !local.join) {
      participating_ = false;
      return;
    }
    // A held participation switch retries joining. Once joined, motion and
    // stick position changes do not remove the robot from the group.
    if (join_rejection == Event::None) {
      participating_ = true;
    }
  }

  Event participation_event(
    const Input & local, const Input & group, bool was_participating,
    const SwitchEdges & edges, Event join_rejection) const
  {
    if (was_participating && !participating_) {
      if (!local.valid) {
        return Event::IndividualLost;
      }
      if (!group.valid) {
        return Event::GroupLost;
      }
      return Event::Left;
    }
    if (!was_participating && participating_) {
      return Event::Joined;
    }
    // Report rejection once per switch request, not on every held-join retry.
    if (edges.join && !participating_) {
      return join_rejection;
    }
    return Event::None;
  }

  std::array<float, 3> combined_velocity(const Input & local, const Input & group) const
  {
    std::array<float, 3> velocity{};
    for (size_t axis = 0; axis < velocity.size(); ++axis) {
      const float individual = local.velocity_fresh ? local.velocity[axis] : 0.0f;
      const float common = participating_ && group.velocity_fresh ? group.velocity[axis] : 0.0f;
      velocity[axis] = std::clamp(individual + common, -1.0f, 1.0f);
    }
    return velocity;
  }

  void select_command(
    const Input & local, const Input & group, bool locomotion, bool posture,
    bool was_participating, const SwitchEdges & edges, Output & out) const
  {
    // Priority 1: individual damping/ready pose always wins.
    if (local.command == Command::Damping || local.command == Command::ReadyPose) {
      out.command = local.command;
      return;
    }
    // Priority 2: individual startup, participation changes and explicit return
    // request locomotion before considering either mimic source.
    const bool individual_startup = posture && local.command == Command::Locomotion;
    const bool participation_changed = out.exited || out.event == Event::Joined;
    if (individual_startup || participation_changed || edges.local_locomotion) {
      out.command = Command::Locomotion;
      return;
    }
    // Priority 3: fresh mimic events only. Individual retains its posture-start
    // behavior; group mimic requires locomotion and existing participation.
    const bool individual_mimic = (locomotion || posture) && edges.local_mimic;
    const bool group_mimic = locomotion && participating_ && was_participating && edges.group_mimic;
    if (!edges.leave && (individual_mimic || group_mimic)) {
      out.command = Command::Mimic;
      out.selector = edges.local_mimic ? local.selector : group.selector;
      return;
    }
    // Priority 4: the owner's held locomotion applies only while walking.
    // Interrupting mimic requires a fresh locomotion edge.
    const auto & owner = participating_ ? group : local;
    const auto & previous = participating_ ? previous_group_ : previous_local_;
    if (owner.command == Command::Locomotion &&
      (locomotion || edge(owner, previous, Command::Locomotion)))
    {
      out.command = Command::Locomotion;
    }
  }

  // Mimic is an edge-triggered action, not a persistent operating mode.
  static bool joinable(Command command)
  {
    return command == Command::Locomotion || command == Command::Mimic;
  }
  static bool edge(const Input & now, const Input & before, Command command)
  {
    return now.valid && before.valid && now.command == command && before.command != command;
  }
  static bool neutral(const Input & input)
  {
    return input.velocity_fresh && std::all_of(
      input.velocity.begin(), input.velocity.end(), [](float v) {return v == 0.0f;});
  }
  bool participating_{false};
  Input previous_local_;
  Input previous_group_;
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__GROUP_ARBITER_HPP_
