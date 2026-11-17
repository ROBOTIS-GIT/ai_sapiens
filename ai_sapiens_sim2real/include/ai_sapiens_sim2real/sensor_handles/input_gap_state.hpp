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

#ifndef AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__INPUT_GAP_STATE_HPP_
#define AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__INPUT_GAP_STATE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace ai_sapiens_sim2real
{

// One warning per gap, including gaps whose timer could not run before recovery.
class InputGapState
{
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  struct Event
  {
    bool is_warning{false};
    bool is_recovery{false};
    double gap_ms{0.0};
  };

  explicit InputGapState(double expected_rate_hz, Time start = Clock::now())
  : last_received_(start)
  {
    if (!std::isfinite(expected_rate_hz) || expected_rate_hz <= 0.0) {
      throw std::invalid_argument("expected input rate must be finite and positive");
    }

    threshold_ = std::chrono::duration<double>(std::max(0.05, 5.0 / expected_rate_hz));
  }

  Event record_reception(Time now)
  {
    auto event = check_gap(now);
    event.is_recovery = is_warned_;
    is_warned_ = false;
    last_received_ = now;
    return event;
  }

  Event check_gap(Time now)
  {
    const auto gap = now - last_received_;
    Event event;
    event.gap_ms = std::chrono::duration<double, std::milli>(gap).count();

    if (gap >= threshold_ && !is_warned_) {
      event.is_warning = true;
      is_warned_ = true;
    }

    return event;
  }

private:
  Time last_received_;
  std::chrono::duration<double> threshold_{};
  bool is_warned_{false};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__SENSOR_HANDLES__INPUT_GAP_STATE_HPP_
