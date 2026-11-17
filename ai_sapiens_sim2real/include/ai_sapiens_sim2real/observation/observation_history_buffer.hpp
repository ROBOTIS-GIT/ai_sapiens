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
// Author: Woojin Wie

// Copyright 2026 The SuperMimic Authors.
// History handling extracted from BlinkMimic/SuperMimic observations.

#ifndef AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_HISTORY_BUFFER_HPP_
#define AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_HISTORY_BUFFER_HPP_

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace ai_sapiens_sim2real
{

// Keeps chronological frames and a snapshot for rejecting the latest inference sample.
class ObservationHistoryBuffer
{
public:
  ObservationHistoryBuffer(size_t frame_width, size_t frame_count)
  : frame_width_(frame_width), frame_count_(frame_count)
  {
    if (frame_width == 0 || frame_count == 0) {
      throw std::invalid_argument("Observation history dimensions must be positive");
    }
    frames_.reserve(frame_width * frame_count);
    previous_frames_.reserve(frame_width * frame_count);
  }

  void reset()
  {
    frames_.clear();
    discard_snapshot();
  }

  void begin_sample()
  {
    previous_frames_ = frames_;
    has_snapshot_ = true;
  }

  void append(const std::vector<float> & frame)
  {
    if (frame.size() != frame_width_) {
      throw std::invalid_argument("Observation history frame size mismatch");
    }

    if (frames_.empty()) {
      for (size_t i = 0; i < frame_count_; ++i) {
        frames_.insert(frames_.end(), frame.begin(), frame.end());
      }
      return;
    }

    std::rotate(frames_.begin(), frames_.begin() + frame_width_, frames_.end());
    std::copy(frame.begin(), frame.end(), frames_.end() - frame_width_);
  }

  bool reject_latest()
  {
    if (!has_snapshot_) {return false;}

    frames_ = previous_frames_;
    discard_snapshot();
    return true;
  }

  void discard_snapshot()
  {
    has_snapshot_ = false;
    previous_frames_.clear();
  }

  const std::vector<float> & frames() const {return frames_;}

private:
  size_t frame_width_;
  size_t frame_count_;
  std::vector<float> frames_;
  std::vector<float> previous_frames_;
  bool has_snapshot_{false};
};

}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_HISTORY_BUFFER_HPP_
