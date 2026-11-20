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

#ifndef ROS_LOG_CAPTURE_HPP_
#define ROS_LOG_CAPTURE_HPP_

#include <rcutils/logging.h>

#include <cstdarg>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <utility>


namespace ai_sapiens_sim2real::test
{

// Install before spinning a test node and restore after all its logging threads have joined.
class RosLogCapture
{
public:
  explicit RosLogCapture(std::string logger, std::function<void()> on_log = {})
  : logger_(std::move(logger)), on_log_(std::move(on_log)),
    previous_(rcutils_logging_get_output_handler())
  {
    active_ = this;
    rcutils_logging_set_output_handler(&capture);
  }

  ~RosLogCapture()
  {
    rcutils_logging_set_output_handler(previous_);
    active_ = nullptr;
  }

  RosLogCapture(const RosLogCapture &) = delete;
  RosLogCapture & operator=(const RosLogCapture &) = delete;

  std::string take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(messages_, {});
  }

private:
  static void capture(
    const rcutils_log_location_t * location, int severity, const char * name,
    rcutils_time_point_value_t stamp, const char * format, va_list * args)
  {
    auto & self = *active_;
    if (!name || self.logger_ != name) {
      if (self.previous_) {
        self.previous_(location, severity, name, stamp, format, args);
      }
      return;
    }
    char message[2048];
    std::vsnprintf(message, sizeof(message), format, *args);
    {
      std::lock_guard<std::mutex> lock(self.mutex_);
      self.messages_.append(message).append("\n");
    }
    if (self.on_log_) {
      self.on_log_();
    }
  }

  inline static RosLogCapture * active_{nullptr};
  std::string logger_;
  std::function<void()> on_log_;
  rcutils_logging_output_handler_t previous_;
  std::mutex mutex_;
  std::string messages_;
};

}  // namespace ai_sapiens_sim2real::test

#endif  // ROS_LOG_CAPTURE_HPP_
