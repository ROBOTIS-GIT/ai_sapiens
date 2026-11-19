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

#ifndef AI_SAPIENS_SIM2REAL__EXCEPTION_DIAGNOSTICS_HPP_
#define AI_SAPIENS_SIM2REAL__EXCEPTION_DIAGNOSTICS_HPP_

#include <exception>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ai_sapiens_sim2real
{

// Log the callback's entry site only when an exception escapes it.
class CallbackLocation
{
public:
  CallbackLocation(const char * function, const char * file, int line) noexcept
  : function_(function), file_(file), line_(line), exception_count_(std::uncaught_exceptions()) {}

  ~CallbackLocation()
  {
    if (std::uncaught_exceptions() > exception_count_) {
      RCUTILS_LOG_ERROR_NAMED(
        "ai_sapiens_sim2real", "Exception escaped callback %s (%s:%d)", function_, file_, line_);
    }
  }

private:
  const char * function_;
  const char * file_;
  int line_;
  int exception_count_;
};

// Called from a catch handler while the exception and ROS logging context are still available.
inline void log_executor_exception(const rclcpp::Logger & logger, const char * operation)
{
  try {
    throw;
  } catch (const rclcpp::exceptions::RCLErrorBase & error) {
    RCLCPP_FATAL(logger, "%s failed (rcl=%d): %s", operation, error.ret,
      error.formatted_message.c_str());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(logger, "%s failed: %s", operation, error.what());
  } catch (...) {
    RCLCPP_FATAL(logger, "%s failed: unknown exception", operation);
  }
}

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__EXCEPTION_DIAGNOSTICS_HPP_
