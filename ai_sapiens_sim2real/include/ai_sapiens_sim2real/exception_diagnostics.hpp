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

#include <rcutils/logging_macros.h>

#include <exception>
#include <utility>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ai_sapiens_sim2real
{

// Bind a ROS callback with its concrete signature so rclcpp can select the overload.
template<typename Owner, typename ... Args>
auto bind_callback(
  Owner * owner, void (Owner::*handler)(Args...), const char * name, const char * file, int line)
{
  return [owner, handler, name, file, line](Args... args) {
           try {
             (owner->*handler)(std::forward<Args>(args)...);
           } catch (...) {
             RCUTILS_LOG_ERROR_NAMED(
               "ai_sapiens_sim2real", "Exception escaped callback %s (registered at %s:%d)",
               name, file, line);
             throw;
           }
         };
}

// Identify input subscriptions and diagnostic timers when an exception reaches their executor.
#define CALLBACK_WITH_EXCEPTION_LOG(owner, handler) \
  ::ai_sapiens_sim2real::bind_callback(owner, &handler, #handler, __FILE__, __LINE__)

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
