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

#ifndef AI_SAPIENS_SIM2REAL__MODE_RUNTIME__REALTIME_REQUEST_GATE_HPP_
#define AI_SAPIENS_SIM2REAL__MODE_RUNTIME__REALTIME_REQUEST_GATE_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>

#include <realtime_tools/realtime_buffer.hpp>

namespace ai_sapiens_sim2real
{

/**
 * @brief Single-slot handoff of a request from non-RT callbacks to the RT loop.
 *
 *   - try_accept() (non-RT): accept a request unless one is already in flight or
 *     the epoch moved since the caller validated it.
 *   - consume() (RT): take the in-flight request and release the slot in one
 *     step, so the next request can be accepted immediately. Returns nullopt
 *     when nothing is pending.
 *   - discard() (RT-safe): drop the in-flight request without consuming it, e.g.
 *     when the authority that owns it is revoked.
 *
 * The slot is cleared only by consuming the request that occupies it or by an
 * explicit discard; a request that was never consumed is never silently cleared.
 */
template<typename RequestT>
class RealtimeRequestGate
{
public:
  void init(const RequestT & empty_request = RequestT{})
  {
    buffer_.initRT(empty_request);
  }

  // Revocation counter, bumped by discard(); see try_accept().
  uint64_t epoch() const
  {
    return epoch_.load(std::memory_order_acquire);
  }

  // Rejects the accept if discard() bumped the epoch since expected_epoch was
  // read, i.e. a revocation raced ahead of this request.
  bool try_accept(const RequestT & request, uint64_t expected_epoch)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch_.load(std::memory_order_acquire) != expected_epoch || busy()) {
      return false;
    }

    buffer_.writeFromNonRT(request);
    in_flight_.store(true, std::memory_order_release);
    return true;
  }

  // Returns the in-flight request without taking it, so a caller can decide on
  // it and free the slot later via consume(). nullopt if none is pending.
  std::optional<RequestT> peek()
  {
    if (!in_flight_.load(std::memory_order_acquire)) {
      return std::nullopt;
    }

    const auto * request = buffer_.readFromRT();
    if (request == nullptr) {
      return std::nullopt;
    }

    return *request;
  }

  // Takes the in-flight request and frees the slot. nullopt if none is pending.
  std::optional<RequestT> consume()
  {
    if (!in_flight_.exchange(false, std::memory_order_acq_rel)) {
      return std::nullopt;
    }

    const auto * request = buffer_.readFromRT();
    if (request == nullptr) {
      return std::nullopt;
    }

    return *request;
  }

  // Drops the in-flight request and bumps the epoch. Lock-free for the RT path.
  void discard()
  {
    in_flight_.store(false, std::memory_order_release);
    epoch_.fetch_add(1, std::memory_order_acq_rel);
  }

  bool busy() const
  {
    return in_flight_.load(std::memory_order_acquire);
  }

private:
  // Non-RT callbacks write buffer_; RT update consumes at most one request.
  realtime_tools::RealtimeBuffer<RequestT> buffer_;
  std::atomic<bool> in_flight_{false};
  std::atomic<uint64_t> epoch_{0};

  // Serializes concurrent try_accept calls; the RT consume()/discard() paths
  // are lock-free.
  std::mutex mutex_;
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__MODE_RUNTIME__REALTIME_REQUEST_GATE_HPP_
