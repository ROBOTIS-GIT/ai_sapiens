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

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

#include "ai_sapiens_sim2real/mode_runtime/realtime_request_gate.hpp"

using ai_sapiens_sim2real::RealtimeRequestGate;

namespace
{

struct CopyBarrier
{
  std::mutex mutex;
  std::condition_variable condition;
  bool enabled{false};
  bool entered{false};
};

class ContendedRequest
{
public:
  ContendedRequest() = default;

  explicit ContendedRequest(int value, std::shared_ptr<CopyBarrier> barrier = nullptr)
  : value(value), barrier(std::move(barrier))
  {
  }

  ContendedRequest(const ContendedRequest & other)
  : value(other.value), barrier(other.barrier)
  {
    wait_while_copy_is_blocked();
  }

  ContendedRequest & operator=(const ContendedRequest & other)
  {
    value = other.value;
    barrier = other.barrier;
    wait_while_copy_is_blocked();
    return *this;
  }

  int value{0};
  std::shared_ptr<CopyBarrier> barrier;

private:
  void wait_while_copy_is_blocked() const
  {
    if (barrier == nullptr) {
      return;
    }

    std::unique_lock<std::mutex> lock(barrier->mutex);
    if (!barrier->enabled) {
      return;
    }

    barrier->entered = true;
    barrier->condition.notify_all();
    barrier->condition.wait(lock, [this]() {return !barrier->enabled;});
  }
};

void wait_for_blocked_copy(const std::shared_ptr<CopyBarrier> & barrier)
{
  std::unique_lock<std::mutex> lock(barrier->mutex);
  ASSERT_TRUE(
    barrier->condition.wait_for(
      lock, std::chrono::seconds(1), [barrier]() {return barrier->entered;}));
}

void release_blocked_copy(const std::shared_ptr<CopyBarrier> & barrier)
{
  std::lock_guard<std::mutex> lock(barrier->mutex);
  barrier->enabled = false;
  barrier->condition.notify_all();
}

}  // namespace

TEST(RealtimeRequestGate, AcceptsWhenIdleAndReportsBusy)
{
  RealtimeRequestGate<int> gate;
  gate.init(0);
  EXPECT_FALSE(gate.busy());
  EXPECT_TRUE(gate.try_accept(42, gate.epoch()));
  EXPECT_TRUE(gate.busy());
}

TEST(RealtimeRequestGate, RejectsWhileBusy)
{
  RealtimeRequestGate<int> gate;
  gate.init(0);
  EXPECT_TRUE(gate.try_accept(1, gate.epoch()));
  EXPECT_FALSE(gate.try_accept(2, gate.epoch()));  // one already in flight
}

TEST(RealtimeRequestGate, ConsumeReturnsRequestAndFreesSlot)
{
  RealtimeRequestGate<int> gate;
  gate.init(0);
  gate.try_accept(7, gate.epoch());

  const auto request = gate.consume();
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(*request, 7);
  EXPECT_FALSE(gate.busy());                 // consume() self-releases
  EXPECT_FALSE(gate.consume().has_value());  // nothing left to take
}

TEST(RealtimeRequestGate, ConsumeWhenEmptyIsNullopt)
{
  RealtimeRequestGate<int> gate;
  gate.init(0);
  EXPECT_FALSE(gate.consume().has_value());
  EXPECT_FALSE(gate.busy());
}

TEST(RealtimeRequestGate, AcceptSucceedsAfterConsume)
{
  RealtimeRequestGate<int> gate;
  gate.init(0);
  gate.try_accept(1, gate.epoch());
  gate.consume();  // normal completion does not bump the epoch

  EXPECT_TRUE(gate.try_accept(2, gate.epoch()));  // slot freed by consume()
  const auto request = gate.consume();
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(*request, 2);
}

TEST(RealtimeRequestGate, DiscardDropsPendingWithoutConsuming)
{
  RealtimeRequestGate<int> gate;
  gate.init(0);
  gate.try_accept(9, gate.epoch());
  gate.discard();

  EXPECT_FALSE(gate.busy());
  EXPECT_FALSE(gate.consume().has_value());        // dropped, never handed out
  EXPECT_TRUE(gate.try_accept(10, gate.epoch()));  // slot is free again
}

TEST(RealtimeRequestGate, RejectsAcceptValidatedBeforeRevocation)
{
  RealtimeRequestGate<int> gate;
  gate.init(0);
  const auto epoch = gate.epoch();
  gate.discard();  // a revocation bumps the epoch

  EXPECT_FALSE(gate.try_accept(5, epoch));        // stale: validated before the revocation
  EXPECT_TRUE(gate.try_accept(5, gate.epoch()));  // a freshly read epoch accepts
}

TEST(RealtimeRequestGate, RejectsRequestDiscardedWhileItIsBeingWritten)
{
  RealtimeRequestGate<ContendedRequest> gate;
  gate.init();

  const auto barrier = std::make_shared<CopyBarrier>();
  barrier->enabled = true;
  bool accepted = true;
  std::thread producer(
    [&gate, &accepted, barrier]() {
      accepted = gate.try_accept(ContendedRequest(11, barrier), gate.epoch());
    });

  wait_for_blocked_copy(barrier);
  gate.discard();
  release_blocked_copy(barrier);
  producer.join();

  EXPECT_FALSE(accepted);
  EXPECT_FALSE(gate.busy());
  EXPECT_FALSE(gate.consume().has_value());
}

TEST(RealtimeRequestGate, KeepsSlotReservedUntilConsumeFinishesCopying)
{
  RealtimeRequestGate<ContendedRequest> gate;
  gate.init();

  const auto barrier = std::make_shared<CopyBarrier>();
  EXPECT_TRUE(gate.try_accept(ContendedRequest(21, barrier), gate.epoch()));

  barrier->enabled = true;
  std::optional<ContendedRequest> consumed;
  std::thread consumer([&gate, &consumed]() {consumed = gate.consume();});

  wait_for_blocked_copy(barrier);
  EXPECT_FALSE(gate.try_accept(ContendedRequest(22), gate.epoch()));
  release_blocked_copy(barrier);
  consumer.join();

  ASSERT_TRUE(consumed.has_value());
  EXPECT_EQ(consumed->value, 21);
  EXPECT_FALSE(gate.busy());
}
