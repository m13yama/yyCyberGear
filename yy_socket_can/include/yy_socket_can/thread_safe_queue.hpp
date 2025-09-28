// Copyright 2025 Yuki Yamamoto
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

#ifndef __YY_SOCKET_CAN_THREAD_SAFE_QUEUE_HPP__
#define __YY_SOCKET_CAN_THREAD_SAFE_QUEUE_HPP__

#include <condition_variable>
#include <deque>
#include <mutex>
#include <utility>

namespace yy_socket_can
{

// Simple thread-safe queue with close() to wake waiting consumers
template <typename T>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue() = default;
  ThreadSafeQueue(const ThreadSafeQueue &) = delete;
  ThreadSafeQueue & operator=(const ThreadSafeQueue &) = delete;

  void push(T value)
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (closed_) return;  // drop item if closed
      q_.emplace_back(std::move(value));
    }
    cv_.notify_one();
  }

  // Returns false if closed and no item retrieved
  bool tryPop(T & out)
  {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) return false;
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

  // Blocks until an item is available or queue is closed. Returns false on closed without item.
  bool waitAndPop(T & out)
  {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&] { return closed_ || !q_.empty(); });
    if (q_.empty()) return false;  // closed and empty
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

  void close()
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  bool closed() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return closed_;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lk(m_);
    q_.clear();
  }

private:
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<T> q_;
  bool closed_{false};
};

}  // namespace yy_socket_can

#endif  // __YY_SOCKET_CAN_THREAD_SAFE_QUEUE_HPP__
