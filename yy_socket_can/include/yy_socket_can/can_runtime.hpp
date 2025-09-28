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

#ifndef __YY_SOCKET_CAN_CAN_RUNTIME_HPP__
#define __YY_SOCKET_CAN_CAN_RUNTIME_HPP__

#include <linux/can.h>

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "yy_socket_can/can_dispatcher.hpp"
#include "yy_socket_can/socket_can.hpp"
#include "yy_socket_can/thread_safe_queue.hpp"

namespace yy_socket_can
{

struct TxRequest
{
  std::string channel;  // interface name (e.g., can0)
  struct can_frame frame;
};

// High-level runtime: 1 Tx thread + N Rx threads (per channel)
class CanRuntime
{
public:
  using Ptr = std::shared_ptr<CanRuntime>;

  // Time units are milliseconds
  static constexpr int kTxInterFrameDelayMs = 1;  // gap between consecutive TX frames
  static constexpr int kRxPollTimeoutMs = 1;      // RX poll timeout

  explicit CanRuntime(bool enable_can_fd = false);
  ~CanRuntime();

  // Non-copyable/movable
  CanRuntime(const CanRuntime &) = delete;
  CanRuntime & operator=(const CanRuntime &) = delete;

  // Add/open a CAN channel (interface). Safe to call before or after start().
  void addChannel(
    const std::string & ifname, bool non_blocking = true, int rcvbuf_bytes = -1,
    int sndbuf_bytes = -1);

  // Start background threads
  void start();

  // Stop threads and close all channels
  void stop();

  // Submit a TX request; thread-safe
  void post(const TxRequest & req);

  // Query runtime state; returns false once stop has been requested
  bool isRunning() const noexcept { return running_.load(); }

  // Register RX handler via dispatcher
  void registerHandler(uint32_t id_begin, uint32_t id_end, CanDispatcher::Handler cb)
  {
    dispatcher_.registerHandler(id_begin, id_end, std::move(cb));
  }

  void setWarningLogger(std::function<void(const std::string &)> logger)
  {
    dispatcher_.setWarningLogger(std::move(logger));
  }

private:
  struct Channel
  {
    SocketCAN sock;
    std::thread rx_thread;
    std::atomic<bool> running{false};

    explicit Channel(SocketCAN s) : sock(std::move(s)) {}
  };

  // threads
  std::thread tx_thread_;
  std::atomic<bool> running_{false};

  // dispatcher and tx queue
  CanDispatcher dispatcher_{};
  ThreadSafeQueue<TxRequest> tx_queue_{};

  // channel map
  std::mutex m_;
  std::map<std::string, std::unique_ptr<Channel>> channels_;

  // workers
  void txWorker();
  void rxWorker(Channel * ch);

  // Emergency stop: immediately stop runtime and close sockets (no graceful drain).
  // Safe to call from any thread. Intended for internal error paths.
  void abort();
};

}  // namespace yy_socket_can

#endif  // __YY_SOCKET_CAN_CAN_RUNTIME_HPP__
