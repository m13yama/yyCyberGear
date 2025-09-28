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

  static constexpr int kTxInterFrameDelayUs = 1000;
  static constexpr int kRxPollTimeoutMs = 1;

  explicit CanRuntime(bool enable_can_fd = false);
  ~CanRuntime();

  // Non-copyable/movable
  CanRuntime(const CanRuntime &) = delete;
  CanRuntime & operator=(const CanRuntime &) = delete;

  // Add/open a CAN channel (interface). Safe to call before or after start().
  void add_channel(
    const std::string & ifname, bool non_blocking = true, int rcvbuf_bytes = -1,
    int sndbuf_bytes = -1);

  // Start background threads
  void start();

  // Stop threads and close all channels
  void stop();

  // Submit a TX request; thread-safe
  void post(const TxRequest & req);

  // Query runtime state; returns false once stop has been requested
  bool is_running() const noexcept { return running_.load(); }

  // Register RX handler via dispatcher
  void register_handler(uint32_t id_begin, uint32_t id_end, CanDispatcher::Handler cb)
  {
    dispatcher_.register_handler(id_begin, id_end, std::move(cb));
  }

  void set_warning_logger(std::function<void(const std::string &)> logger)
  {
    dispatcher_.set_warning_logger(std::move(logger));
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
  void tx_worker();
  void rx_worker(Channel * ch);

  // Emergency stop: immediately stop runtime and close sockets (no graceful drain).
  // Safe to call from any thread. Intended for internal error paths.
  void abort();
};

}  // namespace yy_socket_can

#endif  // __YY_SOCKET_CAN_CAN_RUNTIME_HPP__
