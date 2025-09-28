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

#include "yy_socket_can/can_runtime.hpp"

#include <chrono>
#include <iostream>

namespace yy_socket_can
{

CanRuntime::CanRuntime(bool enable_can_fd)
{
  (void)enable_can_fd;  // reserved for future FD support in Tx API
}

CanRuntime::~CanRuntime() { stop(); }

void CanRuntime::add_channel(
  const std::string & ifname, bool non_blocking, int rcvbuf_bytes, int sndbuf_bytes)
{
  std::unique_ptr<Channel> ch;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (channels_.count(ifname) != 0) return;  // already exists
    SocketCAN sock(ifname, non_blocking, /*enable_can_fd=*/false, rcvbuf_bytes, sndbuf_bytes);
    ch = std::make_unique<Channel>(std::move(sock));
    channels_.emplace(ifname, std::move(ch));
  }
}

void CanRuntime::start()
{
  if (running_.load()) return;
  running_.store(true);

  // Open sockets and spawn RX threads
  {
    std::lock_guard<std::mutex> lk(m_);
    for (auto & kv : channels_) {
      auto & ch = kv.second;
      if (!ch->sock.isOpen()) {
        try {
          ch->sock.open();
        } catch (const std::exception & e) {
          // Log and continue; channel will not start
          std::cerr << "[CanRuntime] Failed to open " << kv.first << ": " << e.what() << std::endl;
          continue;
        }
      }
      ch->running.store(true);
      ch->rx_thread = std::thread([this, ch_ptr = ch.get()]() { rx_worker(ch_ptr); });
    }
  }

  // TX thread
  tx_thread_ = std::thread([this]() { tx_worker(); });
}

void CanRuntime::stop()
{
  if (!running_.load()) return;
  running_.store(false);
  tx_queue_.close();

  if (tx_thread_.joinable()) tx_thread_.join();

  // Stop RX threads and close sockets
  std::lock_guard<std::mutex> lk(m_);
  for (auto & kv : channels_) {
    auto & ch = kv.second;
    ch->running.store(false);
    if (ch->rx_thread.joinable()) ch->rx_thread.join();
    ch->sock.close();
  }
}

void CanRuntime::post(const TxRequest & req) { tx_queue_.push(req); }

void CanRuntime::tx_worker()
{
  TxRequest req;
  while (running_.load()) {
    if (!tx_queue_.wait_and_pop(req)) break;  // queue closed

    // Find channel
    Channel * ch = nullptr;
    {
      std::lock_guard<std::mutex> lk(m_);
      auto it = channels_.find(req.channel);
      if (it != channels_.end()) ch = it->second.get();
    }
    if (!ch) {
      // No such channel; drop with warning
      std::cerr << "[CanRuntime] Warning: TX channel not found: " << req.channel << std::endl;
      continue;
    }

    try {
      ch->sock.send(req.frame);
    } catch (const std::exception & e) {
      std::cerr << "[CanRuntime] TX error on " << req.channel << ": " << e.what() << std::endl;
      // Optional: implement retry/backoff
    }
  }
}

void CanRuntime::rx_worker(Channel * ch)
{
  struct can_frame frame
  {
  };
  while (running_.load() && ch->running.load()) {
    try {
      if (!ch->sock.recv(frame)) {
        continue;  // timeout
      }
      dispatcher_.dispatch(frame);
    } catch (const std::exception & e) {
      std::cerr << "[CanRuntime] RX error on " << ch->sock.ifname() << ": " << e.what()
                << std::endl;
      // Simple recovery: short sleep then continue; in real impl, try reopen etc.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

}  // namespace yy_socket_can
