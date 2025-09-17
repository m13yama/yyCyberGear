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

#ifndef __CANWRAP_SOCKET_CAN_HPP__
#define __CANWRAP_SOCKET_CAN_HPP__

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace yy_socket_can
{

class SocketCAN
{
public:
  explicit SocketCAN(
    std::string ifname, bool non_blocking = false, bool enable_can_fd = false,
    int rcvbuf_bytes = -1, int sndbuf_bytes = -1) noexcept;

  ~SocketCAN();

  SocketCAN(const SocketCAN &) = delete;
  SocketCAN & operator=(const SocketCAN &) = delete;
  SocketCAN(SocketCAN &&) noexcept;
  SocketCAN & operator=(SocketCAN &&) noexcept;

  void open();

  void close() noexcept;

  bool isOpen() const noexcept { return sockfd_ >= 0; }
  int fd() const noexcept { return sockfd_; }
  const std::string & ifname() const noexcept { return ifname_; }
  bool canFdEnabled() const noexcept { return canfd_enabled_; }
  bool nonBlocking() const noexcept { return non_blocking_; }

  void setNonBlocking(bool enable) noexcept { non_blocking_ = enable; }
  void setCanFdEnabled(bool enable) noexcept { canfd_enabled_ = enable; }

  void setLoopback(bool enable);

  void setRecvOwnMsgs(bool enable);

  void setFilters(const std::vector<struct can_filter> & filters);

  void setErrorMask(can_err_mask_t mask);

  std::ptrdiff_t send(const struct can_frame & frame);

  std::ptrdiff_t send(const struct canfd_frame & frame);

  bool recv(struct can_frame & out, int timeout_ms = -1);

  bool recv(struct canfd_frame & out, int timeout_ms = -1);

private:
  int sockfd_{-1};
  std::string ifname_{};
  bool canfd_enabled_{false};
  bool non_blocking_{false};
  int rcvbuf_bytes_{-1};
  int sndbuf_bytes_{-1};

  void ensureOpen(const char * api) const;
};

}  // namespace yy_socket_can

#endif  // __CANWRAP_SOCKET_CAN_HPP__
