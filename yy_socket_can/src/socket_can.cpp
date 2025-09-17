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

#include "yy_socket_can/socket_can.hpp"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace yy_socket_can
{

SocketCAN::SocketCAN(
  std::string ifname, bool non_blocking, bool enable_can_fd, int rcvbuf_bytes,
  int sndbuf_bytes) noexcept
: ifname_(std::move(ifname)),
  canfd_enabled_(enable_can_fd),
  non_blocking_(non_blocking),
  rcvbuf_bytes_(rcvbuf_bytes),
  sndbuf_bytes_(sndbuf_bytes)
{
}

SocketCAN::~SocketCAN() { close(); }

SocketCAN::SocketCAN(SocketCAN && other) noexcept
{
  sockfd_ = other.sockfd_;
  ifname_ = std::move(other.ifname_);
  canfd_enabled_ = other.canfd_enabled_;
  non_blocking_ = other.non_blocking_;
  rcvbuf_bytes_ = other.rcvbuf_bytes_;
  sndbuf_bytes_ = other.sndbuf_bytes_;
  other.sockfd_ = -1;
}

SocketCAN & SocketCAN::operator=(SocketCAN && other) noexcept
{
  if (this != &other) {
    close();
    sockfd_ = other.sockfd_;
    ifname_ = std::move(other.ifname_);
    canfd_enabled_ = other.canfd_enabled_;
    non_blocking_ = other.non_blocking_;
    rcvbuf_bytes_ = other.rcvbuf_bytes_;
    sndbuf_bytes_ = other.sndbuf_bytes_;
    other.sockfd_ = -1;
  }
  return *this;
}

void SocketCAN::ensureOpen(const char * api) const
{
  if (sockfd_ < 0) {
    throw std::runtime_error(std::string(api) + ": socket not open");
  }
}

void SocketCAN::open()
{
  if (sockfd_ >= 0) return;  // already open

  // Create RAW CAN or CAN FD socket
  const int type = SOCK_RAW;
  const int proto = CAN_RAW;
  int fd = ::socket(PF_CAN, type, proto);
  if (fd < 0) {
    throw std::runtime_error(
      std::string("socket(PF_CAN) failed: ") + std::string(std::strerror(errno)));
  }

  // Configure CAN FD if requested
  if (canfd_enabled_) {
    int enable = 1;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
      ::close(fd);
      throw std::runtime_error(
        std::string("setsockopt(CAN_RAW_FD_FRAMES) failed: ") + std::string(std::strerror(errno)));
    }
  }

  // Set non-blocking if requested
  if (non_blocking_) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      ::close(fd);
      throw std::runtime_error(
        std::string("fcntl(O_NONBLOCK) failed: ") + std::string(std::strerror(errno)));
    }
  }

  // Configure socket buffer sizes if requested (must be before bind)
  if (rcvbuf_bytes_ > 0) {
    int val = rcvbuf_bytes_;
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &val, sizeof(val)) < 0) {
      ::close(fd);
      throw std::runtime_error(
        std::string("setsockopt(SO_RCVBUF) failed: ") + std::string(std::strerror(errno)));
    }
  }
  if (sndbuf_bytes_ > 0) {
    int val = sndbuf_bytes_;
    if (::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &val, sizeof(val)) < 0) {
      ::close(fd);
      throw std::runtime_error(
        std::string("setsockopt(SO_SNDBUF) failed: ") + std::string(std::strerror(errno)));
    }
  }

  // Bind to interface
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname_.c_str());
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd);
    throw std::runtime_error(
      std::string("ioctl(SIOCGIFINDEX) failed: ") + std::string(std::strerror(errno)));
  }
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    throw std::runtime_error(
      std::string("bind(AF_CAN) failed: ") + std::string(std::strerror(errno)));
  }

  sockfd_ = fd;
}

void SocketCAN::close() noexcept
{
  if (sockfd_ >= 0) {
    ::close(sockfd_);
    sockfd_ = -1;
  }
}

void SocketCAN::setLoopback(bool enable)
{
  ensureOpen("setLoopback");
  int opt = enable ? 1 : 0;
  if (::setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &opt, sizeof(opt)) < 0) {
    throw std::runtime_error(
      std::string("setsockopt(CAN_RAW_LOOPBACK) failed: ") + std::string(std::strerror(errno)));
  }
}

void SocketCAN::setRecvOwnMsgs(bool enable)
{
  ensureOpen("setRecvOwnMsgs");
  int opt = enable ? 1 : 0;
  if (::setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &opt, sizeof(opt)) < 0) {
    throw std::runtime_error(
      std::string("setsockopt(CAN_RAW_RECV_OWN_MSGS) failed: ") +
      std::string(std::strerror(errno)));
  }
}

void SocketCAN::setFilters(const std::vector<struct can_filter> & filters)
{
  ensureOpen("setFilters");
  if (filters.empty()) {
    // disable filtering -> receive all
    if (::setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) < 0) {
      throw std::runtime_error(
        std::string("setsockopt(CAN_RAW_FILTER, nullptr) failed: ") +
        std::string(std::strerror(errno)));
    }
    return;
  }
  if (
    ::setsockopt(
      sockfd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) < 0) {
    throw std::runtime_error(
      std::string("setsockopt(CAN_RAW_FILTER) failed: ") + std::string(std::strerror(errno)));
  }
}

void SocketCAN::setErrorMask(can_err_mask_t mask)
{
  ensureOpen("setErrorMask");
  if (::setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask, sizeof(mask)) < 0) {
    throw std::runtime_error(
      std::string("setsockopt(CAN_RAW_ERR_FILTER) failed: ") + std::string(std::strerror(errno)));
  }
}

std::ptrdiff_t SocketCAN::send(const struct can_frame & frame)
{
  ensureOpen("send(can_frame)");
  const ssize_t n = ::write(sockfd_, &frame, sizeof(frame));
  if (n < 0) {
    throw std::runtime_error(
      std::string("write(can_frame) failed: ") + std::string(std::strerror(errno)));
  }
  return n;
}

std::ptrdiff_t SocketCAN::send(const struct canfd_frame & frame)
{
  ensureOpen("send(canfd_frame)");
  if (!canfd_enabled_) {
    throw std::runtime_error("CAN FD not enabled for this socket");
  }
  const ssize_t n = ::write(sockfd_, &frame, sizeof(frame));
  if (n < 0) {
    throw std::runtime_error(
      std::string("write(canfd_frame) failed: ") + std::string(std::strerror(errno)));
  }
  return n;
}

static bool wait_fd(int fd, bool read, int timeout_ms)
{
  if (timeout_ms < 0) return true;  // blocking => no pre-wait
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd, &fds);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  int ret = ::select(fd + 1, read ? &fds : nullptr, read ? nullptr : &fds, nullptr, &tv);
  if (ret < 0) {
    throw std::runtime_error(std::string("select() failed: ") + std::string(std::strerror(errno)));
  }
  return ret > 0;
}

bool SocketCAN::recv(struct can_frame & out, int timeout_ms)
{
  ensureOpen("recv(can_frame)");
  if (!wait_fd(sockfd_, true, timeout_ms)) {
    return false;  // timeout
  }
  const ssize_t n = ::read(sockfd_, &out, sizeof(out));
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    throw std::runtime_error(
      std::string("read(can_frame) failed: ") + std::string(std::strerror(errno)));
  }
  if (n != sizeof(out)) {
    throw std::runtime_error("short read for can_frame");
  }
  return true;
}

bool SocketCAN::recv(struct canfd_frame & out, int timeout_ms)
{
  ensureOpen("recv(canfd_frame)");
  if (!canfd_enabled_) {
    throw std::runtime_error("CAN FD not enabled for this socket");
  }
  if (!wait_fd(sockfd_, true, timeout_ms)) {
    return false;  // timeout
  }
  const ssize_t n = ::read(sockfd_, &out, sizeof(out));
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    throw std::runtime_error(
      std::string("read(canfd_frame) failed: ") + std::string(std::strerror(errno)));
  }
  if (n != sizeof(out)) {
    throw std::runtime_error("short read for canfd_frame");
  }
  return true;
}

}  // namespace yy_socket_can
