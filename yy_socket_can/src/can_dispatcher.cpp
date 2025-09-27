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

#include "yy_socket_can/can_dispatcher.hpp"

#include <algorithm>
#include <sstream>

namespace yy_socket_can
{

void CanDispatcher::register_handler(uint32_t id_begin, uint32_t id_end, Handler cb)
{
  if (!cb) return;
  if (id_end < id_begin) std::swap(id_begin, id_end);
  std::lock_guard<std::mutex> lk(m_);
  entries_.push_back(Entry{id_begin, id_end, std::move(cb)});
}

void CanDispatcher::clear()
{
  std::lock_guard<std::mutex> lk(m_);
  entries_.clear();
}

bool CanDispatcher::dispatch(const struct can_frame & frame) const
{
  const uint32_t id = frame.can_id & CAN_EFF_MASK;
  Handler handler;
  {
    std::lock_guard<std::mutex> lk(m_);
    for (const auto & e : entries_) {
      if (id >= e.begin && id <= e.end) {
        handler = e.cb;
        break;
      }
    }
  }
  if (handler) {
    handler(frame);
    return true;
  }
  if (warn_) {
    std::ostringstream oss;
    oss << "[CanDispatcher] Warning: No handler for CAN ID 0x" << std::hex << id;
    warn_(oss.str());
  }
  return false;
}

void CanDispatcher::set_warning_logger(std::function<void(const std::string &)> logger)
{
  std::lock_guard<std::mutex> lk(m_);
  warn_ = std::move(logger);
}

}  // namespace yy_socket_can
