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

#ifndef __YY_SOCKET_CAN_CAN_DISPATCHER_HPP__
#define __YY_SOCKET_CAN_CAN_DISPATCHER_HPP__

#include <linux/can.h>

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace yy_socket_can
{

// Dispatcher maps CAN-ID ranges to handlers
class CanDispatcher
{
public:
  using Handler = std::function<void(const struct can_frame &)>;

  // Register handler for inclusive range [id_begin, id_end]
  void registerHandler(uint32_t id_begin, uint32_t id_end, Handler cb);

  // Remove all handlers
  void clear();

  // Dispatch frame to matching handler, returns true if handled
  bool dispatch(const struct can_frame & frame) const;

  // Optional logging hook
  void setWarningLogger(std::function<void(const std::string &)> logger);

private:
  struct Entry
  {
    uint32_t begin;
    uint32_t end;
    Handler cb;
  };

  // mutable to allow logging from const dispatch()
  mutable std::mutex m_;
  std::vector<Entry> entries_;
  std::function<void(const std::string &)> warn_;
};

}  // namespace yy_socket_can

#endif  // __YY_SOCKET_CAN_CAN_DISPATCHER_HPP__
