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

#include <linux/can.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "yy_socket_can/can_runtime.hpp"

static void print_frame(const struct can_frame & f)
{
  const uint32_t id = f.can_id & CAN_EFF_MASK;
  std::cout << "RX 0x" << std::hex << std::uppercase << id << std::dec
            << " dlc=" << int(f.can_dlc) << " data=";
  std::cout << std::hex << std::uppercase << std::setfill('0');
  for (int i = 0; i < f.can_dlc; ++i) {
    std::cout << ' ' << std::setw(2) << int(f.data[i]);
  }
  std::cout << std::dec << std::endl;
}

int main(int argc, char ** argv)
{
  const char * ifname = (argc > 1) ? argv[1] : "can0";  // interface can be passed as argv[1]

  yy_socket_can::CanRuntime rt;
  rt.set_warning_logger([](const std::string & m) { std::cerr << m << std::endl; });

  // Add single channel and a simple handler for 0x100-0x1FF
  rt.add_channel(ifname);
  rt.register_handler(0x100, 0x1FF, [](const struct can_frame & f) { print_frame(f); });

  // Start runtime
  rt.start();

  // Send one sample frame into the bus (ID 0x123)
  struct can_frame tx{};
  tx.can_id = 0x123;  // standard frame
  tx.can_dlc = 2;
  tx.data[0] = 0xDE;
  tx.data[1] = 0xAD;
  rt.post(yy_socket_can::TxRequest{ifname, tx});

  // Keep running a bit to receive frames
  std::this_thread::sleep_for(std::chrono::seconds(2));

  rt.stop();
  return EXIT_SUCCESS;
}
