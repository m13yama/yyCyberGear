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

#include <CLI/CLI.hpp>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "yy_cybergear/cybergear.hpp"

using yy_cybergear::CyberGear;

int main(int argc, char ** argv)
{
  // CLI11-based argument parsing
  CLI::App app{"CyberGear: get MCU ID over SocketCAN"};
  std::string ifname{"can0"};
  std::string host_id_str{"0x01"};
  std::string motor_id_str{"0x01"};
  int timeout_ms = 2000;
  bool verbose = false;

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-M,--motor-id", motor_id_str, "Motor ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-t,--timeout-ms", timeout_ms, "Timeout in milliseconds to wait for response")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Enable verbose CAN frame prints");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  // Convert IDs allowing decimal or hex (base 0)
  unsigned long host_id = 0x01UL;
  unsigned long motor_id = 0x01UL;
  try {
    host_id = std::stoul(host_id_str, nullptr, 0);
    motor_id = std::stoul(motor_id_str, nullptr, 0);
  } catch (const std::exception & conv_err) {
    std::cerr << "Invalid ID value: " << conv_err.what() << '\n';
    return EXIT_FAILURE;
  }
  if (host_id > 0xFFul || motor_id > 0xFFul) {
    std::cerr << "IDs must be 0..255 (got host=" << host_id << ", motor=" << motor_id << ")\n";
    return EXIT_FAILURE;
  }

  const uint8_t host = static_cast<uint8_t>(host_id & 0xFFu);
  const uint8_t motor = static_cast<uint8_t>(motor_id & 0xFFu);
  try {
    CyberGear dev(ifname, host, motor, verbose);
    dev.open();

    auto r = dev.getMcuId(timeout_ms);
    if (!r.ok()) {
      std::cerr << "Failed to get MCU ID: " << yy_cybergear::to_string(*r.error()) << ", motor=0x"
                << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned>(motor) << std::dec << ", if=" << ifname << '\n';
      return EXIT_FAILURE;
    }

    const auto & uid = *r.value();
    // Print UID (big-endian hex)
    std::cout << "MCU UID: ";
    std::cout << std::uppercase << std::hex << std::setfill('0');
    for (size_t i = 0; i < uid.size(); ++i) {
      std::cout << std::setw(2) << static_cast<unsigned>(uid[i]);
    }
    std::cout << std::dec << '\n';
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
