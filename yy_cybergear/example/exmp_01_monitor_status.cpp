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
#include <poll.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "exmp_helper.hpp"
#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/data_frame_handler.hpp"
#include "yy_cybergear/logging.hpp"
#include "yy_socket_can/can_runtime.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

using exmp_helper::check_for_errors;
using exmp_helper::Clock;
using exmp_helper::preflight_sync;
using exmp_helper::print_params;
using exmp_helper::print_status;
using exmp_helper::register_can_handler;
using exmp_helper::wait_for_enter_or_sigint;

}  // namespace

int main(int argc, char ** argv)
{
  // CLI options (simple): interface, motor id, verbose
  CLI::App app{"Monitor status frames (type 2) for one or more motors"};
  std::string ifname{"can0"};
  std::vector<std::string> motor_id_strs{"0x01"};
  std::string host_id_str{"0x00"};
  bool verbose = false;
  int rate_hz = 10;  // max 200

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app
    .add_option(
      "-M,--motor-id", motor_id_strs,
      "Motor ID(s) (repeat -M or comma-separated, decimal or 0x-hex)")
    ->delimiter(',')
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  // duration option removed: always run until Ctrl+C
  app.add_option("-r,--rate", rate_hz, "Polling rate [Hz] for ClearFaults (>=1, max 200)")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Verbose CAN frame prints");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }
  if (rate_hz > 200) {
    std::cerr << "Safety: capping rate to 200 Hz due to SocketCAN responsiveness (requested "
              << rate_hz << ")\n";
    rate_hz = 200;
  }

  unsigned long host_ul = 0x00UL;
  try {
    host_ul = std::stoul(host_id_str, nullptr, 0);
  } catch (const std::exception & e) {
    std::cerr << "Invalid ID: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
  if (host_ul > 0xFFul) {
    std::cerr << "Host ID must be 0..255 (host=" << host_ul << ")\n";
    return EXIT_FAILURE;
  }
  const uint8_t host = static_cast<uint8_t>(host_ul & 0xFFu);

  // Parse motor IDs
  std::vector<uint8_t> motors;
  motors.reserve(motor_id_strs.size());
  for (const auto & s : motor_id_strs) {
    try {
      unsigned long v = std::stoul(s, nullptr, 0);
      if (v > 0xFFul) {
        std::cerr << "Motor ID out of range (0..255): " << s << "\n";
        return EXIT_FAILURE;
      }
      motors.push_back(static_cast<uint8_t>(v & 0xFFu));
    } catch (const std::exception & e) {
      std::cerr << "Invalid motor ID: '" << s << "' -> " << e.what() << "\n";
      return EXIT_FAILURE;
    }
  }
  if (motors.empty()) {
    std::cerr << "At least one motor ID is required (-M).\n";
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, handle_sigint);

  // Runtime and handler registration
  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  // Create a CyberGear instance per motor
  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.reserve(motors.size());
  for (auto m : motors) cgs.emplace_back(host, m);
  const auto t0 = Clock::now();

  // Track preflight indices (only for issuing requests; readiness is tracked inside CyberGear)
  const std::vector<uint16_t> preflight_params = {
    yy_cybergear::RUN_MODE,
    yy_cybergear::SPEED_LIMIT,
    yy_cybergear::CURRENT_LIMIT,
    yy_cybergear::TORQUE_LIMIT,
    yy_cybergear::CURRENT_KP,
    yy_cybergear::CURRENT_KI,
    yy_cybergear::CURRENT_FILTER_GAIN,
    yy_cybergear::POSITION_KP,
    yy_cybergear::SPEED_KP,
    yy_cybergear::SPEED_KI,
  };

  // Register a handler for all extended CAN IDs and filter in user-space via dispatchAndUpdate.
  register_can_handler(rt, cgs, verbose);

  // Start runtime and wait
  rt.start();
  // Print monitored motors list
  std::cout << "Monitoring motors [";
  for (size_t i = 0; i < motors.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(motors[i]);
  }
  std::cout << std::dec << "] on " << ifname << ", poll ClearFaults at " << rate_hz
            << " Hz. Press Ctrl+C to stop..." << '\n';

  // Preflight: request parameters and UID, then wait (without starting the main loop)
  preflight_sync(g_running, rt, ifname, cgs, preflight_params, verbose);

  // Check for errors after preflight sync
  for (const auto & cg : cgs) {
    if (check_for_errors(cg)) {
      std::cerr << "ERROR: Motor faults detected during initialization. Exiting.\n";
      rt.stop();
      return EXIT_FAILURE;
    }
  }

  // Print collected parameters per motor
  std::cout << "\nCollected parameters (including UID, excluding Status):\n";
  for (const auto & cg : cgs) {
    print_params(cg);
  }
  std::cout << "\nPress Enter to start monitoring (Ctrl+C to exit) ..." << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Periodically post ClearFaults requests
  {
    const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
    while (g_running) {
      const auto start = Clock::now();
      const auto deadline = start + dt_ns;
      const double t_now = std::chrono::duration<double>(start - t0).count();

      // Check for errors in all motors and exit if any are found
      for (const auto & cg : cgs) {
        if (check_for_errors(cg)) {
          std::cerr << "ERROR: Detected motor faults. Exiting monitor loop.\n";
          g_running = false;
          break;
        }
      }
      if (!g_running) break;

      for (const auto & cg : cgs) {
        print_status(cg, t_now);
      }

      for (auto & cg : cgs) {
        struct can_frame tx
        {
        };
        cg.buildClearFaults(tx);
        rt.post(yy_socket_can::TxRequest{ifname, tx});
        if (verbose) {
          const uint32_t id = tx.can_id & CAN_EFF_MASK;
          std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (ClearFaults)"
                    << std::endl;
        }
      }

      std::this_thread::sleep_until(deadline);
    }
  }

  rt.stop();
  // Exit with failure if we stopped due to faults
  return g_running ? EXIT_SUCCESS : EXIT_FAILURE;
}
