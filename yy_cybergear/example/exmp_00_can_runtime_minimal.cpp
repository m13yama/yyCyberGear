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

// clang-format off
#include <linux/can.h>

#include <CLI/CLI.hpp>
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

#include "yy_socket_can/can_runtime.hpp"
#include "yy_cybergear/cybergear.hpp"
// clang-format on

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

void print_status(const yy_cybergear::CyberGear & cg, double t_sec)
{
  std::cout << std::fixed << std::setprecision(3) << " t=" << t_sec << "s"
            << " ang=" << cg.angle_rad() << "rad"
            << " vel=" << cg.vel_rad_s() << "rad/s"
            << " tau=" << cg.torque_Nm() << "Nm"
            << " T=" << cg.temperature_c() << "C"
            << " mode=" << static_cast<unsigned>(cg.mode()) << " faults=0b" << std::uppercase
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(cg.fault_bits()) << std::dec << " mid=0x" << std::uppercase
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(cg.motor_can_id()) << std::dec << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  // CLI options (simple): interface, motor id, duration, verbose
  CLI::App app{"Monitor status frames (type 2) without CyberGear"};
  std::string ifname{"can0"};
  std::string motor_id_str{"0x01"};
  std::string host_id_str{"0x00"};
  double duration = 0.0;  // seconds, 0 => run until Ctrl+C
  bool verbose = false;
  int rate_hz = 1;

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-M,--motor-id", motor_id_str, "Motor ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-d,--duration", duration, "Run duration [s] (0 = infinite)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-r,--rate", rate_hz, "Polling rate [Hz] for ClearFaults (>=1)")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Verbose CAN frame prints");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  unsigned long motor_ul = 0x01UL;
  unsigned long host_ul = 0x00UL;
  try {
    motor_ul = std::stoul(motor_id_str, nullptr, 0);
    host_ul = std::stoul(host_id_str, nullptr, 0);
  } catch (const std::exception & e) {
    std::cerr << "Invalid ID: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
  if (motor_ul > 0xFFul || host_ul > 0xFFul) {
    std::cerr << "IDs must be 0..255 (host=" << host_ul << ", motor=" << motor_ul << ")\n";
    return EXIT_FAILURE;
  }
  const uint8_t motor = static_cast<uint8_t>(motor_ul & 0xFFu);
  const uint8_t host = static_cast<uint8_t>(host_ul & 0xFFu);

  std::signal(SIGINT, handle_sigint);

  // Runtime and handler registration
  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  // Register a handler for all extended CAN IDs and filter in user-space via dispatchAndUpdate.
  const uint32_t min_id = 0u;
  const uint32_t max_id = CAN_EFF_MASK;

  yy_cybergear::CyberGear cg{host, motor};
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();

  rt.registerHandler(min_id, max_id, [verbose, t0, &cg](const struct can_frame & f) {
    const auto kind = cg.dispatchAndUpdate(f);
    if (
      kind == yy_cybergear::CyberGear::UpdateKind::Ignored ||
      kind == yy_cybergear::CyberGear::UpdateKind::None) {
      return;
    }
    const double t = std::chrono::duration<double>(clock::now() - t0).count();
    if (verbose) {
      const uint32_t id = f.can_id & CAN_EFF_MASK;
      std::cout << "RX 0x" << std::hex << std::uppercase << id << std::dec
                << " dlc=" << int(f.can_dlc) << "\n";
    }
    if (kind == yy_cybergear::CyberGear::UpdateKind::Status) {
      print_status(cg, t);
    }
  });

  // Start runtime and wait
  rt.start();
  std::cout << "Monitoring motor 0x" << std::uppercase << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<unsigned>(motor) << std::dec << " on " << ifname
            << ", poll ClearFaults at " << rate_hz << " Hz. Press Ctrl+C to stop..." << '\n';

  // Periodically post ClearFaults requests similar to exmp_02
  const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  while (g_running) {
    const auto start = clock::now();
    const auto deadline = start + dt_ns;

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

    // Stop on duration
    const double t = std::chrono::duration<double>(start - t0).count();
    if (duration > 0.0 && t >= duration) break;

    std::this_thread::sleep_until(deadline);
  }

  rt.stop();
  return EXIT_SUCCESS;
}
