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

// Implement constant speed control using CanRuntime + CyberGear helper.
// - Enable motor once at start, set RunMode=2 (run)
// - Periodically write SPEED_REFERENCE parameter for each motor
// - Receive frames through a single dispatcher and update internal state
// - Print status snapshots from the main loop (thread-safe)

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
#include <vector>

#include "yy_cybergear/cybergear.hpp"
#include "yy_socket_can/can_runtime.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_signal(int) { g_running = false; }

void print_status(const yy_cybergear::CyberGear & cg, double t_sec)
{
  const auto s = cg.getStatus();
  std::cout << std::fixed << std::setprecision(3) << " t=" << t_sec << "s"
            << " ang=" << s.angle_rad << "rad"
            << " vel=" << s.vel_rad_s << "rad/s"
            << " tau=" << s.torque_Nm << "Nm"
            << " T=" << s.temperature_c << "C"
            << " mode=" << static_cast<unsigned>(s.mode) << " faults=0b" << std::uppercase
            << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(s.fault_bits)
            << std::dec << " mid=0x" << std::uppercase << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<unsigned>(s.motor_can_id) << std::dec << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_07: constant speed control using CanRuntime"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01"};
  bool verbose = false;

  double speed_rad_s = 2.0;  // target speed [rad/s]
  int rate_hz = 100;         // control loop rate [Hz]

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app
    .add_option(
      "-M,--motor-id", motor_id_strs,
      "Motor ID(s) (repeat -M or comma-separated; decimal or 0x-prefixed hex)")
    ->delimiter(',')
    ->capture_default_str();
  app.add_option("-s,--speed", speed_rad_s, "Target speed [rad/s] (negative allowed)")
    ->capture_default_str();
  app.add_option("-r,--rate", rate_hz, "Control loop rate [Hz]")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Verbose CAN frame prints");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  unsigned long host_ul = 0x00UL;
  try {
    host_ul = std::stoul(host_id_str, nullptr, 0);
  } catch (const std::exception & e) {
    std::cerr << "Invalid ID: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
  if (host_ul > 0xFFul) {
    std::cerr << "Host ID must be 0..255 (got host=" << host_ul << ")\n";
    return EXIT_FAILURE;
  }
  const uint8_t host = static_cast<uint8_t>(host_ul & 0xFFu);

  // Parse multiple motor IDs
  std::vector<uint8_t> motors;
  motors.reserve(motor_id_strs.size());
  for (const auto & s : motor_id_strs) {
    try {
      unsigned long v = std::stoul(s, nullptr, 0);
      if (v > 0xFFul) {
        std::cerr << "Motor ID must be 0..255 (got " << v << ")\n";
        return EXIT_FAILURE;
      }
      motors.push_back(static_cast<uint8_t>(v & 0xFFu));
    } catch (const std::exception & e) {
      std::cerr << "Invalid motor ID '" << s << "': " << e.what() << "\n";
      return EXIT_FAILURE;
    }
  }
  if (motors.empty()) {
    std::cerr << "No valid motor IDs provided.\n";
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  // Setup CanRuntime
  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();

  // Create a CyberGear instance per motor
  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.reserve(motors.size());
  for (auto m : motors) cgs.emplace_back(host, m);

  // Register a handler for all extended IDs and let CyberGear filter/parse
  const uint32_t min_id = 0u;
  const uint32_t max_id = CAN_EFF_MASK;
  rt.registerHandler(min_id, max_id, [verbose, &cgs](const struct can_frame & f) {
    for (auto & cg : cgs) {
      const auto kind = cg.dispatchAndUpdate(f);
      if (
        kind == yy_cybergear::CyberGear::UpdateKind::Ignored ||
        kind == yy_cybergear::CyberGear::UpdateKind::None) {
        continue;
      }
      if (verbose) {
        const uint32_t id = f.can_id & CAN_EFF_MASK;
        std::cout << "RX 0x" << std::hex << std::uppercase << id << std::dec
                  << " dlc=" << int(f.can_dlc) << "\n";
      }
      // No break: multiple instances theoretically could match
    }
  });

  // Start runtime
  rt.start();

  // Print monitored motors list
  std::cout << "Speed control motors [";
  for (size_t i = 0; i < motors.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(motors[i]);
  }
  std::cout << std::dec << "] on " << ifname << ", target=" << speed_rad_s
            << " rad/s, rate=" << rate_hz << " Hz" << '\n';

  // Set RunMode=2 (run) and Enable motors
  for (auto & cg : cgs) {
    struct can_frame tx
    {
    };
    cg.buildSetRunMode(2u, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write RUN_MODE=2)\n";
    }
  }
  for (auto & cg : cgs) {
    struct can_frame tx
    {
    };
    cg.buildEnable(tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Enable)\n";
    }
  }

  // Control loop: set RunMode=Speed and speed reference
  const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  while (g_running && rt.isRunning()) {
    const auto start = clock::now();
    const auto deadline = start + dt_ns;
    const double t_now = std::chrono::duration<double>(start - t0).count();

    // Print snapshot for each motor
    for (const auto & cg : cgs) {
      print_status(cg, t_now);
    }

    // Write SPEED_REFERENCE (0x700A) for each motor
    for (auto & cg : cgs) {
      struct can_frame tx
      {
      };
      cg.buildSetSpeedReference(static_cast<float>(speed_rad_s), tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write SPEED_REFERENCE)\n";
      }
    }

    // Stop if fault occurred (optional: user could watch last status via shared var)

    std::this_thread::sleep_until(deadline);
  }

  // Stop motors safely on exit
  for (auto & cg : cgs) {
    struct can_frame tx
    {
    };
    cg.buildStop(tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Stop)\n";
    }
  }

  rt.stop();
  return EXIT_SUCCESS;
}
