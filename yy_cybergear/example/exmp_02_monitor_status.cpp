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
#include <unordered_map>
#include <vector>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/data_frame_handler.hpp"
#include "yy_socket_can/can_runtime.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

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

void print_params(const yy_cybergear::CyberGear & cg)
{
  // Print a concise parameter summary (limits and gains)
  const auto uid = cg.uid();
  std::cout << std::fixed << std::setprecision(3) << "  motor_id=0x" << std::uppercase << std::hex
            << std::setw(2) << std::setfill('0') << static_cast<unsigned>(cg.motor_id())
            << std::dec << "\n"
            << "    uid               = 0x" << std::uppercase << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<unsigned>(uid[0]) << std::setw(2)
            << static_cast<unsigned>(uid[1]) << std::setw(2) << static_cast<unsigned>(uid[2])
            << std::setw(2) << static_cast<unsigned>(uid[3]) << std::setw(2)
            << static_cast<unsigned>(uid[4]) << std::setw(2) << static_cast<unsigned>(uid[5])
            << std::setw(2) << static_cast<unsigned>(uid[6]) << std::setw(2)
            << static_cast<unsigned>(uid[7]) << std::dec << "\n"
            << "    speed_limit      = " << cg.speed_limit() << " rad/s\n"
            << "    current_limit    = " << cg.current_limit() << " A\n"
            << "    torque_limit     = " << cg.torque_limit() << " Nm\n"
            << "    current_kp       = " << cg.current_kp() << "\n"
            << "    current_ki       = " << cg.current_ki() << "\n"
            << "    current_filter   = " << cg.current_filter_gain() << "\n"
            << "    position_kp      = " << cg.position_kp() << "\n"
            << "    speed_kp         = " << cg.speed_kp() << "\n"
            << "    speed_ki         = " << cg.speed_ki() << "\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  // CLI options (simple): interface, motor id, duration, verbose
  CLI::App app{"Monitor status frames (type 2) for one or more motors"};
  std::string ifname{"can0"};
  std::vector<std::string> motor_id_strs{"0x01"};
  std::string host_id_str{"0x00"};
  double duration = 0.0;  // seconds, 0 => run until Ctrl+C
  bool verbose = false;
  int rate_hz = 1;

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

  // Register a handler for all extended CAN IDs and filter in user-space via dispatchAndUpdate.
  const uint32_t min_id = 0u;
  const uint32_t max_id = CAN_EFF_MASK;

  // Create a CyberGear instance per motor
  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.reserve(motors.size());
  for (auto m : motors) cgs.emplace_back(host, m);
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();

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

  rt.registerHandler(min_id, max_id, [verbose, &cgs](const struct can_frame & f) {
    // Dispatch updates into CG mirrors (will set initialized flags internally)
    for (auto & cg : cgs) (void)cg.dispatchAndUpdate(f);
    if (verbose) {
      const uint32_t id = f.can_id & CAN_EFF_MASK;
      std::cout << "RX 0x" << std::hex << std::uppercase << id << std::dec
                << " dlc=" << int(f.can_dlc) << "\n";
    }
  });

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
  for (std::size_t i = 0; i < cgs.size(); ++i) {
    for (uint16_t idx : preflight_params) {
      struct can_frame tx
      {
      };
      cgs[i].buildReadParam(idx, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (ReadParam 0x"
                  << std::hex << std::uppercase << idx << std::dec << ")\n";
      }
    }
    // Request MCU UID (device ID)
    {
      struct can_frame tx
      {
      };
      cgs[i].buildGetDeviceId(tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (GetDeviceId)\n";
      }
    }
  }

  bool all_ready = false;
  while (g_running && !all_ready) {
    // Check completion using CyberGear's initialized flags
    all_ready = true;
    for (const auto & cg : cgs) {
      if (!cg.isInitializedFor(preflight_params, /*require_uid=*/true)) {
        all_ready = false;
        break;
      }
    }
    if (all_ready) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Print collected parameters per motor
  std::cout << "\nCollected parameters (including UID, excluding Status):\n";
  for (const auto & cg : cgs) {
    print_params(cg);
  }
  std::cout << "\nPress Enter to start monitoring (Ctrl+C to exit) ..." << std::endl;
  while (g_running) {
    if (std::cin.rdbuf()->in_avail() > 0) {
      int c = std::cin.get();
      if (c == '\n' || c == '\r') break;  // start on Enter
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Periodically post ClearFaults requests similar to exmp_02
  const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  while (g_running) {
    const auto start = clock::now();
    const auto deadline = start + dt_ns;
    const double t_now = std::chrono::duration<double>(start - t0).count();

    // Poll and print status snapshots for all motors (main thread only)
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

    // Stop on duration
    if (duration > 0.0 && t_now >= duration) break;

    std::this_thread::sleep_until(deadline);
  }

  rt.stop();
  return EXIT_SUCCESS;
}
