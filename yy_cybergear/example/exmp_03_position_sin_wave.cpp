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

// Implement sine-wave position control using CanRuntime + CyberGear helper.
// - Preflight: request parameters + UID and wait until ready
// - Show collected parameters, then wait for Enter to start
// - Set RunMode=2 (run) and enable motors once at start
// - Periodically write POSITION_REFERENCE based on a configurable sine wave
// - Receive frames through a dispatcher and update CyberGear mirrors
// - Print status snapshots from the main loop (thread-safe)

#include <linux/can.h>
#include <poll.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/logging.hpp"
#include "yy_socket_can/can_runtime.hpp"

#include "exmp_helper.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

// Aliases used across helpers (match exmp_01)
using exmp_helper::Clock;

// Centralized logging lives in yy_cybergear/logging.hpp (append command info)
inline void print_status(const yy_cybergear::CyberGear & cg, double t_sec, double cmd_rad)
{
  const std::string status = yy_cybergear::logging::formatStatusLine(cg, t_sec);
  std::ostringstream oss;
  oss << status << " cmd=" << std::fixed << std::setprecision(3) << cmd_rad << "rad";
  std::cout << oss.str() << '\n';
}

using exmp_helper::check_for_errors;
using exmp_helper::preflight_sync;
using exmp_helper::print_params;
using exmp_helper::register_can_handler;
using exmp_helper::wait_for_enter_or_sigint;

}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_04: position sine wave control using CanRuntime (multi-motor)"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01"};
  bool verbose = false;

  double amp_rad = 0.5;
  double freq_hz = 0.5;
  double pos_offset = 0.0;
  double phase_deg = 0.0;
  double phase_step_deg = 0.0;
  int rate_hz = 100;

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
  app.add_option("-A,--amp", amp_rad, "Sine amplitude [rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-f,--freq", freq_hz, "Sine frequency [Hz]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-o,--offset", pos_offset, "Position offset [rad]")->capture_default_str();
  app.add_option("--phase-deg", phase_deg, "Global phase offset [deg]")->capture_default_str();
  app
    .add_option(
      "--phase-step-deg", phase_step_deg, "Additional phase offset per motor index [deg]")
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
    std::cerr << "Invalid host ID: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  if (host_ul > 0xFFul) {
    std::cerr << "Host ID must be 0..255 (got host=" << host_ul << ")\n";
    return EXIT_FAILURE;
  }
  const uint8_t host = static_cast<uint8_t>(host_ul & 0xFFu);

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
      std::cerr << "Invalid motor ID '" << s << "': " << e.what() << '\n';
      return EXIT_FAILURE;
    }
  }
  if (motors.empty()) {
    std::cerr << "No valid motor IDs provided.\n";
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, handle_sigint);
  std::signal(SIGTERM, handle_sigint);

  // Setup CanRuntime
  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  const auto t0 = Clock::now();

  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.reserve(motors.size());
  for (auto m : motors) cgs.emplace_back(host, m);

  // Register a handler for all extended IDs like exmp_01
  register_can_handler(rt, cgs, verbose);

  // Start runtime
  rt.start();

  // Print monitored motors list
  std::cout << "Position sine wave control motors [";
  for (std::size_t i = 0; i < motors.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(motors[i]);
  }
  std::cout << std::dec << "] on " << ifname << ", amp=" << amp_rad << " rad, freq=" << freq_hz
            << " Hz, offset=" << pos_offset << " rad, rate=" << rate_hz << " Hz" << '\n';
  if (phase_deg != 0.0 || phase_step_deg != 0.0) {
    std::cout << "Phase offsets: base=" << phase_deg << " deg, step=" << phase_step_deg
              << " deg per motor index\n";
  }

  const std::vector<uint16_t> preflight_params = {
    yy_cybergear::RUN_MODE,     yy_cybergear::POSITION_REFERENCE,
    yy_cybergear::SPEED_LIMIT,  yy_cybergear::CURRENT_LIMIT,
    yy_cybergear::TORQUE_LIMIT, yy_cybergear::CURRENT_KP,
    yy_cybergear::CURRENT_KI,   yy_cybergear::CURRENT_FILTER_GAIN,
    yy_cybergear::POSITION_KP,  yy_cybergear::SPEED_KP,
    yy_cybergear::SPEED_KI,
  };
  preflight_sync(g_running, rt, ifname, cgs, preflight_params, verbose);
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Check for errors after preflight sync
  for (const auto & cg : cgs) {
    if (check_for_errors(cg)) {
      std::cerr << "ERROR: Motor faults detected during initialization. Exiting.\n";
      rt.stop();
      return EXIT_FAILURE;
    }
  }

  std::cout << "\nCollected parameters (including UID, excluding Status):\n";
  for (const auto & cg : cgs) {
    print_params(cg);
  }

  std::cout << "\nPress Enter to start control (Ctrl+C to exit) ..." << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Set RunMode=Position and Enable motors
  for (auto & cg : cgs) {
    struct can_frame tx
    {
    };
    cg.buildSetRunMode(yy_cybergear::RunMode::Position, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write RUN_MODE=Position)\n";
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

  const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  constexpr double kPi = 3.14159265358979323846;
  const double omega = 2.0 * kPi * freq_hz;
  const double base_phase_rad = phase_deg * (kPi / 180.0);
  const double phase_step_rad = phase_step_deg * (kPi / 180.0);

  std::vector<double> motor_phases;
  motor_phases.reserve(cgs.size());
  for (std::size_t i = 0; i < cgs.size(); ++i) {
    motor_phases.push_back(base_phase_rad + phase_step_rad * static_cast<double>(i));
  }

  std::vector<double> pos_cmds(cgs.size(), pos_offset);

  // Control loop: periodically write POSITION_REFERENCE for each motor
  {
    while (g_running && rt.isRunning()) {
      const auto start = Clock::now();
      const auto deadline = start + dt_ns;
      const double t_now = std::chrono::duration<double>(start - t0).count();

      // Check for errors in all motors and exit if any are found
      for (const auto & cg : cgs) {
        if (check_for_errors(cg)) {
          std::cerr << "ERROR: Stopping all motors due to detected faults.\n";
          g_running = false;
          break;
        }
      }
      if (!g_running) break;

      for (std::size_t i = 0; i < cgs.size(); ++i) {
        const double phase = omega * t_now + motor_phases[i];
        pos_cmds[i] = pos_offset + amp_rad * std::sin(phase);
      }

      for (std::size_t i = 0; i < cgs.size(); ++i) {
        print_status(cgs[i], t_now, pos_cmds[i]);
      }

      for (std::size_t i = 0; i < cgs.size(); ++i) {
        struct can_frame tx
        {
        };
        cgs[i].buildSetPositionReference(static_cast<float>(pos_cmds[i]), tx);
        rt.post(yy_socket_can::TxRequest{ifname, tx});
        if (verbose) {
          const uint32_t id = tx.can_id & CAN_EFF_MASK;
          std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                    << " (Write POSITION_REFERENCE)\n";
        }
      }

      std::this_thread::sleep_until(deadline);
    }
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
