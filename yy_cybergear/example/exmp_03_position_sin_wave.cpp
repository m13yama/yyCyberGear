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

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

// Aliases used across helpers (match exmp_01)
using Clock = std::chrono::steady_clock;

// Centralized logging lives in yy_cybergear/logging.hpp (append command info)
inline void print_status(const yy_cybergear::CyberGear & cg, double t_sec, double cmd_rad)
{
  const std::string status = yy_cybergear::logging::format_status_line(cg, t_sec);
  std::ostringstream oss;
  oss << status << " cmd=" << std::fixed << std::setprecision(3) << cmd_rad << "rad";
  std::cout << oss.str() << '\n';
}

inline void print_params(const yy_cybergear::CyberGear & cg)
{
  std::cout << yy_cybergear::logging::format_params_summary(cg);
}

// Register a wide handler and dispatch frames into all CyberGear mirrors
inline void register_can_handler(
  yy_socket_can::CanRuntime & rt, std::vector<yy_cybergear::CyberGear> & cgs, bool verbose)
{
  const uint32_t min_id = 0u;
  const uint32_t max_id = CAN_EFF_MASK;
  rt.registerHandler(min_id, max_id, [verbose, &cgs](const struct can_frame & f) {
    for (auto & cg : cgs) (void)cg.dispatchAndUpdate(f);
    if (verbose) {
      const uint32_t id = f.can_id & CAN_EFF_MASK;
      std::cout << "RX 0x" << std::hex << std::uppercase << id << std::dec
                << " dlc=" << int(f.can_dlc) << '\n';
    }
  });
}

// Preflight: issue initial requests and wait until all are ready (with retries)
inline void preflight_sync(
  yy_socket_can::CanRuntime & rt, const std::string & ifname,
  std::vector<yy_cybergear::CyberGear> & cgs, const std::vector<uint16_t> & preflight_params,
  bool verbose)
{
  // Initial issue of ReadParam and GetDeviceId
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

  // Wait for completion with periodic retries
  bool all_ready = false;
  auto last_retry = Clock::now();
  while (g_running && !all_ready) {
    all_ready = true;
    for (const auto & cg : cgs) {
      if (!cg.isInitializedFor(preflight_params, /*require_uid=*/true)) {
        all_ready = false;
        break;
      }
    }
    if (all_ready) break;

    const auto now = Clock::now();
    if (now - last_retry >= std::chrono::milliseconds(200)) {
      last_retry = now;
      for (std::size_t i = 0; i < cgs.size(); ++i) {
        const auto & cg = cgs[i];
        for (uint16_t idx : preflight_params) {
          if (!cg.isParamInitialized(idx)) {
            struct can_frame tx
            {
            };
            cgs[i].buildReadParam(idx, tx);
            rt.post(yy_socket_can::TxRequest{ifname, tx});
            if (verbose) {
              const uint32_t id = tx.can_id & CAN_EFF_MASK;
              std::cout << "RETRY TX 0x" << std::hex << std::uppercase << id << std::dec
                        << " (ReadParam 0x" << std::hex << std::uppercase << idx << std::dec
                        << ")\n";
            }
          }
        }
        if (!cg.isUidInitialized()) {
          struct can_frame tx
          {
          };
          cgs[i].buildGetDeviceId(tx);
          rt.post(yy_socket_can::TxRequest{ifname, tx});
          if (verbose) {
            const uint32_t id = tx.can_id & CAN_EFF_MASK;
            std::cout << "RETRY TX 0x" << std::hex << std::uppercase << id << std::dec
                      << " (GetDeviceId)\n";
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

// Wait until user presses Enter (or SIGINT occurs). Returns true if Enter was pressed.
inline bool wait_for_enter_or_sigint()
{
  while (g_running) {
    struct pollfd pfd;
    pfd.fd = 0;  // stdin
    pfd.events = POLLIN;
    pfd.revents = 0;
    const int pret = ::poll(&pfd, 1, 200);  // 200 ms timeout
    if (pret > 0 && (pfd.revents & POLLIN)) {
      char ch = 0;
      const ssize_t n = ::read(0, &ch, 1);
      if (n > 0 && (ch == '\n' || ch == '\r')) return true;  // start on Enter
      // If other characters were typed, keep polling until newline arrives
    }
  }
  return false;
}

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
  preflight_sync(rt, ifname, cgs, preflight_params, verbose);
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  std::cout << "\nCollected parameters (including UID, excluding Status):\n";
  for (const auto & cg : cgs) {
    print_params(cg);
  }

  std::cout << "\nPress Enter to start control (Ctrl+C to exit) ..." << std::endl;
  if (!wait_for_enter_or_sigint()) {
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
