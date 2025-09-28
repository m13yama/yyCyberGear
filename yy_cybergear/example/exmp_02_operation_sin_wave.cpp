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

// Implement sine-wave operation control using CanRuntime + CyberGear helper.
// - Preflight: request parameters + UID and wait until ready
// - Show collected parameters, then wait for Enter to start
// - Enable motor once at start, set RunMode=2 (run)
// - Periodically write OP_CONTROL command for each motor (type-1)
// - Receive frames through a single dispatcher and update internal state
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
#include <string>
#include <thread>
#include <vector>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/logging.hpp"
#include "yy_cybergear/protocol_types.hpp"
#include "yy_socket_can/can_runtime.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

// Aliases used across helpers (match exmp_05)
using Clock = std::chrono::steady_clock;

// Centralized logging lives in yy_cybergear/logging.hpp
inline void print_status(const yy_cybergear::CyberGear & cg, double t_sec)
{
  std::cout << yy_cybergear::logging::format_status_line(cg, t_sec) << '\n';
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
                << " dlc=" << int(f.can_dlc) << "\n";
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
  CLI::App app{"exmp_03: sine-wave operation control using CanRuntime (multi-motor)"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01"};
  bool verbose = false;

  // Sine parameters and loop rate
  double amp_rad = 0.5;     // amplitude [rad]
  double freq_hz = 0.5;     // frequency [Hz]
  double kp = 10.0;         // position gain
  double kd = 1.0;          // velocity gain
  double target_vel = 0.0;  // additional target velocity [rad/s]
  double tau_offset = 0.0;  // offset torque [Nm]
  int rate_hz = 100;        // control loop rate [Hz]

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
  app.add_option("--kp", kp, "Position gain (0..500)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--kd", kd, "Velocity gain (0..5)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--target-vel", target_vel, "Additional target velocity [rad/s]")
    ->capture_default_str();
  app.add_option("--tau-offset", tau_offset, "Offset torque [Nm]")->capture_default_str();
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

  std::signal(SIGINT, handle_sigint);
  std::signal(SIGTERM, handle_sigint);

  // Setup CanRuntime
  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  const auto t0 = Clock::now();

  // Create a CyberGear instance per motor
  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.reserve(motors.size());
  for (auto m : motors) cgs.emplace_back(host, m);

  // Register a handler for all extended IDs like exmp_05
  register_can_handler(rt, cgs, verbose);

  // Start runtime
  rt.start();

  // Print monitored motors list
  std::cout << "Sine operation control motors [";
  for (size_t i = 0; i < motors.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(motors[i]);
  }
  std::cout << std::dec << "] on " << ifname << ", amp=" << amp_rad << " rad, freq=" << freq_hz
            << " Hz, kp=" << kp << ", kd=" << kd << ", target_vel=" << target_vel
            << " rad/s, tau_offset=" << tau_offset << " Nm, rate=" << rate_hz << " Hz" << '\n';

  // Preflight: request parameters and UID, then wait (without starting the main loop)
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
  preflight_sync(rt, ifname, cgs, preflight_params, verbose);

  // Print collected parameters per motor
  std::cout << "\nCollected parameters (including UID, excluding Status):\n";
  for (const auto & cg : cgs) {
    print_params(cg);
  }

  std::cout << "\nPress Enter to start control (Ctrl+C to exit) ..." << std::endl;
  if (!wait_for_enter_or_sigint()) {
    rt.stop();
    return EXIT_SUCCESS;
  }

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

  // Control loop: periodically send OP_CONTROL (type-1) for each motor
  {
    const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
    constexpr double kPi = 3.14159265358979323846;
    const double omega = 2.0 * kPi * freq_hz;
    while (g_running && rt.isRunning()) {
      const auto start = Clock::now();
      const auto deadline = start + dt_ns;
      const double t_now = std::chrono::duration<double>(start - t0).count();

      // Print snapshot for each motor
      for (const auto & cg : cgs) {
        print_status(cg, t_now);
      }

      // Build and send OpControl for each motor
      const double pos_ref = amp_rad * std::sin(omega * t_now);
      const double vel_ref = omega * amp_rad * std::cos(omega * t_now) + target_vel;
      yy_cybergear::OpCommand cmd{};
      cmd.pos_rad = static_cast<float>(pos_ref);
      cmd.vel_rad_s = static_cast<float>(vel_ref);
      cmd.kp = static_cast<float>(kp);
      cmd.kd = static_cast<float>(kd);
      cmd.torque_Nm = static_cast<float>(tau_offset);

      for (auto & cg : cgs) {
        struct can_frame tx
        {
        };
        cg.buildOpControl(cmd, tx);
        rt.post(yy_socket_can::TxRequest{ifname, tx});
        if (verbose) {
          const uint32_t id = tx.can_id & CAN_EFF_MASK;
          std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                    << " (OP_CONTROL)\n";
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
