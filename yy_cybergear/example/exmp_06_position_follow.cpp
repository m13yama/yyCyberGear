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

// Follow-position control: use the lowest-ID motor as master (source of angle),
// and command all other motors to follow its mechanical angle using Position mode.
// Uses CanRuntime + CyberGear helper, multi-motor friendly.

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
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/logging.hpp"
#include "yy_socket_can/can_runtime.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }
using Clock = std::chrono::steady_clock;

inline void print_status(const yy_cybergear::CyberGear & cg, double t_sec)
{
  std::cout << yy_cybergear::logging::formatStatusLine(cg, t_sec) << '\n';
}
inline void print_params(const yy_cybergear::CyberGear & cg)
{
  std::cout << yy_cybergear::logging::formatParamsSummary(cg);
}

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

inline void preflight_sync(
  yy_socket_can::CanRuntime & rt, const std::string & ifname,
  std::vector<yy_cybergear::CyberGear> & cgs, const std::vector<uint16_t> & preflight_params,
  bool verbose)
{
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

inline bool wait_for_enter_or_sigint()
{
  while (g_running) {
    struct pollfd pfd;
    pfd.fd = 0;
    pfd.events = POLLIN;
    pfd.revents = 0;
    const int pret = ::poll(&pfd, 1, 200);
    if (pret > 0 && (pfd.revents & POLLIN)) {
      char ch = 0;
      const ssize_t n = ::read(0, &ch, 1);
      if (n > 0 && (ch == '\n' || ch == '\r')) return true;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_06: position follow lowest-ID motor"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01", "0x02"};
  bool verbose = false;
  int rate_hz = 200;  // faster to track position smoothly

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
  if (motors.size() < 2) {
    std::cerr << "Provide at least two motor IDs to follow (master + follower).\n";
    return EXIT_FAILURE;
  }

  std::sort(motors.begin(), motors.end());
  const uint8_t master_id = motors.front();

  std::signal(SIGINT, handle_sigint);
  std::signal(SIGTERM, handle_sigint);

  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  const auto t0 = Clock::now();

  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.reserve(motors.size());
  for (auto m : motors) cgs.emplace_back(host, m);

  register_can_handler(rt, cgs, verbose);
  rt.start();

  std::cout << "Position follow: master=0x" << std::uppercase << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<unsigned>(master_id) << std::dec << ", followers=";
  for (size_t i = 1; i < motors.size(); ++i) {
    if (i > 1) std::cout << ", ";
    std::cout << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(motors[i]);
  }
  std::cout << std::dec << " on " << ifname << ", rate=" << rate_hz << " Hz" << '\n';

  const std::vector<uint16_t> preflight_params = {
    yy_cybergear::RUN_MODE,
    yy_cybergear::POSITION_REFERENCE,
    yy_cybergear::SPEED_LIMIT,
    yy_cybergear::POSITION_KP,
    yy_cybergear::SPEED_KP,
    yy_cybergear::SPEED_KI,
    yy_cybergear::MECHANICAL_POSITION,
    yy_cybergear::MECHANICAL_VELOCITY,
  };
  preflight_sync(rt, ifname, cgs, preflight_params, verbose);

  std::cout << "\nCollected parameters (including UID):\n";
  for (const auto & cg : cgs) print_params(cg);

  std::cout << "\nPress Enter to start follow (Ctrl+C to exit) ..." << std::endl;
  if (!wait_for_enter_or_sigint()) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Configure modes: master -> Current (will hold 0 A), followers -> Position; then enable all
  for (auto & cg : cgs) {
    struct can_frame tx
    {
    };
    if (cg.motor_id() == master_id) {
      cg.buildSetRunMode(yy_cybergear::RunMode::Current, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write RUN_MODE=Current [master])\n";
      }
    } else {
      cg.buildSetRunMode(yy_cybergear::RunMode::Position, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write RUN_MODE=Position [follower])\n";
      }
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
  while (g_running && rt.isRunning()) {
    const auto t = Clock::now();
    const double t_now = std::chrono::duration<double>(t - t0).count();

    // Find current master angle (from status mirror). Fallback to param mirror if needed.
    float master_angle = 0.0f;
    bool master_found = false;
    for (const auto & cg : cgs) {
      if (cg.motor_id() == master_id) {
        master_angle = cg.getStatus().angle_rad;  // fast path
        if (master_angle == 0.0f && !cg.isStatusInitialized()) {
          master_angle = cg.mechanical_position();  // fallback
        }
        master_found = true;
        break;
      }
    }
    if (!master_found) break;  // unlikely

    // Print snapshot for each motor
    for (const auto & cg : cgs) print_status(cg, t_now);

    // Keep master at 0 A current (free to be moved by hand), and command followers to master's angle
    for (auto & cg : cgs) {
      if (cg.motor_id() == master_id) {
        struct can_frame tx
        {
        };
        cg.buildSetIqReference(0.0f, tx);
        rt.post(yy_socket_can::TxRequest{ifname, tx});
        if (verbose) {
          const uint32_t id = tx.can_id & CAN_EFF_MASK;
          std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                    << " (Write IQ_REFERENCE=0A [master])\n";
        }
      }
    }

    // Command followers to master's angle
    for (auto & cg : cgs) {
      if (cg.motor_id() == master_id) continue;
      struct can_frame tx
      {
      };
      cg.buildSetPositionReference(master_angle, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write POSITION_REFERENCE)\n";
      }
    }

    std::this_thread::sleep_until(t + dt_ns);
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
