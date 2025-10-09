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

// Follow-position control with manual zero capture:
// - Lowest-ID motor is master (angle source, run in Current mode with 0 A reference).
// - User manually arranges relative poses, presses Enter to capture per-follower offsets.
// - Each follower commanded to (master_angle - captured_offset), maintaining initial configuration.
// - Uses CanRuntime + CyberGear helper, multi-motor friendly.

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

#include "exmp_helper.hpp"
#include "yy_cybergear/cybergear.hpp"
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
  CLI::App app{"exmp_06: unilateral control lowest-ID motor"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01", "0x02"};
  bool verbose = false;
  int rate_hz = 200;          // faster to track position smoothly (safety cap <= 200)
  float position_kp = 50.0f;  // example default (tune per hardware)
  float speed_kp = 1.0f;      // example default
  float speed_ki = 0.0f;      // example default

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
  app.add_option("-r,--rate", rate_hz, "Control loop rate [Hz] (max 200)")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_option("--pos-kp", position_kp, "Position proportional gain (POSITION_KP)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--spd-kp", speed_kp, "Speed loop proportional gain (SPEED_KP)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--spd-ki", speed_ki, "Speed loop integral gain (SPEED_KI)")
    ->check(CLI::NonNegativeNumber)
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
  std::cout << std::dec << " on " << ifname << ", rate=" << rate_hz << " Hz"
            << "\n  Gains: POSITION_KP=" << position_kp << ", SPEED_KP=" << speed_kp
            << ", SPEED_KI=" << speed_ki << '\n';

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
  preflight_sync(g_running, rt, ifname, cgs, preflight_params, verbose);

  // Check for errors after preflight sync
  for (const auto & cg : cgs) {
    if (check_for_errors(cg)) {
      std::cerr << "ERROR: Motor faults detected during initialization. Exiting.\n";
      rt.stop();
      return EXIT_FAILURE;
    }
  }

  std::cout << "\nCollected parameters (including UID):\n";
  for (const auto & cg : cgs) print_params(cg);

  std::cout << "\nArrange motors in desired relative configuration (followers vs master)."
               "\nPress Enter to capture zero offsets and start follow (Ctrl+C to exit) ..."
            << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Configure modes: master -> Current (will hold 0 A), followers -> Position; then set gains & enable all
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

  // Apply gains (followers only for position/speed; master usually doesn't need position gains here)
  for (auto & cg : cgs) {
    if (cg.motor_id() == master_id) continue;  // skip master for position loop gains
    // Position KP
    {
      struct can_frame tx
      {
      };
      cg.buildSetPositionKp(position_kp, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write POSITION_KP)\n";
      }
    }
    // Speed KP
    {
      struct can_frame tx
      {
      };
      cg.buildSetSpeedKp(speed_kp, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write SPEED_KP)\n";
      }
    }
    // Speed KI
    {
      struct can_frame tx
      {
      };
      cg.buildSetSpeedKi(speed_ki, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write SPEED_KI)\n";
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

  // Capture per-follower offsets now (after enabling so status is streaming)
  // offset[follower] = master_angle_at_capture - follower_angle_at_capture
  std::vector<float> follower_offsets;  // aligned with cgs indices (only used for followers)
  follower_offsets.resize(cgs.size(), 0.0f);
  float master_capture_angle = 0.0f;
  {
    // Obtain current master angle (status preferred, fallback allowed)
    for (const auto & cg : cgs) {
      if (cg.motor_id() == master_id) {
        master_capture_angle = cg.getStatus().angle_rad;
        if (master_capture_angle == 0.0f && !cg.isStatusInitialized()) {
          master_capture_angle = cg.mechanical_position();
        }
        break;
      }
    }
    for (size_t i = 0; i < cgs.size(); ++i) {
      const auto & cg = cgs[i];
      if (cg.motor_id() == master_id) continue;
      float follower_angle = cg.getStatus().angle_rad;
      if (follower_angle == 0.0f && !cg.isStatusInitialized()) {
        follower_angle = cg.mechanical_position();
      }
      follower_offsets[i] = master_capture_angle - follower_angle;
    }
  }
  std::cout << "Captured offsets relative to master:" << std::endl;
  for (size_t i = 0; i < cgs.size(); ++i) {
    const auto & cg = cgs[i];
    if (cg.motor_id() == master_id) continue;
    std::cout << "  follower 0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(cg.motor_id()) << std::dec
              << ": offset = " << follower_offsets[i] << " rad (master - follower)" << std::endl;
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

    // Check for errors in all motors and exit if any are found
    for (const auto & cg : cgs) {
      if (check_for_errors(cg)) {
        std::cerr << "ERROR: Stopping all motors due to detected faults.\n";
        g_running = false;
        break;
      }
    }
    if (!g_running) break;

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

    // Command followers to master's angle minus captured offset
    for (auto & cg : cgs) {
      if (cg.motor_id() == master_id) continue;
      struct can_frame tx
      {
      };
      // Use stored offset for this follower index
      const size_t idx = &cg - &cgs[0];
      const float target = master_angle - follower_offsets[idx];
      cg.buildSetPositionReference(target, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write POSITION_REFERENCE with offset)\n";
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
