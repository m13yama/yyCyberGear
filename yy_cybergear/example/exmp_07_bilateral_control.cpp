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

// Bilateral current control sample:
// - Two motors share motion like a virtual rigid bar using current mode.
// - Read angles/velocities from status mirror and compute differential torque.
// - Command equal and opposite motor currents with configurable stiffness / damping.

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

float current_angle_rad(const yy_cybergear::CyberGear & cg)
{
  const float angle = cg.getStatus().angle_rad;
  if (cg.isStatusInitialized()) return angle;
  return cg.mechanical_position();
}

float current_velocity_rad_s(const yy_cybergear::CyberGear & cg)
{
  const float vel = cg.getStatus().vel_rad_s;
  if (cg.isStatusInitialized()) return vel;
  return cg.mechanical_velocity();
}

}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_07: bilateral current coupling between two motors"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01", "0x02"};
  bool verbose = false;
  int rate_hz = 200;

  // Approach (move-to-zero) settings
  double approach_speed_rad_s = 1.0;     // speed limit during approach [rad/s]
  double approach_tolerance_rad = 0.03;  // consider reached when |angle-0| < tol [rad]
  int approach_hold_ms = 300;            // keep within tol for this long [ms]
  double approach_timeout_s = 10.0;      // abort if not reached within this time [s]

  double stiffness_a_per_rad = 3.0;   // virtual stiffness [A/rad]
  double damping_a_per_rad_s = 0.08;  // virtual damping [A/(rad/s)]
  double iq_limit = 8.0;              // saturate commanded currents [A]

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app
    .add_option(
      "-M,--motor-id", motor_id_strs,
      "Motor IDs for bilateral pair (repeat -M or comma-separated; decimal or 0x-prefixed hex)")
    ->delimiter(',')
    ->capture_default_str();
  app.add_option("-r,--rate", rate_hz, "Control loop rate [Hz] (max 200)")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_option("-k,--stiffness", stiffness_a_per_rad, "Virtual stiffness gain [A/rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-d,--damping", damping_a_per_rad_s, "Virtual damping gain [A/(rad/s)]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-l,--limit", iq_limit, "Current saturation limit |Iq| [A]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Verbose CAN frame prints");

  // Approach options
  app
    .add_option(
      "--approach-speed", approach_speed_rad_s, "Approach speed limit in Position mode [rad/s]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--zero-tolerance", approach_tolerance_rad, "Zero detection tolerance [rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--zero-hold-ms", approach_hold_ms, "Zero detection hold duration [ms]")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_option("--approach-timeout", approach_timeout_s, "Approach timeout [s]")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();

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
  if (motors.size() != 2) {
    std::cerr << "Provide exactly two motor IDs for bilateral control.\n";
    return EXIT_FAILURE;
  }
  if (motors[0] == motors[1]) {
    std::cerr << "Motor IDs must be distinct.\n";
    return EXIT_FAILURE;
  }

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

  std::cout << "Bilateral current coupling motors [0x" << std::uppercase << std::hex
            << std::setw(2) << std::setfill('0') << static_cast<unsigned>(motors[0]) << ", 0x"
            << std::setw(2) << static_cast<unsigned>(motors[1]) << std::dec << "] on " << ifname
            << "\n  approach: speed=" << approach_speed_rad_s
            << " rad/s, tol=" << approach_tolerance_rad << " rad, hold=" << approach_hold_ms
            << " ms, timeout=" << approach_timeout_s << " s"
            << "\n  current mode: stiffness=" << stiffness_a_per_rad << " A/rad"
            << ", damping=" << damping_a_per_rad_s << " A/(rad/s)"
            << ", limit=" << iq_limit << " A, rate=" << rate_hz << " Hz" << '\n';

  const std::vector<uint16_t> preflight_params = {
    yy_cybergear::RUN_MODE,
    yy_cybergear::SPEED_LIMIT,
    yy_cybergear::CURRENT_LIMIT,
    yy_cybergear::TORQUE_LIMIT,
    yy_cybergear::MECHANICAL_POSITION,
    yy_cybergear::MECHANICAL_VELOCITY,
  };
  preflight_sync(g_running, rt, ifname, cgs, preflight_params, verbose);

  for (const auto & cg : cgs) {
    if (check_for_errors(cg)) {
      std::cerr << "ERROR: Motor faults detected during initialization. Exiting.\n";
      rt.stop();
      return EXIT_FAILURE;
    }
  }

  std::cout << "\nCollected parameters (including UID):\n";
  for (const auto & cg : cgs) print_params(cg);

  std::cout << "\nPress Enter to start approach (Ctrl+C to exit) ..." << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Approach phase: move both motors to 0 rad in Position mode slowly
  std::cout << "Approach: switching to Position mode and moving to 0 rad ..." << std::endl;
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
    cg.buildSetSpeedLimit(static_cast<float>(approach_speed_rad_s), tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write SPEED_LIMIT for approach)\n";
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

  // Command zero position until within tolerance for hold duration
  {
    const auto approach_start = Clock::now();
    std::optional<Clock::time_point> hold_start;
    const auto hold_dur = std::chrono::milliseconds(approach_hold_ms);
    const auto timeout_dur = std::chrono::duration<double>(approach_timeout_s);
    const auto dt_approach =
      std::chrono::nanoseconds{static_cast<long long>(1e9 / std::max(1, rate_hz))};
    while (g_running && rt.isRunning()) {
      const auto t = Clock::now();
      const double t_now = std::chrono::duration<double>(t - t0).count();

      for (const auto & cg : cgs) {
        if (check_for_errors(cg)) {
          std::cerr << "ERROR: Stopping due to detected faults during approach." << std::endl;
          g_running = false;
          break;
        }
      }
      if (!g_running) break;

      // Send position references to 0.0 rad
      for (auto & cg : cgs) {
        struct can_frame tx
        {
        };
        cg.buildSetPositionReference(0.0f, tx);
        rt.post(yy_socket_can::TxRequest{ifname, tx});
        if (verbose) {
          const uint32_t id = tx.can_id & CAN_EFF_MASK;
          std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                    << " (Write POSITION_REFERENCE=0)\n";
        }
      }

      const float a0 = current_angle_rad(cgs[0]);
      const float a1 = current_angle_rad(cgs[1]);
      const float e0 = std::abs(a0);
      const float e1 = std::abs(a1);
      const bool in_tol = (e0 <= static_cast<float>(approach_tolerance_rad)) &&
                          (e1 <= static_cast<float>(approach_tolerance_rad));

      for (const auto & cg : cgs) print_status(cg, t_now);

      if (in_tol) {
        if (!hold_start.has_value()) hold_start = t;
        if ((t - *hold_start) >= hold_dur) {
          std::cout << "Approach: both motors within tolerance for hold duration." << std::endl;
          break;
        }
      } else {
        hold_start.reset();
      }

      if ((t - approach_start) > timeout_dur) {
        std::cerr << "Approach timeout after " << approach_timeout_s << " s. Aborting."
                  << std::endl;
        rt.stop();
        return EXIT_FAILURE;
      }

      std::this_thread::sleep_until(t + dt_approach);
    }

    if (!g_running || !rt.isRunning()) {
      rt.stop();
      return EXIT_SUCCESS;
    }
  }

  // Stop motors once after approach before changing mode
  std::cout << "Approach complete: stopping motors before mode switch ..." << std::endl;
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
  // brief settle
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Switch to Current mode to start bilateral current control
  for (auto & cg : cgs) {
    struct can_frame tx
    {
    };
    cg.buildSetRunMode(yy_cybergear::RunMode::Current, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write RUN_MODE=Current)\n";
    }
  }

  // Wait for user confirmation to start teleoperation
  std::cout << "\nPress Enter to start teleoperation (Ctrl+C to exit) ..." << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  // Re-enable motors in Current mode before entering control loop
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
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  float neutral_offset = 0.0f;
  bool neutral_captured = false;

  const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  while (g_running && rt.isRunning()) {
    const auto t = Clock::now();
    const double t_now = std::chrono::duration<double>(t - t0).count();

    for (const auto & cg : cgs) {
      if (check_for_errors(cg)) {
        std::cerr << "ERROR: Stopping due to detected faults.\n";
        g_running = false;
        break;
      }
    }
    if (!g_running) break;

    const float angle_a = current_angle_rad(cgs[0]);
    const float angle_b = current_angle_rad(cgs[1]);
    const float vel_a = current_velocity_rad_s(cgs[0]);
    const float vel_b = current_velocity_rad_s(cgs[1]);

    if (!neutral_captured) {
      neutral_offset = angle_a - angle_b;
      neutral_captured = true;
    }

    const float angle_error = (angle_a - angle_b) - neutral_offset;
    const float vel_error = vel_a - vel_b;

    const double torque_cmd = stiffness_a_per_rad * static_cast<double>(angle_error) +
                              damping_a_per_rad_s * static_cast<double>(vel_error);

    const double sat = std::max(0.0, iq_limit);
    const double iq_a_cmd = std::clamp(-torque_cmd, -sat, sat);
    const double iq_b_cmd = std::clamp(+torque_cmd, -sat, sat);

    for (const auto & cg : cgs) print_status(cg, t_now);

    struct can_frame tx_a
    {
    };
    cgs[0].buildSetIqReference(static_cast<float>(iq_a_cmd), tx_a);
    rt.post(yy_socket_can::TxRequest{ifname, tx_a});
    if (verbose) {
      const uint32_t id = tx_a.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write IQ_REFERENCE motor[0])\n";
    }

    struct can_frame tx_b
    {
    };
    cgs[1].buildSetIqReference(static_cast<float>(iq_b_cmd), tx_b);
    rt.post(yy_socket_can::TxRequest{ifname, tx_b});
    if (verbose) {
      const uint32_t id = tx_b.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write IQ_REFERENCE motor[1])\n";
    }

    std::this_thread::sleep_until(t + dt_ns);
  }

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
