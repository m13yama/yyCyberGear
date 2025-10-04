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

// Four-channel impedance teleoperation sample:
// - Treat the first motor ID as "master" (operator side) and the second as "slave" (remote side).
// - Estimate velocities and accelerations from status feedback to build a virtual reaction force.
// - Apply a 4-channel controller (position + force) and command symmetric current references.

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

float current_torque_nm(const yy_cybergear::CyberGear & cg)
{
  if (cg.isStatusInitialized()) return cg.getStatus().torque_Nm;
  return 0.0f;
}

}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_07: four-channel impedance control (master/slave current mode)"};

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

  // Four-channel control gains (default tuned for gentle response)
  double kx_gain_a_per_rad = 8.0;   // position coupling gain (Kx)
  double kf_gain_a_per_unit = 1.5;  // force coupling gain (Kf)
  double mv_virtual = 0.04;         // virtual mass term (Mv)
  double bv_virtual = 0.35;         // virtual damping term (Bv)
  double kv_virtual = 1.5;          // virtual stiffness term (Kv)
  double force_scale = 1.0;         // scale torque_Nm -> force units for Fs
  double force_bias = 0.0;          // bias added to measured force after scaling
  double force_alpha = 0.2;         // EWMA for measured force (0..1)
  double accel_alpha = 0.3;         // EWMA for acceleration estimate (0..1)
  double iq_limit = 20.0;           // saturate commanded currents [A]

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app
    .add_option(
      "-M,--motor-id", motor_id_strs,
      "Motor IDs for master/slave pair (repeat -M or comma-separated; decimal or 0x-prefixed hex)")
    ->delimiter(',')
    ->capture_default_str();
  app.add_option("-r,--rate", rate_hz, "Control loop rate [Hz] (max 200)")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_option("--kx", kx_gain_a_per_rad, "Position gain Kx [A/rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--kf", kf_gain_a_per_unit, "Force gain Kf [A/unit]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--mv", mv_virtual, "Virtual mass Mv (force / rad/s^2)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--bv", bv_virtual, "Virtual damping Bv (force / rad/s)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--kv", kv_virtual, "Virtual stiffness Kv (force / rad)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--force-scale", force_scale, "Scale applied to slave torque_Nm before using as Fs")
    ->capture_default_str();
  app.add_option("--force-bias", force_bias, "Bias added to measured force Fs after scaling")
    ->capture_default_str();
  app
    .add_option("--force-alpha", force_alpha, "EWMA coefficient for force measurement (0..1)")
    ->check(CLI::Range(0.0, 1.0))
    ->capture_default_str();
  app
    .add_option("--accel-alpha", accel_alpha, "EWMA coefficient for acceleration estimate (0..1)")
    ->check(CLI::Range(0.0, 1.0))
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

  force_alpha = std::clamp(force_alpha, 0.0, 1.0);
  accel_alpha = std::clamp(accel_alpha, 0.0, 1.0);

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
    std::cerr << "Provide exactly two motor IDs for four-channel control (master + slave).\n";
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

  std::cout << "Four-channel impedance control master=0x" << std::uppercase << std::hex
            << std::setw(2) << std::setfill('0') << static_cast<unsigned>(motors[0])
            << ", slave=0x" << std::setw(2) << static_cast<unsigned>(motors[1]) << std::dec
            << std::setfill(' ') << " on " << ifname << '\n'
            << "  approach: speed=" << approach_speed_rad_s << " rad/s, tol="
            << approach_tolerance_rad << " rad, hold=" << approach_hold_ms << " ms, timeout="
            << approach_timeout_s << " s\n"
            << "  Kx=" << kx_gain_a_per_rad << " A/rad, Kf=" << kf_gain_a_per_unit
            << " A/unit, |Iq| limit=" << iq_limit << " A, rate=" << rate_hz << " Hz\n"
            << "  Virtual mass Mv=" << mv_virtual << ", Bv=" << bv_virtual << ", Kv="
            << kv_virtual << ", force_scale=" << force_scale << ", force_alpha=" << force_alpha
            << ", accel_alpha=" << accel_alpha << '\n';

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

  struct DerivedState
  {
    float prev_vel_rad_s{0.0f};
    float accel_rad_s2{0.0f};
    bool has_prev{false};
  };

  DerivedState master_state;
  DerivedState slave_state;

  double filtered_force = 0.0;
  bool force_initialized = false;

  float neutral_offset = 0.0f;
  bool neutral_captured = false;

  const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  auto last_loop_time = Clock::now();
  bool first_loop_iteration = true;

  while (g_running && rt.isRunning()) {
    const auto t = Clock::now();
    double dt = first_loop_iteration
                  ? (1.0 / static_cast<double>(std::max(1, rate_hz)))
                  : std::chrono::duration<double>(t - last_loop_time).count();
    last_loop_time = t;
    first_loop_iteration = false;
    if (dt <= 1e-6) dt = 1.0 / static_cast<double>(std::max(1, rate_hz));

    const double t_now = std::chrono::duration<double>(t - t0).count();

    for (const auto & cg : cgs) {
      if (check_for_errors(cg)) {
        std::cerr << "ERROR: Stopping due to detected faults.\n";
        g_running = false;
        break;
      }
    }
    if (!g_running) break;

    const float xm = current_angle_rad(cgs[0]);
    const float xs = current_angle_rad(cgs[1]);
    const float vm = current_velocity_rad_s(cgs[0]);
    const float vs = current_velocity_rad_s(cgs[1]);

    if (!neutral_captured) {
      neutral_offset = xs - xm;
      neutral_captured = true;
    }

    auto update_accel = [dt, accel_alpha](DerivedState & state, float vel) {
      float velocity = std::isfinite(vel) ? vel : 0.0f;
      if (!state.has_prev) {
        state.prev_vel_rad_s = velocity;
        state.accel_rad_s2 = 0.0f;
        state.has_prev = true;
        return state.accel_rad_s2;
      }

      double raw = 0.0;
      if (dt > 1e-6) {
        raw = (static_cast<double>(velocity) - static_cast<double>(state.prev_vel_rad_s)) / dt;
      }
      if (!std::isfinite(raw)) raw = 0.0;

      state.accel_rad_s2 = static_cast<float>(
        accel_alpha * raw + (1.0 - accel_alpha) * static_cast<double>(state.accel_rad_s2));
      state.prev_vel_rad_s = velocity;
      state.has_prev = true;
      return state.accel_rad_s2;
    };

    const float am = update_accel(master_state, vm);
    const float as = update_accel(slave_state, vs);

    const float torque_slave_nm = current_torque_nm(cgs[1]);
    double measured_force = static_cast<double>(torque_slave_nm) * force_scale + force_bias;
    if (!std::isfinite(measured_force)) measured_force = 0.0;

    if (!force_initialized) {
      filtered_force = measured_force;
      force_initialized = true;
    } else {
      filtered_force = force_alpha * measured_force + (1.0 - force_alpha) * filtered_force;
    }

    const double pos_diff = static_cast<double>((xs - xm) - neutral_offset);
    const double vel_diff = static_cast<double>(vs - vm);
    const double accel_diff = static_cast<double>(as - am);

    const double fm_star = mv_virtual * accel_diff + bv_virtual * vel_diff + kv_virtual * pos_diff;
    const double combined_force = filtered_force + fm_star;

    double um = kx_gain_a_per_rad * pos_diff + kf_gain_a_per_unit * combined_force;
    double us = -kx_gain_a_per_rad * pos_diff - kf_gain_a_per_unit * combined_force;

    const double sat = std::max(0.0, iq_limit);
    um = std::clamp(um, -sat, sat);
    us = std::clamp(us, -sat, sat);

    for (const auto & cg : cgs) print_status(cg, t_now);

    if (verbose) {
      std::cout << std::fixed << std::setprecision(4)
                << "  pos_diff=" << pos_diff << " rad, vel_diff=" << vel_diff
                << " rad/s, accel_diff=" << accel_diff << " rad/s^2\n"
                << "  Fs=" << filtered_force << ", Fm*=" << fm_star << ", um=" << um << ", us="
                << us << '\n';
      std::cout.unsetf(std::ios::floatfield);
      std::cout << std::setprecision(6);
    }

    struct can_frame tx_master
    {
    };
    cgs[0].buildSetIqReference(static_cast<float>(um), tx_master);
    rt.post(yy_socket_can::TxRequest{ifname, tx_master});
    if (verbose) {
      const uint32_t id = tx_master.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write IQ_REFERENCE master)\n";
    }

    struct can_frame tx_slave
    {
    };
    cgs[1].buildSetIqReference(static_cast<float>(us), tx_slave);
    rt.post(yy_socket_can::TxRequest{ifname, tx_slave});
    if (verbose) {
      const uint32_t id = tx_slave.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write IQ_REFERENCE slave)\n";
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
