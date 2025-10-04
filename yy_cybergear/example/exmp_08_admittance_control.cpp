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

// Admittance control sample (single motor):
// - Capture current pose as neutral before enabling the actuator.
// - Read joint torque from status frames and integrate a virtual mass-spring-damper model.
// - Command Position mode references derived from the admittance state.
// - Enforces displacement/velocity limits and monitors for faults each loop iteration.

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
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
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
  if (cg.isStatusInitialized()) return cg.getStatus().angle_rad;
  return cg.mechanical_position();
}

}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_08: single-motor admittance control in Position mode"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01"};
  bool verbose = false;
  int rate_hz = 200;

  double virtual_mass = 0.05;                 // virtual inertia [N·m·s^2/rad]
  double virtual_damping = 0.4;               // virtual damping [N·m·s/rad]
  double virtual_stiffness = 1.5;             // virtual stiffness [N·m/rad]
  double displacement_limit_rad = 0.6;        // max admittance displacement [rad]
  double velocity_limit_rad_s = 4.0;          // max admittance velocity [rad/s]
  double torque_bias_nm = 0.0;                // subtract constant torque bias [N·m]
  double torque_deadband_nm = 0.02;           // ignore small torques [N·m]
  double torque_alpha = 1.0;                  // EMA coefficient (1.0 = no filtering)
  double speed_limit_rad_s = 12.0;            // firmware Position-mode speed limit [rad/s]

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app
    .add_option(
      "-M,--motor-id", motor_id_strs,
      "Motor ID (single motor; decimal or 0x-prefixed hex)")
    ->delimiter(',')
    ->capture_default_str();
  app.add_option("-r,--rate", rate_hz, "Control loop rate [Hz] (max 200)")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Verbose CAN frame prints");

  app.add_option("--mass", virtual_mass, "Virtual inertia term M [N·m·s^2/rad]")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_option("--damping", virtual_damping, "Virtual damping term D [N·m·s/rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--stiffness", virtual_stiffness, "Virtual stiffness term K [N·m/rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--disp-limit", displacement_limit_rad, "Clamp admittance displacement |theta| [rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--vel-limit", velocity_limit_rad_s, "Clamp admittance velocity |theta_dot| [rad/s]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--torque-bias", torque_bias_nm, "Torque bias to subtract [N·m]")
    ->capture_default_str();
  app.add_option("--torque-deadband", torque_deadband_nm, "Deadband before admitting torque [N·m]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--torque-alpha", torque_alpha,
    "EMA coefficient for torque filtering (1=no filter, 0<alpha<=1)")
    ->check(CLI::Range(0.0, 1.0))
    ->capture_default_str();
  app.add_option("--speed-limit", speed_limit_rad_s,
    "Firmware speed limit applied in Position mode [rad/s]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  if (rate_hz > 200) {
    std::cerr << "Safety: capping control loop rate to 200 Hz (requested " << rate_hz << ")\n";
    rate_hz = 200;
  }
  if (virtual_mass <= std::numeric_limits<double>::epsilon()) {
    std::cerr << "Virtual mass must be positive." << std::endl;
    return EXIT_FAILURE;
  }

  unsigned long host_ul = 0x00UL;
  try {
    host_ul = std::stoul(host_id_str, nullptr, 0);
  } catch (const std::exception & e) {
    std::cerr << "Invalid host ID: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
  if (host_ul > 0xFFul) {
    std::cerr << "Host ID must be 0..255 (got " << host_ul << ")\n";
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
  if (motors.size() != 1) {
    std::cerr << "Admittance demo expects exactly one motor ID." << std::endl;
    return EXIT_FAILURE;
  }
  const uint8_t motor_id = motors.front();

  std::signal(SIGINT, handle_sigint);
  std::signal(SIGTERM, handle_sigint);

  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  const auto t0 = Clock::now();

  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.emplace_back(host, motor_id);

  register_can_handler(rt, cgs, verbose);
  rt.start();

  std::cout << "Admittance control motor 0x" << std::uppercase << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<unsigned>(motor_id) << std::dec << " on " << ifname
            << "\n  M=" << virtual_mass << " N·m·s^2/rad, D=" << virtual_damping
            << " N·m·s/rad, K=" << virtual_stiffness << " N·m/rad"
            << "\n  disp limit=" << displacement_limit_rad << " rad, vel limit=" << velocity_limit_rad_s
            << " rad/s, torque deadband=" << torque_deadband_nm << " N·m" << std::endl;

  const std::vector<uint16_t> preflight_params = {
    yy_cybergear::RUN_MODE,
    yy_cybergear::POSITION_REFERENCE,
    yy_cybergear::SPEED_LIMIT,
    yy_cybergear::MECHANICAL_POSITION,
    yy_cybergear::MECHANICAL_VELOCITY,
  };
  preflight_sync(g_running, rt, ifname, cgs, preflight_params, verbose);

  if (!g_running || !rt.isRunning()) {
    rt.stop();
    return EXIT_FAILURE;
  }

  for (const auto & motor : cgs) {
    if (check_for_errors(motor)) {
      std::cerr << "ERROR: Motor fault detected during initialization." << std::endl;
      rt.stop();
      return EXIT_FAILURE;
    }
  }

  std::cout << "\nCollected parameters and UID:\n";
  for (const auto & motor : cgs) print_params(motor);

  std::cout << "\nAlign the motor to the desired neutral pose, then press Enter to capture neutral." << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  if (!cgs.front().isStatusInitialized()) {
    std::cerr << "Waiting for status frames to provide torque feedback..." << std::endl;
    while (g_running && rt.isRunning() && !cgs.front().isStatusInitialized()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!cgs.front().isStatusInitialized()) {
      std::cerr << "No status frames received; cannot perform admittance control." << std::endl;
      rt.stop();
      return EXIT_FAILURE;
    }
  }

  const float neutral_angle = current_angle_rad(cgs.front());
  std::cout << "Captured neutral angle: " << neutral_angle << " rad" << std::endl;

  {
    struct can_frame tx
    {
    };
    cgs.front().buildStop(tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Stop)\n";
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (speed_limit_rad_s > 0.0) {
    struct can_frame tx
    {
    };
    cgs.front().buildSetSpeedLimit(static_cast<float>(speed_limit_rad_s), tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write SPEED_LIMIT)\n";
    }
  }

  {
    struct can_frame tx
    {
    };
    cgs.front().buildSetRunMode(yy_cybergear::RunMode::Position, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write RUN_MODE=Position)\n";
    }
  }

  {
    struct can_frame tx
    {
    };
    cgs.front().buildEnable(tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Enable)\n";
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  {
    struct can_frame tx
    {
    };
    cgs.front().buildSetPositionReference(neutral_angle, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write POSITION_REFERENCE neutral)\n";
    }
  }

  std::cout << "Starting admittance loop. Press Ctrl+C to exit." << std::endl;

  const std::chrono::nanoseconds dt_nominal{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  auto last_step = Clock::now();
  double admittance_pos = 0.0;
  double admittance_vel = 0.0;
  double torque_filtered_nm = 0.0;
  bool torque_initialized = false;

  while (g_running && rt.isRunning()) {
    const auto t = Clock::now();
    double dt = std::chrono::duration<double>(t - last_step).count();
    last_step = t;
    dt = std::clamp(dt, 0.0, 0.1);  // constrain integration step
    if (dt <= 0.0) dt = 1.0 / std::max(1, rate_hz);

    if (check_for_errors(cgs.front())) {
      std::cerr << "ERROR: Stopping due to detected fault." << std::endl;
      g_running = false;
      break;
    }

    if (!cgs.front().isStatusInitialized()) {
      std::this_thread::sleep_until(t + dt_nominal);
      continue;
    }

    const auto status = cgs.front().getStatus();
    double torque_measured_nm = static_cast<double>(status.torque_Nm) - torque_bias_nm;
    if (std::abs(torque_measured_nm) < torque_deadband_nm) torque_measured_nm = 0.0;

    if (!torque_initialized) {
      torque_filtered_nm = torque_measured_nm;
      torque_initialized = true;
    } else {
      torque_filtered_nm = torque_alpha * torque_measured_nm + (1.0 - torque_alpha) * torque_filtered_nm;
    }

    double accel = (torque_filtered_nm - virtual_damping * admittance_vel -
                    virtual_stiffness * admittance_pos) /
                   virtual_mass;
    if (!std::isfinite(accel)) accel = 0.0;

    admittance_vel += accel * dt;
    if (velocity_limit_rad_s > 0.0) {
      admittance_vel = std::clamp(admittance_vel, -velocity_limit_rad_s, velocity_limit_rad_s);
    }

    admittance_pos += admittance_vel * dt;
    if (displacement_limit_rad > 0.0) {
      admittance_pos = std::clamp(admittance_pos, -displacement_limit_rad, displacement_limit_rad);
    }

    const double desired_angle = static_cast<double>(neutral_angle) + admittance_pos;

    print_status(cgs.front(), std::chrono::duration<double>(t - t0).count());

    struct can_frame tx
    {
    };
    cgs.front().buildSetPositionReference(static_cast<float>(desired_angle), tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write POSITION_REFERENCE admittance)\n";
    }

    std::this_thread::sleep_until(t + dt_nominal);
  }

  {
    struct can_frame tx
    {
    };
    cgs.front().buildStop(tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Stop)\n";
    }
  }

  rt.stop();
  return g_running ? EXIT_SUCCESS : EXIT_FAILURE;
}
