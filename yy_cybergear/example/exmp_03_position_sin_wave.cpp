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

#include <CLI/CLI.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/protocol_types.hpp"

using yy_cybergear::CyberGear;

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"CyberGear: sine-wave position control using type-1 op commands"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x01"};
  std::string motor_id_str{"0x01"};
  bool verbose = false;

  // Sine parameters
  double amp_rad = 0.5;     // amplitude [rad]
  double freq_hz = 0.5;     // frequency [Hz]
  double center_rad = 0.0;  // center offset [rad]
  double kp = 50.0;         // position gain
  double kd = 1.0;          // velocity gain
  double limit_tau = 6.0;   // torque limit [Nm]
  double limit_spd = 10.0;  // speed limit [rad/s]
  int rate_hz = 100;        // command rate
  double duration = 0.0;    // 0 => run until Ctrl+C

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-M,--motor-id", motor_id_str, "Motor ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-A,--amp", amp_rad, "Sine amplitude [rad]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-f,--freq", freq_hz, "Sine frequency [Hz]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-c,--center", center_rad, "Center position [rad]")->capture_default_str();
  app.add_option("--kp", kp, "Position gain (0..500)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--kd", kd, "Velocity gain (0..5)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--limit-tau", limit_tau, "Torque limit [Nm]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--limit-spd", limit_spd, "Speed limit [rad/s]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("-r,--rate", rate_hz, "Command rate [Hz]")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();
  app.add_option("-d,--duration", duration, "Run duration [s] (0 = infinite)")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Verbose CAN frame prints");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  unsigned long host_ul = 0x01UL, motor_ul = 0x01UL;
  try {
    host_ul = std::stoul(host_id_str, nullptr, 0);
    motor_ul = std::stoul(motor_id_str, nullptr, 0);
  } catch (const std::exception & e) {
    std::cerr << "Invalid ID: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
  if (host_ul > 0xFFul || motor_ul > 0xFFul) {
    std::cerr << "IDs must be 0..255 (got host=" << host_ul << ", motor=" << motor_ul << ")\n";
    return EXIT_FAILURE;
  }
  const uint8_t host = static_cast<uint8_t>(host_ul & 0xFFu);
  const uint8_t motor = static_cast<uint8_t>(motor_ul & 0xFFu);

  std::signal(SIGINT, handle_sigint);

  try {
    CyberGear dev(ifname, host, motor, verbose);
    dev.open();

    // Basic safe setup: clear faults, operation mode, limits, enable motor
    (void)dev.clearFaults();
    (void)dev.setRunMode(CyberGear::RunMode::Operation).ok();
    (void)dev.setTorqueLimit(static_cast<float>(limit_tau)).ok();
    (void)dev.setSpeedLimit(static_cast<float>(limit_spd)).ok();
    (void)dev.enableMotor();

    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / rate_hz)};

    std::cout << "Sine position control on motor 0x" << std::uppercase << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<unsigned>(motor) << std::dec
              << ": amp=" << amp_rad << " rad, freq=" << freq_hz << " Hz, center=" << center_rad
              << " rad, kp=" << kp << ", kd=" << kd << ", rate=" << rate_hz << " Hz" << '\n';

    while (g_running) {
      const auto start = clock::now();
      const auto deadline = start + dt_ns;
      const double t = std::chrono::duration<double>(start - t0).count();
      if (duration > 0.0 && t >= duration) break;

      // Compute reference
      // Use local constant for pi to avoid non-standard M_PI macro dependency
      constexpr double kPi = 3.14159265358979323846;
      const double omega = 2.0 * kPi * freq_hz;
      const double pos = center_rad + amp_rad * std::sin(omega * t);
      const double vel = omega * amp_rad * std::cos(omega * t);

      yy_cybergear::OpCommand cmd{};
      cmd.pos_rad = static_cast<float>(pos);
      cmd.vel_rad_s = static_cast<float>(vel);
      cmd.kp = static_cast<float>(kp);
      cmd.kd = static_cast<float>(kd);
      cmd.torque_Nm = 0.0f;

      // Send op command and print returned status (type-2)
      {
        const auto now = clock::now();
        const auto remaining = deadline - now;
        int timeout_ms = std::max(
          0, static_cast<int>(
               std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count()));
        if (timeout_ms <= 0) timeout_ms = 1;

        auto r = dev.sendOperationCommand(cmd, timeout_ms);
        if (r.ok()) {
          const auto & st = *r.value();
          std::cout << std::fixed << std::setprecision(3) << " t=" << t << "s"
                    << " ang=" << st.angle_rad << "rad"
                    << " vel=" << st.vel_rad_s << "rad/s"
                    << " tau=" << st.torque_Nm << "Nm"
                    << " T=" << st.temperature_c << "C"
                    << " mode=" << static_cast<unsigned>(st.mode) << " faults=0b" << std::uppercase
                    << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<unsigned>(st.fault_bits) << std::dec << " mid=0x"
                    << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<unsigned>(st.motor_can_id) << std::dec << '\n';
        } else {
          std::cerr << "sendOperationCommand failed: " << yy_cybergear::to_string(*r.error())
                    << ". Stopping motor and exiting." << '\n';
          (void)dev.stopMotor();
          return EXIT_FAILURE;
        }
      }

      // Overrun detection: if work time exceeded the period, abort safely
      const auto end = clock::now();
      if (end > deadline) {
        const auto over = end - deadline;
        const auto over_us = std::chrono::duration_cast<std::chrono::microseconds>(over).count();
        const auto period_us = std::chrono::duration_cast<std::chrono::microseconds>(dt_ns).count();
        std::cerr << "Control loop overrun: " << over_us << " us past the period (" << period_us
                  << " us). Aborting." << '\n';
        (void)dev.stopMotor();
        return EXIT_FAILURE;
      }

      std::this_thread::sleep_until(deadline);
    }

    // Stop motor safely
    (void)dev.stopMotor();
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
