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
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "yy_cybergear/cybergear.hpp"

using yy_cybergear::CyberGear;

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }
}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"CyberGear: zero torque (Iq=0) in current mode (simple)"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x01"};
  std::string motor_id_str{"0x01"};
  bool verbose = false;

  double limit_tau = 6.0;   // torque limit [Nm]
  double limit_cur = 10.0;  // current limit [A]
  int rate_hz = 100;        // command rate [Hz]
  double duration = 0.0;    // 0 => run until Ctrl+C

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-M,--motor-id", motor_id_str, "Motor ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("--limit-tau", limit_tau, "Torque limit [Nm]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--limit-cur", limit_cur, "Current limit [A]")
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

    // Safe setup: clear faults, set current mode, limits, enable
    (void)dev.clearFaults();
    (void)dev.setRunMode(CyberGear::RunMode::Current).ok();
    (void)dev.setTorqueLimit(static_cast<float>(limit_tau)).ok();
    (void)dev.setCurrentLimit(static_cast<float>(limit_cur)).ok();
    (void)dev.enableMotor();

    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / rate_hz)};

    std::cout << "Zero torque on motor 0x" << std::uppercase << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<unsigned>(motor) << std::dec
              << ": IqRef=0.0 A, rate=" << rate_hz << " Hz, limit_tau=" << limit_tau
              << " Nm, limit_cur=" << limit_cur << " A" << '\n';

    while (g_running) {
      const auto start = clock::now();
      const auto deadline = start + dt_ns;
      const double t = std::chrono::duration<double>(start - t0).count();
      if (duration > 0.0 && t >= duration) break;

      {
        auto r = dev.setIqReference(0.0f);
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
          std::cerr << "Failed to set Iq reference: " << yy_cybergear::to_string(*r.error())
                    << ". Stopping motor and exiting." << '\n';
          (void)dev.stopMotor();
          return EXIT_FAILURE;
        }
      }

      // Overrun detection
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
