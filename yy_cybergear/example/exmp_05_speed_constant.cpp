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
#include "yy_cybergear/protocol_types.hpp"

using yy_cybergear::CyberGear;

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }
}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"CyberGear: constant speed control (simple)"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x01"};
  std::string motor_id_str{"0x01"};
  bool verbose = false;

  // Control parameters
  double speed_rad_s = 2.0;  // desired constant speed [rad/s] (can be negative)
  // Loop timing (fixed internal rate)
  constexpr int kRateHz = 100;

  app.add_option("-i,--interface", ifname, "CAN interface name (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-M,--motor-id", motor_id_str, "Motor ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-s,--speed", speed_rad_s, "Target speed [rad/s] (negative allowed)")
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

    // Basic safe setup: clear faults, set speed mode, enable motor
    (void)dev.clearFaults();
    (void)dev.setRunMode(CyberGear::RunMode::Speed).ok();
    (void)dev.enableMotor();

    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / kRateHz)};

    std::cout << "Constant speed control on motor 0x" << std::uppercase << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<unsigned>(motor) << std::dec
              << ": speed=" << speed_rad_s << " rad/s" << '\n';

    while (g_running) {
      const auto start = clock::now();
      const auto deadline = start + dt_ns;
      const double t = std::chrono::duration<double>(start - t0).count();

      {
        auto r = dev.setSpeedReference(static_cast<float>(speed_rad_s));
        if (r.ok()) {
          const auto & st = *r.value();
          const auto mode_str = yy_cybergear::mode_to_string(st.mode);
          const auto faults_vec = yy_cybergear::fault_bits_to_string(st.fault_bits);
          std::string faults_str = faults_vec.empty() ? std::string("none") : faults_vec.front();
          for (size_t i = 1; i < faults_vec.size(); ++i) faults_str += ", " + faults_vec[i];
          std::cout << std::fixed << std::setprecision(3) << " t=" << t << "s"
                    << " ang=" << st.angle_rad << "rad"
                    << " vel=" << st.vel_rad_s << "rad/s"
                    << " tau=" << st.torque_Nm << "Nm"
                    << " T=" << st.temperature_c << "C"
                    << " mode=" << mode_str << " faults=" << faults_str << " mid=0x"
                    << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<unsigned>(st.motor_can_id) << std::dec << '\n';

          // If any fault bits are set, stop immediately and exit with failure
          if (st.fault_bits != 0) {
            std::cerr << "Fault detected: bits=0b" << std::uppercase << std::hex << std::setw(2)
                      << std::setfill('0') << static_cast<unsigned>(st.fault_bits) << std::dec
                      << ". Stopping motor and exiting." << '\n';
            (void)dev.stopMotor();
            return EXIT_FAILURE;
          }
        } else {
          std::cerr << "Failed to set speed reference: " << yy_cybergear::to_string(*r.error())
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
        const auto period_us =
          std::chrono::duration_cast<std::chrono::microseconds>(dt_ns).count();
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
