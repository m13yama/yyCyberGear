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

// Implement exmp_05 (constant speed) using CanRuntime framework (no CyberGear class).
// - Periodically set RunMode=Speed and write SPEED_REFERENCE parameter
// - Enable motor once at start, stop on exit
// - Receive and print type-2 status via dispatcher

#include <linux/can.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "yy_cybergear/data_frame_handler.hpp"
#include "yy_cybergear/protocol_types.hpp"
#include "yy_socket_can/can_runtime.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

void print_status(const yy_cybergear::Status & st, double t_sec)
{
  std::cout << std::fixed << std::setprecision(3) << " t=" << t_sec << "s"
            << " ang=" << st.angle_rad << "rad"
            << " vel=" << st.vel_rad_s << "rad/s"
            << " tau=" << st.torque_Nm << "Nm"
            << " T=" << st.temperature_c << "C"
            << " mode=" << static_cast<unsigned>(st.mode) << " faults=0b" << std::uppercase
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(st.fault_bits) << std::dec << " mid=0x" << std::uppercase
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(st.motor_can_id) << std::dec << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_07: constant speed control using CanRuntime"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::vector<std::string> motor_id_strs{"0x01"};
  bool verbose = false;

  double speed_rad_s = 2.0;  // target speed [rad/s]
  int rate_hz = 100;         // control loop rate [Hz]

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
  app.add_option("-s,--speed", speed_rad_s, "Target speed [rad/s] (negative allowed)")
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
      const uint8_t mid = static_cast<uint8_t>(v & 0xFFu);
      if (std::find(motors.begin(), motors.end(), mid) == motors.end()) {
        motors.push_back(mid);
      }
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

  // Setup CanRuntime
  yy_socket_can::CanRuntime rt;
  rt.set_warning_logger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.add_channel(ifname);

  using yy_cybergear::Status;
  using yy_cybergear::data_frame_handler::parseStatus;
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();

  // Register status handlers for each motor (type 2 range)
  for (uint8_t motor : motors) {
    const uint32_t min_id = (2u << 24) | (static_cast<uint32_t>(motor) << 8);
    const uint32_t max_id =
      (2u << 24) | (3u << 22) | (0x3Fu << 16) | (static_cast<uint32_t>(motor) << 8) | 0xFFu;
    rt.register_handler(min_id, max_id, [verbose, t0](const struct can_frame & f) {
      Status st{};
      if (!parseStatus(f, st)) return;
      const double t = std::chrono::duration<double>(clock::now() - t0).count();
      if (verbose) {
        const uint32_t id = f.can_id & CAN_EFF_MASK;
        std::cout << "RX 0x" << std::hex << std::uppercase << id << std::dec
                  << " dlc=" << int(f.can_dlc) << "\n";
      }
      print_status(st, t);
    });
  }

  // Start runtime
  rt.start();

  // Write RunMode (0x7005) = Speed(2) for each motor
  for (uint8_t motor : motors) {
    std::array<uint8_t, 4> data{static_cast<uint8_t>(2), 0, 0, 0};
    struct can_frame tx
    {
    };
    yy_cybergear::data_frame_handler::buildWriteParamReq(
      host, motor, yy_cybergear::RUN_MODE, data, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Write RUN_MODE=Speed)\n";
    }
  }

  // Enable motors
  for (uint8_t motor : motors) {
    struct can_frame tx
    {
    };
    yy_cybergear::data_frame_handler::buildEnableReq(host, motor, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Enable)\n";
    }
  }

  // Control loop: set RunMode=Speed and speed reference
  const std::chrono::nanoseconds dt_ns{static_cast<long long>(1e9 / std::max(1, rate_hz))};
  while (g_running && rt.is_running()) {
    const auto start = clock::now();
    const auto deadline = start + dt_ns;

    // Write SPEED_REFERENCE (0x700A) for each motor
    for (uint8_t motor : motors) {
      float v = static_cast<float>(speed_rad_s);
      std::array<uint8_t, 4> raw{0, 0, 0, 0};
      std::memcpy(raw.data(), &v, 4);
      struct can_frame tx
      {
      };
      yy_cybergear::data_frame_handler::buildWriteParamReq(
        host, motor, yy_cybergear::SPEED_REFERENCE, raw, tx);
      rt.post(yy_socket_can::TxRequest{ifname, tx});
      if (verbose) {
        const uint32_t id = tx.can_id & CAN_EFF_MASK;
        std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                  << " (Write SPEED_REFERENCE)\n";
      }
    }

    // Stop if fault occurred (optional: user could watch last status via shared var)

    std::this_thread::sleep_until(deadline);
  }

  // Stop motors safely on exit
  for (uint8_t motor : motors) {
    struct can_frame tx
    {
    };
    yy_cybergear::data_frame_handler::buildStopReq(host, motor, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Stop)\n";
    }
  }

  rt.stop();
  return EXIT_SUCCESS;
}
