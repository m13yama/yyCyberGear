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

// exmp_00: guide a single motor through parameter setup, mechanical zeroing,
// and a short hold at zero position using the CyberGear helper with CanRuntime.

#include <linux/can.h>
#include <poll.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/logging.hpp"
#include "yy_socket_can/can_runtime.hpp"

#include "exmp_helper.hpp"

namespace
{
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

using exmp_helper::Clock;
using exmp_helper::check_for_errors;
using exmp_helper::preflight_sync;
using exmp_helper::print_params;
using exmp_helper::print_status;
using exmp_helper::register_can_handler;
using exmp_helper::wait_for_enter_or_sigint;

void sleep_until_or_abort(const Clock::time_point & deadline, Clock::duration poll)
{
  while (g_running && Clock::now() + poll < deadline) {
    std::this_thread::sleep_for(poll);
  }
  if (g_running) {
    std::this_thread::sleep_until(deadline);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  CLI::App app{"exmp_00: set key parameters, capture mechanical zero, and drive to zero"};

  std::string ifname{"can0"};
  std::string host_id_str{"0x00"};
  std::string motor_id_str{"0x01"};
  bool verbose = false;

  std::optional<double> speed_limit;
  std::optional<double> current_limit;
  std::optional<double> torque_limit;
  std::optional<double> position_kp;
  std::optional<double> speed_kp;
  std::optional<double> speed_ki;

  int status_rate_hz = 20;
  double hold_duration_sec = 3.0;  // time to hold zero after enable

  app.add_option("-i,--interface", ifname, "CAN interface (e.g., can0)")
    ->capture_default_str();
  app.add_option("-H,--host-id", host_id_str, "Host ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_option("-M,--motor-id", motor_id_str, "Motor ID (decimal or 0x-prefixed hex)")
    ->capture_default_str();
  app.add_flag("-v,--verbose", verbose, "Verbose CAN frame prints");

  app.add_option("--speed-limit", speed_limit, "Write SPEED_LIMIT [rad/s]")
    ->check(CLI::NonNegativeNumber);
  app.add_option("--current-limit", current_limit, "Write CURRENT_LIMIT [A]")
    ->check(CLI::NonNegativeNumber);
  app.add_option("--torque-limit", torque_limit, "Write TORQUE_LIMIT [Nm]")
    ->check(CLI::NonNegativeNumber);
  app.add_option("--position-kp", position_kp, "Write POSITION_KP (dimensionless)")
    ->check(CLI::NonNegativeNumber);
  app.add_option("--speed-kp", speed_kp, "Write SPEED_KP (dimensionless)")
    ->check(CLI::NonNegativeNumber);
  app.add_option("--speed-ki", speed_ki, "Write SPEED_KI (dimensionless)")
    ->check(CLI::NonNegativeNumber);

  app.add_option("--hold-sec", hold_duration_sec, "Duration to hold zero after enable [s]")
    ->check(CLI::NonNegativeNumber)
    ->capture_default_str();
  app.add_option("--status-rate", status_rate_hz, "Status print rate during hold [Hz]")
    ->check(CLI::PositiveNumber)
    ->capture_default_str();

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  unsigned long host_ul = 0;
  unsigned long motor_ul = 0;
  try {
    host_ul = std::stoul(host_id_str, nullptr, 0);
    motor_ul = std::stoul(motor_id_str, nullptr, 0);
  } catch (const std::exception & e) {
    std::cerr << "Invalid ID: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  if (host_ul > 0xFFul || motor_ul > 0xFFul) {
    std::cerr << "Host and motor IDs must be 0..255 (got host=" << host_ul << ", motor="
              << motor_ul << ")\n";
    return EXIT_FAILURE;
  }
  const uint8_t host = static_cast<uint8_t>(host_ul & 0xFFu);
  const uint8_t motor = static_cast<uint8_t>(motor_ul & 0xFFu);

  std::signal(SIGINT, handle_sigint);
  std::signal(SIGTERM, handle_sigint);

  yy_socket_can::CanRuntime rt;
  rt.setWarningLogger([](const std::string & m) { std::cerr << m << std::endl; });
  rt.addChannel(ifname);

  std::vector<yy_cybergear::CyberGear> cgs;
  cgs.emplace_back(host, motor);
  auto & cg = cgs.front();

  register_can_handler(rt, cgs, verbose);
  rt.start();

  const auto t0 = Clock::now();

  std::cout << "exmp_00 on " << ifname << " targeting motor 0x" << std::uppercase << std::hex
            << std::setw(2) << std::setfill('0') << static_cast<unsigned>(motor) << std::dec
            << ". Optional parameter writes will be applied before zeroing.\n";

  const std::vector<uint16_t> preflight_params = {
    yy_cybergear::RUN_MODE,
    yy_cybergear::SPEED_LIMIT,
    yy_cybergear::CURRENT_LIMIT,
    yy_cybergear::TORQUE_LIMIT,
    yy_cybergear::POSITION_KP,
    yy_cybergear::SPEED_KP,
    yy_cybergear::SPEED_KI,
    yy_cybergear::POSITION_REFERENCE,
    yy_cybergear::MECHANICAL_POSITION,
    yy_cybergear::CURRENT_KP,
    yy_cybergear::CURRENT_KI,
    yy_cybergear::CURRENT_FILTER_GAIN,
  };
  preflight_sync(g_running, rt, ifname, cgs, preflight_params, verbose);
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  if (check_for_errors(cg)) {
    std::cerr << "ERROR: Motor reported faults during initialization.\n";
    rt.stop();
    return EXIT_FAILURE;
  }

  std::cout << "\nInitial parameters:\n";
  print_params(cg);

  using SetterFn = void (yy_cybergear::CyberGear::*)(float, struct can_frame &) const noexcept;
  auto maybe_write_param = [&](const std::optional<double> & value, SetterFn setter,
                               uint16_t index, const std::string & label) {
    if (!value) return;
    const float fvalue = static_cast<float>(*value);
    std::cout << "Writing " << label << " = " << *value << '\n';
    struct can_frame tx
    {
    };
    (cg.*setter)(fvalue, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Write "
                << label << ")\n";
    }
    struct can_frame read_tx
    {
    };
    cg.buildReadParam(index, read_tx);
    rt.post(yy_socket_can::TxRequest{ifname, read_tx});
    if (verbose) {
      const uint32_t id = read_tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (Read " << label
                << ")\n";
    }
  };

  maybe_write_param(speed_limit, &yy_cybergear::CyberGear::buildSetSpeedLimit,
                    yy_cybergear::SPEED_LIMIT, "SPEED_LIMIT");
  maybe_write_param(current_limit, &yy_cybergear::CyberGear::buildSetCurrentLimit,
                    yy_cybergear::CURRENT_LIMIT, "CURRENT_LIMIT");
  maybe_write_param(torque_limit, &yy_cybergear::CyberGear::buildSetTorqueLimit,
                    yy_cybergear::TORQUE_LIMIT, "TORQUE_LIMIT");
  maybe_write_param(position_kp, &yy_cybergear::CyberGear::buildSetPositionKp,
                    yy_cybergear::POSITION_KP, "POSITION_KP");
  maybe_write_param(speed_kp, &yy_cybergear::CyberGear::buildSetSpeedKp,
                    yy_cybergear::SPEED_KP, "SPEED_KP");
  maybe_write_param(speed_ki, &yy_cybergear::CyberGear::buildSetSpeedKi,
                    yy_cybergear::SPEED_KI, "SPEED_KI");

  if (speed_limit || current_limit || torque_limit || position_kp || speed_kp || speed_ki) {
    std::cout << "Waiting for parameter write responses..." << std::endl;
    sleep_until_or_abort(Clock::now() + std::chrono::milliseconds(300), std::chrono::milliseconds(20));
  }

  if (check_for_errors(cg)) {
    std::cerr << "ERROR: Motor reported faults after parameter writes.\n";
    rt.stop();
    return EXIT_FAILURE;
  }

  std::cout << "\nUpdated parameters:\n";
  print_params(cg);

  std::cout << "\nAlign the motor to your desired zero position, then press Enter to store mechanical zero (Ctrl+C to abort)." << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  {
    struct can_frame tx
    {
    };
    cg.buildSetMechanicalZero(tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    std::cout << "Mechanical zero command sent." << std::endl;
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec << " (SetMechanicalZero)\n";
    }
  }

  {
    struct can_frame tx
    {
    };
    cg.buildReadParam(yy_cybergear::MECHANICAL_POSITION, tx);
    rt.post(yy_socket_can::TxRequest{ifname, tx});
    if (verbose) {
      const uint32_t id = tx.can_id & CAN_EFF_MASK;
      std::cout << "TX 0x" << std::hex << std::uppercase << id << std::dec
                << " (Read MECHANICAL_POSITION)\n";
    }
  }

  sleep_until_or_abort(Clock::now() + std::chrono::milliseconds(200), std::chrono::milliseconds(20));

  if (check_for_errors(cg)) {
    std::cerr << "ERROR: Motor reported faults after zeroing command.\n";
    rt.stop();
    return EXIT_FAILURE;
  }

  std::cout << "\nPress Enter to enable the motor and drive/hold at zero (Ctrl+C to abort)." << std::endl;
  if (!wait_for_enter_or_sigint(g_running)) {
    rt.stop();
    return EXIT_SUCCESS;
  }
  if (!g_running) {
    rt.stop();
    return EXIT_SUCCESS;
  }

  {
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
  {
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

  const std::chrono::nanoseconds dt_ns{
    static_cast<long long>(1e9 / std::max(1, status_rate_hz))};
  const auto hold_deadline = Clock::now() + std::chrono::duration<double>(hold_duration_sec);

  while (g_running && Clock::now() < hold_deadline) {
    const auto loop_start = Clock::now();
    const double t_now = std::chrono::duration<double>(loop_start - t0).count();

    if (check_for_errors(cg)) {
      std::cerr << "ERROR: Fault detected during zero hold. Aborting." << std::endl;
      g_running = false;
      break;
    }

    print_status(cg, t_now);

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

    sleep_until_or_abort(loop_start + dt_ns, std::chrono::milliseconds(5));
  }

  {
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
