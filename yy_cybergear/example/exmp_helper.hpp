#pragma once

#include <linux/can.h>
#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "yy_cybergear/cybergear.hpp"
#include "yy_cybergear/logging.hpp"
#include "yy_socket_can/can_runtime.hpp"

namespace exmp_helper
{
using Clock = std::chrono::steady_clock;

inline void print_status(const yy_cybergear::CyberGear & cg, double t_sec)
{
  std::cout << yy_cybergear::logging::formatStatusLine(cg, t_sec) << '\n';
}

inline void print_params(const yy_cybergear::CyberGear & cg)
{
  std::cout << yy_cybergear::logging::formatParamsSummary(cg);
}

inline bool check_for_errors(const yy_cybergear::CyberGear & cg)
{
  if (cg.isStatusInitialized()) {
    const auto status = cg.getStatus();
    if (status.fault_bits != 0) {
      std::cerr << "ERROR: Motor 0x" << std::uppercase << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<unsigned>(cg.motor_id()) << std::dec
                << " has status fault bits: 0x" << std::uppercase << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<unsigned>(status.fault_bits) << std::dec
                << "\n";

      auto fault_strings = yy_cybergear::logging::faultBitsToString(status.fault_bits);
      for (const auto & fault : fault_strings) {
        std::cerr << "  - " << fault << "\n";
      }
      return true;
    }
  }

  const uint32_t fault_bits = cg.fault_bits_agg();
  const uint32_t warning_bits = cg.warning_bits_agg();

  if (fault_bits != 0) {
    std::cerr << "ERROR: Motor 0x" << std::uppercase << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<unsigned>(cg.motor_id()) << std::dec
              << " has aggregate fault bits: 0x" << std::uppercase << std::hex << std::setw(8)
              << std::setfill('0') << fault_bits << std::dec << "\n";

    auto fault_strings =
      yy_cybergear::logging::faultBitsToString(fault_bits);
    for (const auto & fault : fault_strings) {
      std::cerr << "  - " << fault << "\n";
    }
    return true;
  }

  if (warning_bits != 0) {
    std::cerr << "WARNING: Motor 0x" << std::uppercase << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<unsigned>(cg.motor_id()) << std::dec
              << " has warning bits: 0x" << std::uppercase << std::hex << std::setw(8)
              << std::setfill('0') << warning_bits << std::dec << "\n";
  }

  return false;
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
  std::atomic<bool> & running, yy_socket_can::CanRuntime & rt, const std::string & ifname,
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
  while (running.load() && !all_ready) {
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

inline bool wait_for_enter_or_sigint(std::atomic<bool> & running)
{
  while (running.load()) {
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

}  // namespace exmp_helper

