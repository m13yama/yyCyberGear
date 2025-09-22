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

#ifndef YY_CYBERGEAR__PROTOCOL_TYPES_HPP_
#define YY_CYBERGEAR__PROTOCOL_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "yy_cybergear/error_code.hpp"
#include "yy_cybergear/result.hpp"

namespace yy_cybergear
{

// Length of MCU UID in bytes
constexpr int kUidLen = 8;

// Parameter index constants used in read/write parameter requests.
constexpr uint16_t RUN_MODE = 0x7005;      // Run mode (0: reset, 1: cali, 2: run)
constexpr uint16_t IQ_REFERENCE = 0x7006;  // Iq reference [A]

constexpr uint16_t SPEED_REFERENCE = 0x700A;  // Speed reference [rad/s]
constexpr uint16_t TORQUE_LIMIT = 0x700B;     // Torque limit [Nm]

constexpr uint16_t CURRENT_KP = 0x7010;           // Current loop Kp
constexpr uint16_t CURRENT_KI = 0x7011;           // Current loop Ki
constexpr uint16_t CURRENT_FILTER_GAIN = 0x7014;  // Current filter gain

constexpr uint16_t POSITION_REFERENCE = 0x7016;  // Position reference [rad]
constexpr uint16_t SPEED_LIMIT = 0x7017;         // Speed limit [rad/s]
constexpr uint16_t CURRENT_LIMIT = 0x7018;       // Current limit [A]

constexpr uint16_t MECHANICAL_POSITION = 0x7019;  // Mechanical position [rad]
constexpr uint16_t IQ_FILTER = 0x701A;            // Iq filtered [A]
constexpr uint16_t MECHANICAL_VELOCITY = 0x701B;  // Mechanical velocity [rad/s]
constexpr uint16_t BUS_VOLTAGE = 0x701C;          // Bus voltage [V]
constexpr uint16_t ROTATION_TURNS = 0x701D;       // Rotation turns (int16)

constexpr uint16_t POSITION_KP = 0x701E;  // Position loop Kp
constexpr uint16_t SPEED_KP = 0x701F;     // Speed loop Kp
constexpr uint16_t SPEED_KI = 0x7020;     // Speed loop Ki

// Command to control the actuator (used by op-control frames)
struct OpCommand
{
  float pos_rad{0.0f};    // [-4π, 4π]
  float vel_rad_s{0.0f};  // [-30, 30]
  float kp{0.0f};         // [0, 500]
  float kd{0.0f};         // [0, 5]
  float torque_Nm{0.0f};  // [-12, 12] (encoded in ID bits 23..8 for type-1 commands)
};

// Status feedback from the actuator (decoded from status frames)
struct Status
{
  float angle_rad{0.0f};
  float vel_rad_s{0.0f};
  float torque_Nm{0.0f};
  float temperature_c{0.0f};
  uint8_t motor_can_id{0};
  uint8_t mode{0};         // 0: reset, 1: cali, 2: run
  uint8_t fault_bits{0};   // lower 6 bits map to 21..16 per manual
  uint32_t raw_eff_id{0};  // 29-bit EFF identifier (no flags)
};

// Snapshot of device faults and warnings
struct FaultWarning
{
  uint32_t faults{0};
  uint32_t warnings{0};
};

// Human-readable helpers for decoding status fields
inline std::string mode_to_string(uint8_t mode)
{
  switch (mode & 0x03u) {
    case 0:
      return "Reset";
    case 1:
      return "Calibration";
    case 2:
      return "Run";
    default:
      break;
  }
  return std::string("Unknown(") + std::to_string(static_cast<unsigned>(mode)) + ")";
}

// Fault bits (from EFF ID bits 21..16) mapped onto Status::fault_bits lower 6 bits
// Returns a list of active fault labels. Empty vector means no faults.
inline std::vector<std::string> fault_bits_to_string(uint8_t fault_bits)
{
  const uint8_t fb = static_cast<uint8_t>(fault_bits & 0x3Fu);
  std::vector<std::string> out;
  if (fb & (1u << 0)) out.emplace_back("Undervoltage fault");
  if (fb & (1u << 1)) out.emplace_back("Overcurrent fault");
  if (fb & (1u << 2)) out.emplace_back("Over temperature fault");
  if (fb & (1u << 3)) out.emplace_back("Magnetic encoding failure");
  if (fb & (1u << 4)) out.emplace_back("HALL encoding failure");
  if (fb & (1u << 5)) out.emplace_back("not calibrated");
  return out;
}

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__PROTOCOL_TYPES_HPP_
