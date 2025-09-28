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

namespace yy_cybergear
{

// ======== Constants ========
constexpr int kUidLen = 8;

// ======== Parameter indices ========
constexpr uint16_t RUN_MODE = 0x7005;
constexpr uint16_t IQ_REFERENCE = 0x7006;

constexpr uint16_t SPEED_REFERENCE = 0x700A;
constexpr uint16_t TORQUE_LIMIT = 0x700B;

constexpr uint16_t CURRENT_KP = 0x7010;
constexpr uint16_t CURRENT_KI = 0x7011;
constexpr uint16_t CURRENT_FILTER_GAIN = 0x7014;

constexpr uint16_t POSITION_REFERENCE = 0x7016;
constexpr uint16_t SPEED_LIMIT = 0x7017;
constexpr uint16_t CURRENT_LIMIT = 0x7018;

constexpr uint16_t MECHANICAL_POSITION = 0x7019;
constexpr uint16_t IQ_FILTER = 0x701A;
constexpr uint16_t MECHANICAL_VELOCITY = 0x701B;
constexpr uint16_t BUS_VOLTAGE = 0x701C;
constexpr uint16_t ROTATION_TURNS = 0x701D;

constexpr uint16_t POSITION_KP = 0x701E;
constexpr uint16_t SPEED_KP = 0x701F;
constexpr uint16_t SPEED_KI = 0x7020;

// ======== Command & status types ========
struct OpCommand
{
  float pos_rad{0.0f};
  float vel_rad_s{0.0f};
  float kp{0.0f};
  float kd{0.0f};
  float torque_Nm{0.0f};
};

struct Status
{
  enum class StatusMode : uint8_t { Reset = 0, Calibration = 1, Run = 2 };

  float angle_rad{0.0f};
  float vel_rad_s{0.0f};
  float torque_Nm{0.0f};
  float temperature_c{0.0f};
  uint8_t motor_can_id{0};
  StatusMode status_mode{StatusMode::Reset};
  uint8_t fault_bits{0};
  uint32_t raw_eff_id{0};
};

struct FaultWarning
{
  uint32_t faults{0};
  uint32_t warnings{0};
};

enum class RunMode : uint32_t { OperationControl = 0, Position = 1, Speed = 2, Current = 3 };

// ======== Helpers ========
inline std::string run_mode_to_string(RunMode mode)
{
  switch (mode) {
    case RunMode::OperationControl:
      return "OperationControl";
    case RunMode::Position:
      return "Position";
    case RunMode::Speed:
      return "Speed";
    case RunMode::Current:
      return "Current";
    default:
      break;
  }
  return std::string("Unknown(") + std::to_string(static_cast<unsigned>(mode)) + ")";
}

inline std::string run_mode_to_string(uint32_t mode)
{
  return run_mode_to_string(static_cast<RunMode>(mode));
}

inline std::string status_mode_to_string(Status::StatusMode status_mode)
{
  switch (status_mode) {
    case Status::StatusMode::Reset:
      return "Reset";
    case Status::StatusMode::Calibration:
      return "Calibration";
    case Status::StatusMode::Run:
      return "Run";
    default:
      break;
  }
  return std::string("Unknown(") + std::to_string(static_cast<unsigned>(status_mode)) + ")";
}

inline std::string status_mode_to_string(uint8_t status_mode)
{
  return status_mode_to_string(static_cast<Status::StatusMode>(status_mode & 0x03u));
}

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
