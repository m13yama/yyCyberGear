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

#include "yy_cybergear/error_code.hpp"
#include "yy_cybergear/result.hpp"

namespace yy_cybergear
{

// Length of MCU UID in bytes
constexpr int kUidLen = 8;

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

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__PROTOCOL_TYPES_HPP_
