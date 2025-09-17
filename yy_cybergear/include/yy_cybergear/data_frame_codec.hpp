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

#ifndef YY_CYBERGEAR__DATA_FRAME_CODEC_HPP_
#define YY_CYBERGEAR__DATA_FRAME_CODEC_HPP_

#include <linux/can.h>

#include <array>
#include <cstdint>

#include "yy_cybergear/protocol_types.hpp"

namespace yy_cybergear
{
namespace data_frame_codec
{

// Pure helper API to build and parse CyberGear CAN frames.
// Implemented as free functions in a namespace (no classes), suitable for unit testing.

// Expose common types from root namespace for backward compatibility
using OpCommand = yy_cybergear::OpCommand;
using Status = yy_cybergear::Status;
using FaultWarning = yy_cybergear::FaultWarning;
constexpr int kUidLen = yy_cybergear::kUidLen;

// ========= Builders =========
// Type 0: Get device ID request
void buildGetDeviceIdReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
// Type 1: Operation control request
void buildOpCtrlReq(uint8_t motor_id, const OpCommand & cmd, struct can_frame & out);
// Type 3: Enable motor request
void buildEnableReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
// Type 4: Stop motor request (data cleared)
void buildStopReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
// Type 4: Clear faults request (Byte[0]=1)
void buildClearFaultsReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
// Type 7: Change motor CAN_ID request
void buildChangeMotorIdReq(
  uint8_t host_id, uint8_t motor_id, uint8_t new_motor_id, struct can_frame & out);
// Type 6: Set mechanical zero (Byte[0]=1)
void buildSetMechanicalZeroReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
// Type 17: Read parameter request
void buildReadParamReq(uint8_t host_id, uint8_t motor_id, uint16_t index, struct can_frame & out);
// Type 18: Write parameter request
void buildWriteParamReq(
  uint8_t host_id, uint8_t motor_id, uint16_t index, const std::array<uint8_t, 4> & data,
  struct can_frame & out);
// Type 21: Fault/warning request
void buildFaultWarningReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
// Type 22: Set baud rate request
void buildSetBaudRateReq(uint8_t host_id, uint8_t motor_id, uint8_t code, struct can_frame & out);

// ========= Parsers =========
// MCU UID response: bits 28..24 == 0, low 8 bits == 0xFE, middle bytes contain motor id
bool parseDeviceIdResp(
  const struct can_frame & in, uint8_t expect_motor_id, std::array<uint8_t, kUidLen> & out_uid);
bool parseStatus(const struct can_frame & in, Status & out);
bool parseFaultWarningResp(const struct can_frame & in, uint8_t expect_host_id, FaultWarning & out);
bool parseReadParamResp(
  const struct can_frame & in, uint8_t expect_host_id, uint16_t expect_index,
  std::array<uint8_t, 4> & out_data);

// ======== ID helpers (for tests) ========
constexpr uint32_t buildEffId(uint8_t type, uint8_t host_id, uint8_t motor_id) noexcept
{
  return (static_cast<uint32_t>(type & 0x1Fu) << 24) | (static_cast<uint32_t>(host_id) << 8) |
         static_cast<uint32_t>(motor_id);
}

constexpr bool isExtended(uint32_t can_id) noexcept { return (can_id & CAN_EFF_FLAG) != 0; }

}  // namespace data_frame_codec
}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__DATA_FRAME_CODEC_HPP_
