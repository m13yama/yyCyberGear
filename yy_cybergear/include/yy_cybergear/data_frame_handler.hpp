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

#ifndef YY_CYBERGEAR__DATA_FRAME_HANDLER_HPP_
#define YY_CYBERGEAR__DATA_FRAME_HANDLER_HPP_

#include <linux/can.h>

#include <array>
#include <cstdint>

#include "yy_cybergear/protocol_types.hpp"

namespace yy_cybergear
{
namespace data_frame_handler
{
// Expose common types from root namespace for backward compatibility
using OpCommand = yy_cybergear::OpCommand;
using Status = yy_cybergear::Status;
using FaultWarning = yy_cybergear::FaultWarning;

// ======== Frame type classification ========
enum class DataFrameType : uint8_t {
  Unknown = 0xFF,
  Type0_DeviceId = 0,
  Type1_OpControl = 1,
  Type2_Status = 2,
  Type3_Enable = 3,
  Type4_StopOrClear = 4,
  Type6_SetMechanicalZero = 6,
  Type7_ChangeMotorId = 7,
  Type17_ReadParam = 17,
  Type18_WriteParam = 18,
  Type21_FaultWarning = 21,
  Type22_SetBaudRate = 22,
};

// ========= Builders =========
void buildGetDeviceIdReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
void buildOpCtrlReq(uint8_t motor_id, const OpCommand & cmd, struct can_frame & out);
void buildEnableReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
void buildStopReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
void buildClearFaultsReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
void buildChangeMotorIdReq(
  uint8_t host_id, uint8_t motor_id, uint8_t new_motor_id, struct can_frame & out);
void buildSetMechanicalZeroReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out);
void buildReadParamReq(uint8_t host_id, uint8_t motor_id, uint16_t index, struct can_frame & out);
void buildWriteParamReq(
  uint8_t host_id, uint8_t motor_id, uint16_t index, const std::array<uint8_t, 4> & data,
  struct can_frame & out);
void buildSetBaudRateReq(uint8_t host_id, uint8_t motor_id, uint8_t code, struct can_frame & out);

// ========= Parsers =========
bool parseDeviceIdResp(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id,
  std::array<uint8_t, yy_cybergear::can_dlc::DeviceIdResp> & out_uid, bool type_check = true);
bool parseStatus(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id, Status & out,
  bool type_check = true);
bool parseFaultWarningResp(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id, FaultWarning & out,
  bool type_check = true);
bool parseReadParamResp(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id,
  uint16_t & out_index, std::array<uint8_t, 4> & out_data, bool type_check = true);

// ======== ID helpers (for tests) ========
constexpr uint32_t buildEffId(uint8_t type, uint8_t host_id, uint8_t motor_id) noexcept
{
  return (static_cast<uint32_t>(type & 0x1Fu) << 24) | (static_cast<uint32_t>(host_id) << 8) |
         static_cast<uint32_t>(motor_id);
}

constexpr bool isExtended(uint32_t can_id) noexcept { return (can_id & CAN_EFF_FLAG) != 0; }

DataFrameType getFrameType(const struct can_frame & in) noexcept;

}  // namespace data_frame_handler
}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__DATA_FRAME_HANDLER_HPP_
