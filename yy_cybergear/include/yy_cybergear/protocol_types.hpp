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

// ======== CAN DLC (data length) constants ========
namespace can_dlc
{
constexpr uint8_t DeviceIdReq = 0;
constexpr uint8_t DeviceIdResp = 8;
constexpr uint8_t OpControl = 8;
constexpr uint8_t Status = 8;
constexpr uint8_t Enable = 8;
constexpr uint8_t StopOrClear = 8;
constexpr uint8_t SetMechanicalZero = 8;
constexpr uint8_t ChangeMotorId = 8;
constexpr uint8_t ReadParamReq = 8;
constexpr uint8_t ReadParamResp = 8;
constexpr uint8_t WriteParamReq = 8;
constexpr uint8_t FaultWarningResp = 8;
constexpr uint8_t SetBaudRateReq = 8;
}  // namespace can_dlc

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
enum class RunMode : uint32_t { OperationControl = 0, Position = 1, Speed = 2, Current = 3 };

struct FaultWarning
{
  uint32_t faults{0};
  uint32_t warnings{0};
};
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
}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__PROTOCOL_TYPES_HPP_
