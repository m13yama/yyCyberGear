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

#include "yy_cybergear/data_frame_handler.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "yy_cybergear/byte_utils.hpp"

namespace yy_cybergear
{
namespace data_frame_handler
{
namespace
{
constexpr float kPi = 3.14159265358979323846f;
}  // namespace

void buildGetDeviceIdReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // Build EFF-ID for type 0 (device ID request)
  const uint32_t req_id = buildEffId(0, host_id, motor_id);
  // Use Extended Frame Format (EFF)
  out.can_id = (req_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  // No payload for device ID request
  out.can_dlc = yy_cybergear::can_dlc::DeviceIdReq;
}

void buildOpCtrlReq(uint8_t motor_id, const OpCommand & cmd, struct can_frame & out)
{
  // Quantize torque (Nm) into 16-bit and embed in EFF-ID per protocol
  const uint16_t tor = yy_cybergear::byte_util::f2u16(cmd.torque_Nm, -12.0f, 12.0f);
  const uint32_t id = (static_cast<uint32_t>(1u) << 24) | (static_cast<uint32_t>(tor) << 8) |
                      static_cast<uint32_t>(motor_id);
  // Extended frame with 8-byte payload
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::OpControl;
  // Quantize and pack P, V, Kp, Kd as big-endian uint16 into data[0..7]
  const uint16_t p = yy_cybergear::byte_util::f2u16(cmd.pos_rad, -4.0f * kPi, 4.0f * kPi);
  const uint16_t v = yy_cybergear::byte_util::f2u16(cmd.vel_rad_s, -30.0f, 30.0f);
  const uint16_t kp = yy_cybergear::byte_util::f2u16(cmd.kp, 0.0f, 500.0f);
  const uint16_t kd = yy_cybergear::byte_util::f2u16(cmd.kd, 0.0f, 5.0f);
  yy_cybergear::byte_util::packBE16(&out.data[0], p);
  yy_cybergear::byte_util::packBE16(&out.data[2], v);
  yy_cybergear::byte_util::packBE16(&out.data[4], kp);
  yy_cybergear::byte_util::packBE16(&out.data[6], kd);
}

void buildStopReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // According to protocol 4.1.5: type 4 is Motor stopped (data area cleared to 0)
  const uint32_t id = buildEffId(4, host_id, motor_id);
  // Extended frame with 8 zero bytes
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::StopOrClear;
  std::memset(out.data, 0, sizeof(out.data));
}

void buildClearFaultsReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // 4.1.5: Clearing faults is also type 4 with Byte[0] = 1
  const uint32_t id = buildEffId(4, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::StopOrClear;
  std::memset(out.data, 0, sizeof(out.data));
  // Set flag to request fault clear
  out.data[0] = 0x01;
}

void buildEnableReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // 4.1.4: Motor enable operation is type 3; data area cleared to 0
  const uint32_t id = buildEffId(3, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  // Extended frame with zeroed payload
  out.can_dlc = yy_cybergear::can_dlc::Enable;
  std::memset(out.data, 0, sizeof(out.data));
}

void buildChangeMotorIdReq(
  uint8_t host_id, uint8_t motor_id, uint8_t new_motor_id, struct can_frame & out)
{
  // Type 7: change motor ID (new ID placed in EFF-ID high byte field)
  const uint32_t id = (static_cast<uint32_t>(7u) << 24) |
                      (static_cast<uint32_t>(new_motor_id) << 16) |
                      (static_cast<uint32_t>(host_id) << 8) | static_cast<uint32_t>(motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::ChangeMotorId;
  std::memset(out.data, 0, sizeof(out.data));
  // Apply flag
  out.data[0] = 0x01;
}

void buildSetMechanicalZeroReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // 4.1.6: Setting mechanical zero is type 6; data area cleared with Byte[0] = 1
  const uint32_t id = buildEffId(6, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::SetMechanicalZero;
  std::memset(out.data, 0, sizeof(out.data));
  // Commit zeroing
  out.data[0] = 0x01;
}

void buildSetBaudRateReq(uint8_t host_id, uint8_t motor_id, uint8_t code, struct can_frame & out)
{
  // Type 22: set CAN baud rate, code in data[0]
  const uint32_t id = buildEffId(22, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::SetBaudRateReq;
  std::memset(out.data, 0, sizeof(out.data));
  out.data[0] = code;
}

void buildReadParamReq(uint8_t host_id, uint8_t motor_id, uint16_t index, struct can_frame & out)
{
  // Type 17: read parameter (little-endian 16-bit index in payload)
  const uint32_t id = buildEffId(17, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::ReadParamReq;
  std::memset(out.data, 0, sizeof(out.data));
  // Protocol uses little-endian for parameter index in payload
  yy_cybergear::byte_util::packLE16(&out.data[0], index);
}

void buildWriteParamReq(
  uint8_t host_id, uint8_t motor_id, uint16_t index, const std::array<uint8_t, 4> & data,
  struct can_frame & out)
{
  // Type 18: write parameter (LE 16-bit index + 4 bytes data)
  const uint32_t id = buildEffId(18, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = yy_cybergear::can_dlc::WriteParamReq;
  std::memset(out.data, 0, sizeof(out.data));
  // Protocol uses little-endian for parameter index in payload
  yy_cybergear::byte_util::packLE16(&out.data[0], index);
  std::copy_n(data.data(), 4, &out.data[4]);
}

DataFrameType getFrameType(const struct can_frame & in) noexcept
{
  // Only extended frames carry typed EIDs
  if ((in.can_id & CAN_EFF_FLAG) == 0) return DataFrameType::Unknown;

  // Extract extended ID
  const uint32_t eid = in.can_id & CAN_EFF_MASK;

  // Decode 5-bit type field from EID (bits 24..28)
  const uint8_t type = static_cast<uint8_t>((eid >> 24) & 0x1Fu);
  switch (type) {
    case 0:
      return DataFrameType::Type0_DeviceId;
    case 1:
      return DataFrameType::Type1_OpControl;
    case 2:
      return DataFrameType::Type2_Status;
    case 3:
      return DataFrameType::Type3_Enable;
    case 4:
      return DataFrameType::Type4_StopOrClear;
    case 6:
      return DataFrameType::Type6_SetMechanicalZero;
    case 7:
      return DataFrameType::Type7_ChangeMotorId;
    case 17:
      return DataFrameType::Type17_ReadParam;
    case 18:
      return DataFrameType::Type18_WriteParam;
    case 21:
      return DataFrameType::Type21_FaultWarning;
    case 22:
      return DataFrameType::Type22_SetBaudRate;
    default:
      break;
  }
  return DataFrameType::Unknown;
}

bool parseDeviceIdResp(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id,
  std::array<uint8_t, yy_cybergear::can_dlc::DeviceIdResp> & out_uid, bool type_check)
{
  if (type_check) {
    // Check Extended Frame Format (EFF)
    if ((in.can_id & CAN_EFF_FLAG) == 0) return false;

    // Check frame type
    const auto t = getFrameType(in);
    if (t != DataFrameType::Type0_DeviceId) return false;
  }

  // Extract extended ID
  const uint32_t eid = in.can_id & CAN_EFF_MASK;

  // Check host id
  const uint8_t hid = static_cast<uint8_t>((eid >> 16) & 0xFFu);
  if (hid != expect_host_id) return false;

  // Check motor id
  const uint8_t mid = static_cast<uint8_t>((eid >> 8) & 0xFFu);
  if (mid != expect_motor_id) return false;

  // Response spec: low 8 bits should be 0xFE
  if ((eid & 0xFFu) != 0xFEu) return false;

  // UID length must match
  if (in.can_dlc != yy_cybergear::can_dlc::DeviceIdResp) return false;

  // Copy UID bytes
  std::copy_n(&in.data[0], yy_cybergear::can_dlc::DeviceIdResp, out_uid.begin());

  return true;
}

bool parseFaultWarningResp(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id, FaultWarning & out,
  bool type_check)
{
  if (type_check) {
    // Check Extended Frame Format (EFF)
    if ((in.can_id & CAN_EFF_FLAG) == 0) return false;

    // Check frame type
    const auto t = getFrameType(in);
    if (t != DataFrameType::Type21_FaultWarning) return false;
  }

  // Extract extended ID
  const uint32_t eid = in.can_id & CAN_EFF_MASK;

  // Check host id
  const uint8_t hid = static_cast<uint8_t>((eid >> 8) & 0xFFu);
  if (hid != expect_host_id) return false;

  // Check motor id
  const uint8_t mid = static_cast<uint8_t>(eid & 0xFFu);
  if (mid != expect_motor_id) return false;

  // Expect 8-byte payload
  if (in.can_dlc != yy_cybergear::can_dlc::FaultWarningResp) return false;

  // Parse LE32 fault and warning bitfields
  out.faults = yy_cybergear::byte_util::readLE32(&in.data[0]);
  out.warnings = yy_cybergear::byte_util::readLE32(&in.data[4]);

  return true;
}

bool parseReadParamResp(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id,
  uint16_t & out_index, std::array<uint8_t, 4> & out_data, bool type_check)
{
  if (type_check) {
    // Check Extended Frame Format (EFF)
    if ((in.can_id & CAN_EFF_FLAG) == 0) return false;

    // Check frame type
    const auto t = getFrameType(in);
    if (t != DataFrameType::Type17_ReadParam) return false;
  }

  // Extract extended ID
  const uint32_t eid = in.can_id & CAN_EFF_MASK;

  // Check host id
  const uint8_t hid = static_cast<uint8_t>(eid & 0xFFu);
  if (hid != expect_host_id) return false;

  // Check motor id
  const uint8_t mid = static_cast<uint8_t>((eid >> 8) & 0xFFu);
  if (mid != expect_motor_id) return false;

  // Expect 8-byte payload
  if (in.can_dlc != yy_cybergear::can_dlc::ReadParamResp) return false;

  // Read LE16 index and copy 4 bytes of data
  const uint16_t idx = yy_cybergear::byte_util::readLE16(&in.data[0]);
  out_index = idx;
  std::copy_n(&in.data[4], 4, out_data.begin());

  return true;
}

bool parseStatus(
  const struct can_frame & in, uint8_t expect_host_id, uint8_t expect_motor_id, Status & out,
  bool type_check)
{
  if (type_check) {
    // Check Extended Frame Format (EFF)
    if ((in.can_id & CAN_EFF_FLAG) == 0) return false;

    // Check frame type
    const auto t = getFrameType(in);
    if (t != DataFrameType::Type2_Status) return false;
  }

  // Extract extended ID
  const uint32_t eid = in.can_id & CAN_EFF_MASK;

  // Check host id
  const uint8_t hid = static_cast<uint8_t>(eid & 0xFFu);
  if (hid != expect_host_id) return false;

  // Check motor id
  const uint8_t mid = static_cast<uint8_t>((eid >> 8) & 0xFFu);
  if (mid != expect_motor_id) return false;

  // Expect 8-byte payload
  if (in.can_dlc != yy_cybergear::can_dlc::Status) return false;

  // Parse motor status encoded in EFF-ID bits
  out.motor_can_id = mid;
  out.fault_bits = static_cast<uint8_t>((eid >> 16) & 0x3Fu);
  out.status_mode = static_cast<Status::StatusMode>(static_cast<uint8_t>((eid >> 22) & 0x03u));
  out.raw_eff_id = eid;

  // Parse motor status payload (big-endian u16 fields)
  const uint16_t u_pos = yy_cybergear::byte_util::readBE16(&in.data[0]);
  const uint16_t u_vel = yy_cybergear::byte_util::readBE16(&in.data[2]);
  const uint16_t u_tor = yy_cybergear::byte_util::readBE16(&in.data[4]);
  const uint16_t u_tmp = yy_cybergear::byte_util::readBE16(&in.data[6]);
  out.angle_rad = yy_cybergear::byte_util::u16toF(u_pos, -4.0f * kPi, 4.0f * kPi);
  out.vel_rad_s = yy_cybergear::byte_util::u16toF(u_vel, -30.0f, 30.0f);
  out.torque_Nm = yy_cybergear::byte_util::u16toF(u_tor, -12.0f, 12.0f);
  out.temperature_c = static_cast<float>(u_tmp) / 10.0f;

  return true;
}

}  // namespace data_frame_handler
}  // namespace yy_cybergear
