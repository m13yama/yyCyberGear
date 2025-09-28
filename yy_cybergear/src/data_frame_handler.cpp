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

namespace yy_cybergear
{
namespace data_frame_handler
{
namespace
{
constexpr float kPi = 3.14159265358979323846f;

inline float clipf(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }

inline uint16_t f2u16(float x, float xmin, float xmax)
{
  // Normalize into [0,1], quantize to [0,65535] with nearest rounding
  const float v = clipf(x, xmin, xmax);
  const float n = (v - xmin) / (xmax - xmin);
  float q = std::round(n * 65535.0f);
  if (q < 0.0f) q = 0.0f;
  if (q > 65535.0f) q = 65535.0f;
  return static_cast<uint16_t>(q);
}

inline float u16toF(uint16_t u, float xmin, float xmax)
{
  const float n = static_cast<float>(u) / 65535.0f;
  return xmin + n * (xmax - xmin);
}

inline void packBE16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 0) & 0xFF);
}

inline uint16_t readBE16(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

inline void packLE16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>((v >> 0) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline uint16_t readLE16(const uint8_t * p)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(p[0]) << 0) | static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t readLE32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

void buildGetDeviceIdReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // Build EFF-ID for type 0 (device ID request)
  const uint32_t req_id = buildEffId(0, host_id, motor_id);
  // Use Extended Frame Format (EFF)
  out.can_id = (req_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  // No payload for device ID request
  out.can_dlc = 0;
}

void buildOpCtrlReq(uint8_t motor_id, const OpCommand & cmd, struct can_frame & out)
{
  // Quantize torque (Nm) into 16-bit and embed in EFF-ID per protocol
  const uint16_t tor = f2u16(cmd.torque_Nm, -12.0f, 12.0f);
  const uint32_t id = (static_cast<uint32_t>(1u) << 24) | (static_cast<uint32_t>(tor) << 8) |
                      static_cast<uint32_t>(motor_id);
  // Extended frame with 8-byte payload
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  // Quantize and pack P, V, Kp, Kd as big-endian uint16 into data[0..7]
  const uint16_t p = f2u16(cmd.pos_rad, -4.0f * kPi, 4.0f * kPi);
  const uint16_t v = f2u16(cmd.vel_rad_s, -30.0f, 30.0f);
  const uint16_t kp = f2u16(cmd.kp, 0.0f, 500.0f);
  const uint16_t kd = f2u16(cmd.kd, 0.0f, 5.0f);
  packBE16(&out.data[0], p);
  packBE16(&out.data[2], v);
  packBE16(&out.data[4], kp);
  packBE16(&out.data[6], kd);
}

void buildStopReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // According to protocol 4.1.5: type 4 is Motor stopped (data area cleared to 0)
  const uint32_t id = buildEffId(4, host_id, motor_id);
  // Extended frame with 8 zero bytes
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
}

void buildClearFaultsReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // 4.1.5: Clearing faults is also type 4 with Byte[0] = 1
  const uint32_t id = buildEffId(4, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
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
  out.can_dlc = 8;
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
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  // Apply flag
  out.data[0] = 0x01;
}

void buildSetMechanicalZeroReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // 4.1.6: Setting mechanical zero is type 6; data area cleared with Byte[0] = 1
  const uint32_t id = buildEffId(6, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  // Commit zeroing
  out.data[0] = 0x01;
}

void buildSetBaudRateReq(uint8_t host_id, uint8_t motor_id, uint8_t code, struct can_frame & out)
{
  // Type 22: set CAN baud rate, code in data[0]
  const uint32_t id = buildEffId(22, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  out.data[0] = code;
}

void buildReadParamReq(uint8_t host_id, uint8_t motor_id, uint16_t index, struct can_frame & out)
{
  // Type 17: read parameter (little-endian 16-bit index in payload)
  const uint32_t id = buildEffId(17, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  // Protocol uses little-endian for parameter index in payload
  packLE16(&out.data[0], index);
}

void buildWriteParamReq(
  uint8_t host_id, uint8_t motor_id, uint16_t index, const std::array<uint8_t, 4> & data,
  struct can_frame & out)
{
  // Type 18: write parameter (LE 16-bit index + 4 bytes data)
  const uint32_t id = buildEffId(18, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  // Protocol uses little-endian for parameter index in payload
  packLE16(&out.data[0], index);
  std::copy_n(data.data(), 4, &out.data[4]);
}

DataFrameType getFrameType(const struct can_frame & in) noexcept
{
  // Only extended frames carry typed EIDs
  if ((in.can_id & CAN_EFF_FLAG) == 0) return DataFrameType::Unknown;
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
  std::array<uint8_t, kUidLen> & out_uid, bool type_check)
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
  if (in.can_dlc != kUidLen) return false;

  // Copy UID bytes
  std::copy_n(&in.data[0], kUidLen, out_uid.begin());

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
  if (in.can_dlc != 8) return false;

  // Parse LE32 fault and warning bitfields
  out.faults = readLE32(&in.data[0]);
  out.warnings = readLE32(&in.data[4]);

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
  if (in.can_dlc != 8) return false;

  // Read LE16 index and copy 4 bytes of data
  const uint16_t idx = readLE16(&in.data[0]);
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
  if (in.can_dlc != 8) return false;

  // Parse motor status encoded in EFF-ID bits
  out.motor_can_id = mid;
  out.fault_bits = static_cast<uint8_t>((eid >> 16) & 0x3Fu);
  out.mode = static_cast<uint8_t>((eid >> 22) & 0x03u);
  out.raw_eff_id = eid;

  // Parse motor status payload (big-endian u16 fields)
  const uint16_t u_pos = readBE16(&in.data[0]);
  const uint16_t u_vel = readBE16(&in.data[2]);
  const uint16_t u_tor = readBE16(&in.data[4]);
  const uint16_t u_tmp = readBE16(&in.data[6]);
  out.angle_rad = u16toF(u_pos, -4.0f * kPi, 4.0f * kPi);
  out.vel_rad_s = u16toF(u_vel, -30.0f, 30.0f);
  out.torque_Nm = u16toF(u_tor, -12.0f, 12.0f);
  out.temperature_c = static_cast<float>(u_tmp) / 10.0f;

  return true;
}

}  // namespace data_frame_handler
}  // namespace yy_cybergear
