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

#include "yy_cybergear/data_frame_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace yy_cybergear
{
namespace data_frame_codec
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
  const uint32_t req_id = buildEffId(0, host_id, motor_id);
  out.can_id = (req_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 0;
}

void buildOpCtrlReq(uint8_t motor_id, const OpCommand & cmd, struct can_frame & out)
{
  const uint16_t tor = f2u16(cmd.torque_Nm, -12.0f, 12.0f);
  const uint32_t id = (static_cast<uint32_t>(1u) << 24) | (static_cast<uint32_t>(tor) << 8) |
                      static_cast<uint32_t>(motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
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
  out.data[0] = 0x01;
}

void buildEnableReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // 4.1.4: Motor enable operation is type 3; data area cleared to 0
  const uint32_t id = buildEffId(3, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
}

void buildChangeMotorIdReq(
  uint8_t host_id, uint8_t motor_id, uint8_t new_motor_id, struct can_frame & out)
{
  const uint32_t id = (static_cast<uint32_t>(7u) << 24) |
                      (static_cast<uint32_t>(new_motor_id) << 16) |
                      (static_cast<uint32_t>(host_id) << 8) | static_cast<uint32_t>(motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  out.data[0] = 0x01;
}

void buildSetMechanicalZeroReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  // 4.1.6: Setting mechanical zero is type 6; data area cleared with Byte[0] = 1
  const uint32_t id = buildEffId(6, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  out.data[0] = 0x01;
}

void buildFaultWarningReq(uint8_t host_id, uint8_t motor_id, struct can_frame & out)
{
  const uint32_t id = buildEffId(21, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
}

void buildSetBaudRateReq(uint8_t host_id, uint8_t motor_id, uint8_t code, struct can_frame & out)
{
  const uint32_t id = buildEffId(22, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  out.data[0] = code;
}

void buildReadParamReq(uint8_t host_id, uint8_t motor_id, uint16_t index, struct can_frame & out)
{
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
  const uint32_t id = buildEffId(18, host_id, motor_id);
  out.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  out.can_dlc = 8;
  std::memset(out.data, 0, sizeof(out.data));
  // Protocol uses little-endian for parameter index in payload
  packLE16(&out.data[0], index);
  std::copy_n(data.data(), 4, &out.data[4]);
}

bool parseDeviceIdResp(
  const struct can_frame & in, uint8_t expect_motor_id, std::array<uint8_t, kUidLen> & out_uid)
{
  if ((in.can_id & CAN_EFF_FLAG) == 0) return false;
  const uint32_t rid = in.can_id & CAN_EFF_MASK;
  if (((rid >> 24) & 0x1Fu) != 0) return false;  // bits 28..24 must be 0
  if ((rid & 0xFFu) != 0xFEu) return false;      // low 8 bits = 0xFE
  const uint8_t mid_lo = static_cast<uint8_t>((rid >> 8) & 0xFFu);
  const uint8_t mid_hi = static_cast<uint8_t>((rid >> 16) & 0xFFu);
  const uint8_t mid = static_cast<uint8_t>(expect_motor_id & 0xFFu);
  if (!((mid_lo == mid) || (mid_hi == mid))) return false;
  if (in.can_dlc != kUidLen) return false;
  std::copy_n(std::begin(in.data), kUidLen, out_uid.begin());
  return true;
}

bool parseStatus(const struct can_frame & in, Status & out)
{
  if ((in.can_id & CAN_EFF_FLAG) == 0) return false;
  const uint32_t eid = in.can_id & CAN_EFF_MASK;
  const uint8_t type = static_cast<uint8_t>((eid >> 24) & 0x1Fu);
  if (type != 2) return false;
  if (in.can_dlc != 8) return false;

  out.motor_can_id = static_cast<uint8_t>((eid >> 8) & 0xFFu);
  out.fault_bits = static_cast<uint8_t>((eid >> 16) & 0x3Fu);
  out.mode = static_cast<uint8_t>((eid >> 22) & 0x03u);
  out.raw_eff_id = eid;

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

bool parseFaultWarningResp(const struct can_frame & in, uint8_t expect_host_id, FaultWarning & out)
{
  if ((in.can_id & CAN_EFF_FLAG) == 0) return false;
  const uint32_t eid = in.can_id & CAN_EFF_MASK;
  const uint8_t type = static_cast<uint8_t>((eid >> 24) & 0x1Fu);
  if (type != 21) return false;
  // 4.1.10: Bit 15..8 holds Host CAN_ID in the ID field for fault frames
  if (static_cast<uint8_t>((eid >> 8) & 0xFFu) != expect_host_id) return false;
  if (in.can_dlc != 8) return false;
  out.faults = readLE32(&in.data[0]);
  out.warnings = readLE32(&in.data[4]);
  return true;
}

bool parseReadParamResp(
  const struct can_frame & in, uint8_t expect_host_id, uint16_t expect_index,
  std::array<uint8_t, 4> & out_data)
{
  if ((in.can_id & CAN_EFF_FLAG) == 0) return false;
  const uint32_t eid = in.can_id & CAN_EFF_MASK;
  const uint8_t type = static_cast<uint8_t>((eid >> 24) & 0x1Fu);
  if (type != 17) return false;
  if ((eid & 0xFFu) != expect_host_id) return false;
  if (in.can_dlc != 8) return false;
  // Parameter index is little-endian in payload
  const uint16_t idx = readLE16(&in.data[0]);
  if (idx != expect_index) return false;
  std::copy_n(&in.data[4], 4, out_data.begin());
  return true;
}

}  // namespace data_frame_codec
}  // namespace yy_cybergear
