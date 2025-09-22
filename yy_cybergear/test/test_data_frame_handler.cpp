#include <gtest/gtest.h>
#include <linux/can.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "yy_cybergear/data_frame_handler.hpp"

namespace yyc = yy_cybergear::data_frame_handler;

namespace
{

// Tiny helpers for packing in tests
inline void packBE16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 0) & 0xFF);
}

inline void packLE32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>((v >> 0) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline uint16_t be16(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

inline uint16_t le16(const uint8_t * p)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(p[0]) << 0) | static_cast<uint16_t>(p[1]) << 8);
}

}  // namespace

TEST(OpCommandHandler, BasicZerosAndQuantization)
{
  can_frame f{};
  yyc::OpCommand cmd{};  // all zeros
  const uint8_t motor_id = 0x42;
  yyc::buildOpCtrlReq(motor_id, cmd, f);

  ASSERT_NE(f.can_id & CAN_EFF_FLAG, 0u);
  const uint32_t eid = f.can_id & CAN_EFF_MASK;

  // Type must be 1
  EXPECT_EQ(((eid >> 24) & 0x1Fu), 1u);
  // Low 8 bits = motor id
  EXPECT_EQ((eid & 0xFFu), motor_id);

  // Torque (0 Nm) maps to 0x8000 in bits 23..8
  const uint16_t u_tor = 0x8000;
  EXPECT_EQ(((eid >> 8) & 0xFFFFu), static_cast<uint32_t>(u_tor));

  ASSERT_EQ(f.can_dlc, 8);
  // pos(0) => 0x8000, vel(0)=>0x8000, kp(0)=>0, kd(0)=>0
  EXPECT_EQ(be16(&f.data[0]), 0x8000);
  EXPECT_EQ(be16(&f.data[2]), 0x8000);
  EXPECT_EQ(be16(&f.data[4]), 0x0000);
  EXPECT_EQ(be16(&f.data[6]), 0x0000);

  // Round-trip a non-zero cmd and verify quantization roughly inverts
  cmd.pos_rad = 1.5f;      // within [-4π,4π]
  cmd.vel_rad_s = -7.25f;  // within [-30,30]
  cmd.kp = 123.4f;         // within [0,500]
  cmd.kd = 3.3f;           // within [0,5]
  cmd.torque_Nm = 5.0f;    // within [-12,12]
  yyc::buildOpCtrlReq(motor_id, cmd, f);
  const uint16_t ut = static_cast<uint16_t>((((cmd.torque_Nm + 12.0f) / 24.0f) * 65535.0f) + 0.5f);
  EXPECT_EQ(((f.can_id & CAN_EFF_MASK) >> 8) & 0xFFFFu, ut);
}

TEST(ControlFrameHandler, HelperFunctions)
{
  can_frame f{};
  const uint8_t host = 0xA5;
  const uint8_t motor = 0x01;

  yyc::buildGetDeviceIdReq(host, motor, f);
  EXPECT_NE(f.can_id & CAN_EFF_FLAG, 0u);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(0, host, motor));
  EXPECT_EQ(f.can_dlc, 0);

  yyc::buildStopReq(host, motor, f);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(4, host, motor));
  EXPECT_EQ(f.can_dlc, 8);
  for (int i = 0; i < 8; ++i) EXPECT_EQ(f.data[i], 0);

  yyc::buildClearFaultsReq(host, motor, f);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(4, host, motor));
  EXPECT_EQ(f.data[0], 0x01);

  yyc::buildEnableReq(host, motor, f);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(3, host, motor));
  for (int i = 0; i < 8; ++i) EXPECT_EQ(f.data[i], 0);

  const uint8_t new_motor = 0x55;
  yyc::buildChangeMotorIdReq(host, motor, new_motor, f);
  const uint32_t id = f.can_id & CAN_EFF_MASK;
  EXPECT_EQ(((id >> 24) & 0x1Fu), 7u);
  EXPECT_EQ(((id >> 16) & 0xFFu), new_motor);
  EXPECT_EQ(((id >> 8) & 0xFFu), host);
  EXPECT_EQ((id & 0xFFu), motor);
  EXPECT_EQ(f.data[0], 0x01);

  const uint8_t code = 3;
  yyc::buildSetBaudRateReq(host, motor, code, f);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(22, host, motor));
  EXPECT_EQ(f.data[0], code);

  const uint16_t index = 0x1234;
  yyc::buildReadParamReq(host, motor, index, f);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(17, host, motor));
  EXPECT_EQ(le16(&f.data[0]), index);

  const std::array<uint8_t, 4> payload{{0xDE, 0xAD, 0xBE, 0xEF}};
  yyc::buildWriteParamReq(host, motor, index, payload, f);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(18, host, motor));
  EXPECT_EQ(le16(&f.data[0]), index);
  EXPECT_EQ(std::memcmp(&f.data[4], payload.data(), 4), 0);

  // Type 6: Set mechanical zero
  yyc::buildSetMechanicalZeroReq(host, motor, f);
  EXPECT_EQ((f.can_id & CAN_EFF_MASK), yyc::buildEffId(6, host, motor));
  EXPECT_EQ(f.can_dlc, 8);
  EXPECT_EQ(f.data[0], 0x01);
  for (int i = 1; i < 8; ++i) EXPECT_EQ(f.data[i], 0);
}

TEST(StatusHandler, ParseStatusFrame)
{
  can_frame f{};
  // Build an EFF id for type 2 with mode=2, fault bits=0x15, motor id=0x2C
  const uint8_t mode = 2;         // bits 23..22
  const uint8_t faults = 0x15;    // bits 21..16
  const uint8_t motor_id = 0x2C;  // bits 15..8
  const uint8_t host_id = 0x7F;   // bits 7..0 (don't care for decodeStatus)
  const uint32_t eid = (static_cast<uint32_t>(2u) << 24) |
                       (static_cast<uint32_t>(mode & 0x03u) << 22) |
                       (static_cast<uint32_t>(faults & 0x3Fu) << 16) |
                       (static_cast<uint32_t>(motor_id) << 8) | static_cast<uint32_t>(host_id);
  f.can_id = (eid & CAN_EFF_MASK) | CAN_EFF_FLAG;
  f.can_dlc = 8;
  // Encode pos=0, vel=0, tor=0, temp=25.0C
  packBE16(&f.data[0], 0x8000);
  packBE16(&f.data[2], 0x8000);
  packBE16(&f.data[4], 0x8000);
  packBE16(&f.data[6], 250);

  yyc::Status s{};
  ASSERT_TRUE(yyc::parseStatus(f, s));
  EXPECT_NEAR(s.angle_rad, 0.0f, 2.0e-4f);
  EXPECT_NEAR(s.vel_rad_s, 0.0f, 6.0e-4f);
  EXPECT_NEAR(s.torque_Nm, 0.0f, 2.0e-4f);
  EXPECT_FLOAT_EQ(s.temperature_c, 25.0f);
  EXPECT_EQ(s.motor_can_id, motor_id);
  EXPECT_EQ(s.mode, mode);
  EXPECT_EQ(s.fault_bits, static_cast<uint8_t>(faults & 0x3F));
  EXPECT_EQ(s.raw_eff_id, eid);

  // Negative: wrong type
  can_frame f2 = f;
  const uint32_t bad = (static_cast<uint32_t>(3u) << 24) | (eid & 0x00FFFFFFu);
  f2.can_id = bad | CAN_EFF_FLAG;
  EXPECT_FALSE(yyc::parseStatus(f2, s));
}

TEST(DeviceIdHandler, ParseMcuIdResponse)
{
  can_frame f{};
  // type==0, low8==0xFE, mid bytes contain motor id (either hi or lo)
  const uint8_t motor_id = 0x33;
  const uint8_t other_mid = 0x77;
  const uint32_t eid = (0u << 24) | (static_cast<uint32_t>(other_mid) << 16) |
                       (static_cast<uint32_t>(motor_id) << 8) | 0xFEu;
  f.can_id = eid | CAN_EFF_FLAG;
  f.can_dlc = yyc::kUidLen;
  std::array<uint8_t, yyc::kUidLen> uid{};
  for (int i = 0; i < yyc::kUidLen; ++i) {
    const uint8_t v = static_cast<uint8_t>(i * 3 + 1);
    f.data[i] = v;
    uid[i] = v;
  }

  std::array<uint8_t, yyc::kUidLen> out{};
  ASSERT_TRUE(yyc::parseDeviceIdResp(f, motor_id, out));
  EXPECT_EQ(out, uid);

  // Negative: low8 not 0xFE
  can_frame fbad = f;
  const uint32_t wrong_fe = (eid & ~0xFFu) | 0xAAu;
  fbad.can_id = wrong_fe | CAN_EFF_FLAG;
  EXPECT_FALSE(yyc::parseDeviceIdResp(fbad, motor_id, out));

  // Alternative: motor id in high middle byte also passes
  can_frame f_hi = f;
  const uint32_t eid_hi = (0u << 24) | (static_cast<uint32_t>(motor_id) << 16) |
                          (static_cast<uint32_t>(other_mid) << 8) | 0xFEu;
  f_hi.can_id = eid_hi | CAN_EFF_FLAG;
  EXPECT_TRUE(yyc::parseDeviceIdResp(f_hi, motor_id, out));
}

TEST(FaultWarningHandler, ParseFaultWarningResponse)
{
  can_frame f{};
  const uint8_t host = 0x55;
  const uint8_t motor = 0x2A;
  const uint32_t faults = 0x01234567;
  const uint32_t warns = 0x89ABCDEF;
  // Type 21 with Host in bits 15..8 and Motor in bits 7..0
  const uint32_t eid = (static_cast<uint32_t>(21u) << 24) | (static_cast<uint32_t>(host) << 8) |
                       static_cast<uint32_t>(motor);
  f.can_id = eid | CAN_EFF_FLAG;
  f.can_dlc = 8;
  packLE32(&f.data[0], faults);
  packLE32(&f.data[4], warns);

  yyc::FaultWarning fw{};
  ASSERT_TRUE(yyc::parseFaultWarningResp(f, host, fw));
  EXPECT_EQ(fw.faults, faults);
  EXPECT_EQ(fw.warnings, warns);

  // Negative: wrong host id
  EXPECT_FALSE(yyc::parseFaultWarningResp(f, static_cast<uint8_t>(host + 1), fw));
}

TEST(ReadParamHandler, ParseReadParamResponse)
{
  can_frame f{};
  const uint8_t host = 0x0A;
  const uint16_t index = 0xBEEF;
  const std::array<uint8_t, 4> data{{1, 2, 3, 4}};
  const uint32_t eid = (static_cast<uint32_t>(17u) << 24) | static_cast<uint32_t>(host);
  f.can_id = eid | CAN_EFF_FLAG;
  f.can_dlc = 8;
  // Little-endian index in response payload
  f.data[0] = static_cast<uint8_t>(index & 0xFF);
  f.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  std::memcpy(&f.data[4], data.data(), 4);

  std::array<uint8_t, 4> out{};
  ASSERT_TRUE(yyc::parseReadParamResp(f, host, index, out));
  EXPECT_EQ(out, data);

  // Negative: wrong index
  EXPECT_FALSE(yyc::parseReadParamResp(f, host, static_cast<uint16_t>(index + 1), out));
}
