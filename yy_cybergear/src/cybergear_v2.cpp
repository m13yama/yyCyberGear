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

#include "yy_cybergear/cybergear_v2.hpp"

#include <cstring>

#include "yy_cybergear/data_frame_handler.hpp"

namespace yy_cybergear
{

// ===== Basic requests =====
void CyberGearV2::buildGetDeviceId(struct can_frame & out) const
{
  data_frame_handler::buildGetDeviceIdReq(host_id_, motor_id_, out);
}

void CyberGearV2::buildEnable(struct can_frame & out) const
{
  data_frame_handler::buildEnableReq(host_id_, motor_id_, out);
}

void CyberGearV2::buildStop(struct can_frame & out) const
{
  data_frame_handler::buildStopReq(host_id_, motor_id_, out);
}

void CyberGearV2::buildClearFaults(struct can_frame & out) const
{
  data_frame_handler::buildClearFaultsReq(host_id_, motor_id_, out);
}

void CyberGearV2::buildSetMechanicalZero(struct can_frame & out) const
{
  data_frame_handler::buildSetMechanicalZeroReq(host_id_, motor_id_, out);
}

void CyberGearV2::buildChangeMotorId(uint8_t new_motor_id, struct can_frame & out) const
{
  data_frame_handler::buildChangeMotorIdReq(host_id_, motor_id_, new_motor_id, out);
}

void CyberGearV2::buildOpControl(const OpCommand & cmd, struct can_frame & out) const
{
  data_frame_handler::buildOpCtrlReq(motor_id_, cmd, out);
}

void CyberGearV2::buildSetBaudRate(uint8_t code, struct can_frame & out) const
{
  data_frame_handler::buildSetBaudRateReq(host_id_, motor_id_, code, out);
}

// ===== Generic param =====
void CyberGearV2::buildReadParam(uint16_t index, struct can_frame & out) const
{
  data_frame_handler::buildReadParamReq(host_id_, motor_id_, index, out);
}

void CyberGearV2::buildWriteParam(
  uint16_t index, const std::array<uint8_t, 4> & data, struct can_frame & out) const
{
  data_frame_handler::buildWriteParamReq(host_id_, motor_id_, index, data, out);
}

// ===== Per-parameter helpers =====
void CyberGearV2::buildSetRunMode(uint32_t mode, struct can_frame & out) const
{
  buildWriteParam(RUN_MODE, packLE32(mode), out);
}

void CyberGearV2::buildSetIqReference(float ampere, struct can_frame & out) const
{
  buildWriteParam(IQ_REFERENCE, packLEf(ampere), out);
}

void CyberGearV2::buildSetSpeedReference(float rad_s, struct can_frame & out) const
{
  buildWriteParam(SPEED_REFERENCE, packLEf(rad_s), out);
}

void CyberGearV2::buildSetTorqueLimit(float nm, struct can_frame & out) const
{
  buildWriteParam(TORQUE_LIMIT, packLEf(nm), out);
}

void CyberGearV2::buildSetCurrentKp(float v, struct can_frame & out) const
{
  buildWriteParam(CURRENT_KP, packLEf(v), out);
}

void CyberGearV2::buildSetCurrentKi(float v, struct can_frame & out) const
{
  buildWriteParam(CURRENT_KI, packLEf(v), out);
}

void CyberGearV2::buildSetCurrentFilterGain(float v, struct can_frame & out) const
{
  buildWriteParam(CURRENT_FILTER_GAIN, packLEf(v), out);
}

void CyberGearV2::buildSetPositionReference(float rad, struct can_frame & out) const
{
  buildWriteParam(POSITION_REFERENCE, packLEf(rad), out);
}

void CyberGearV2::buildSetSpeedLimit(float rad_s, struct can_frame & out) const
{
  buildWriteParam(SPEED_LIMIT, packLEf(rad_s), out);
}

void CyberGearV2::buildSetCurrentLimit(float ampere, struct can_frame & out) const
{
  buildWriteParam(CURRENT_LIMIT, packLEf(ampere), out);
}

void CyberGearV2::buildSetMechanicalPosition(float rad, struct can_frame & out) const
{
  buildWriteParam(MECHANICAL_POSITION, packLEf(rad), out);
}

void CyberGearV2::buildSetIqFilter(float ampere, struct can_frame & out) const
{
  buildWriteParam(IQ_FILTER, packLEf(ampere), out);
}

void CyberGearV2::buildSetMechanicalVelocity(float rad_s, struct can_frame & out) const
{
  buildWriteParam(MECHANICAL_VELOCITY, packLEf(rad_s), out);
}

void CyberGearV2::buildSetBusVoltage(float volt, struct can_frame & out) const
{
  buildWriteParam(BUS_VOLTAGE, packLEf(volt), out);
}

void CyberGearV2::buildSetRotationTurns(int16_t turns, struct can_frame & out) const
{
  buildWriteParam(ROTATION_TURNS, packLE16s(turns), out);
}

void CyberGearV2::buildSetPositionKp(float v, struct can_frame & out) const
{
  buildWriteParam(POSITION_KP, packLEf(v), out);
}

void CyberGearV2::buildSetSpeedKp(float v, struct can_frame & out) const
{
  buildWriteParam(SPEED_KP, packLEf(v), out);
}

void CyberGearV2::buildSetSpeedKi(float v, struct can_frame & out) const
{
  buildWriteParam(SPEED_KI, packLEf(v), out);
}

// ===== pack helpers =====
std::array<uint8_t, 4> CyberGearV2::packLE32(uint32_t v)
{
  return {
    static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>((v >> 8) & 0xFFu),
    static_cast<uint8_t>((v >> 16) & 0xFFu), static_cast<uint8_t>((v >> 24) & 0xFFu)};
}

std::array<uint8_t, 4> CyberGearV2::packLEf(float v)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit IEEE754");
  uint32_t u = 0;
  std::memcpy(&u, &v, 4);
  return packLE32(u);
}

std::array<uint8_t, 4> CyberGearV2::packLE16u(uint16_t v)
{
  return {static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>((v >> 8) & 0xFFu), 0x00, 0x00};
}

std::array<uint8_t, 4> CyberGearV2::packLE16s(int16_t v)
{
  const uint16_t u = static_cast<uint16_t>(v);
  return {static_cast<uint8_t>(u & 0xFFu), static_cast<uint8_t>((u >> 8) & 0xFFu), 0x00, 0x00};
}

bool CyberGearV2::updateFromDeviceIdResp(const struct can_frame & in, bool type_check)
{
  std::array<uint8_t, kUidLen> uid{};
  if (!data_frame_handler::parseDeviceIdResp(in, host_id_, motor_id_, uid, type_check)) {
    return false;
  }
  uid_ = uid;
  return true;
}

bool CyberGearV2::updateFromStatusFrame(const struct can_frame & in, bool type_check)
{
  data_frame_handler::Status s{};
  if (!data_frame_handler::parseStatus(in, host_id_, motor_id_, s, type_check)) {
    return false;
  }
  angle_rad_ = s.angle_rad;
  vel_rad_s_ = s.vel_rad_s;
  torque_Nm_ = s.torque_Nm;
  temperature_c_ = s.temperature_c;
  motor_can_id_ = s.motor_can_id;
  mode_ = s.mode;
  fault_bits_ = s.fault_bits;
  raw_eff_id_ = s.raw_eff_id;
  return true;
}

bool CyberGearV2::updateFromReadParamResp(const struct can_frame & in, bool type_check)
{
  uint16_t index = 0;
  std::array<uint8_t, 4> data{};
  if (!data_frame_handler::parseReadParamResp(in, host_id_, motor_id_, index, data, type_check)) {
    return false;
  }
  const uint32_t u32 = readLE32(data.data());

  // Interpret based on known parameter index
  switch (index) {
    case RUN_MODE:
      run_mode_ = u32;
      return true;
    case IQ_REFERENCE:
      std::memcpy(&iq_reference_, &u32, 4);
      return true;
    case SPEED_REFERENCE:
      std::memcpy(&speed_reference_, &u32, 4);
      return true;
    case TORQUE_LIMIT:
      std::memcpy(&torque_limit_, &u32, 4);
      return true;
    case CURRENT_KP:
      std::memcpy(&current_kp_, &u32, 4);
      return true;
    case CURRENT_KI:
      std::memcpy(&current_ki_, &u32, 4);
      return true;
    case CURRENT_FILTER_GAIN:
      std::memcpy(&current_filter_gain_, &u32, 4);
      return true;
    case POSITION_REFERENCE:
      std::memcpy(&position_reference_, &u32, 4);
      return true;
    case SPEED_LIMIT:
      std::memcpy(&speed_limit_, &u32, 4);
      return true;
    case CURRENT_LIMIT:
      std::memcpy(&current_limit_, &u32, 4);
      return true;
    case MECHANICAL_POSITION:
      std::memcpy(&mechanical_position_, &u32, 4);
      return true;
    case IQ_FILTER:
      std::memcpy(&iq_filter_, &u32, 4);
      return true;
    case MECHANICAL_VELOCITY:
      std::memcpy(&mechanical_velocity_, &u32, 4);
      return true;
    case BUS_VOLTAGE:
      std::memcpy(&bus_voltage_, &u32, 4);
      return true;
    case ROTATION_TURNS:
      rotation_turns_ = static_cast<int16_t>(u32 & 0xFFFFu);
      return true;
    case POSITION_KP:
      std::memcpy(&position_kp_, &u32, 4);
      return true;
    case SPEED_KP:
      std::memcpy(&speed_kp_, &u32, 4);
      return true;
    case SPEED_KI:
      std::memcpy(&speed_ki_, &u32, 4);
      return true;
    default:
      break;
  }
  return false;
}

bool CyberGearV2::updateFromFaultWarningResp(const struct can_frame & in, bool type_check)
{
  data_frame_handler::FaultWarning fw{};
  if (!data_frame_handler::parseFaultWarningResp(in, host_id_, motor_id_, fw, type_check)) {
    return false;
  }
  faults_bits_ = fw.faults;
  warnings_bits_ = fw.warnings;
  return true;
}

CyberGearV2::UpdateKind CyberGearV2::dispatchAndUpdate(const struct can_frame & in)
{
  namespace yycdf = data_frame_handler;
  const auto t = yycdf::getFrameType(in);
  switch (t) {
    case yycdf::DataFrameType::Type2_Status: {
      if (updateFromStatusFrame(in, false))
        return UpdateKind::Status;
      else
        return UpdateKind::Ignored;
    }
    case yycdf::DataFrameType::Type0_DeviceId: {
      if (updateFromDeviceIdResp(in, false))
        return UpdateKind::DeviceId;
      else
        return UpdateKind::Ignored;
    }
    case yycdf::DataFrameType::Type21_FaultWarning: {
      if (updateFromFaultWarningResp(in, false))
        return UpdateKind::FaultWarning;
      else
        return UpdateKind::Ignored;
    }
    case yycdf::DataFrameType::Type17_ReadParam: {
      if (updateFromReadParamResp(in, false))
        return UpdateKind::ReadParam;
      else
        return UpdateKind::Ignored;
    }
    default:
      break;
  }
  return UpdateKind::None;
}

}  // namespace yy_cybergear
