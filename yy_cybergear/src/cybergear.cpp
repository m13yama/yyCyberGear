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

#include "yy_cybergear/cybergear.hpp"

#include <cstring>

#include "yy_cybergear/byte_utils.hpp"
#include "yy_cybergear/data_frame_handler.hpp"

namespace yy_cybergear
{

// ===== Basic requests =====
void CyberGear::buildGetDeviceId(struct can_frame & out) const noexcept
{
  data_frame_handler::buildGetDeviceIdReq(host_id_, motor_id_, out);
}

void CyberGear::buildEnable(struct can_frame & out) const noexcept
{
  data_frame_handler::buildEnableReq(host_id_, motor_id_, out);
}

void CyberGear::buildStop(struct can_frame & out) const noexcept
{
  data_frame_handler::buildStopReq(host_id_, motor_id_, out);
}

void CyberGear::buildClearFaults(struct can_frame & out) const noexcept
{
  data_frame_handler::buildClearFaultsReq(host_id_, motor_id_, out);
}

void CyberGear::buildSetMechanicalZero(struct can_frame & out) const noexcept
{
  data_frame_handler::buildSetMechanicalZeroReq(host_id_, motor_id_, out);
}

void CyberGear::buildChangeMotorId(uint8_t new_motor_id, struct can_frame & out) const noexcept
{
  data_frame_handler::buildChangeMotorIdReq(host_id_, motor_id_, new_motor_id, out);
}

void CyberGear::buildOpControl(const OpCommand & cmd, struct can_frame & out) const noexcept
{
  data_frame_handler::buildOpCtrlReq(motor_id_, cmd, out);
}

void CyberGear::buildSetBaudRate(uint8_t code, struct can_frame & out) const noexcept
{
  data_frame_handler::buildSetBaudRateReq(host_id_, motor_id_, code, out);
}

// ===== Generic param =====
void CyberGear::buildReadParam(uint16_t index, struct can_frame & out) const noexcept
{
  data_frame_handler::buildReadParamReq(host_id_, motor_id_, index, out);
}

void CyberGear::buildWriteParam(
  uint16_t index, const std::array<uint8_t, 4> & data, struct can_frame & out) const noexcept
{
  data_frame_handler::buildWriteParamReq(host_id_, motor_id_, index, data, out);
}

// ===== Per-parameter helpers =====
void CyberGear::buildSetRunMode(RunMode mode, struct can_frame & out) const noexcept
{
  const auto v = static_cast<uint32_t>(mode);
  buildWriteParam(RUN_MODE, byte_util::packLE32(v), out);
}

void CyberGear::buildSetIqReference(float ampere, struct can_frame & out) const noexcept
{
  buildWriteParam(IQ_REFERENCE, byte_util::packLEf(ampere), out);
}

void CyberGear::buildSetSpeedReference(float rad_s, struct can_frame & out) const noexcept
{
  buildWriteParam(SPEED_REFERENCE, byte_util::packLEf(rad_s), out);
}

void CyberGear::buildSetTorqueLimit(float nm, struct can_frame & out) const noexcept
{
  buildWriteParam(TORQUE_LIMIT, byte_util::packLEf(nm), out);
}

void CyberGear::buildSetCurrentKp(float v, struct can_frame & out) const noexcept
{
  buildWriteParam(CURRENT_KP, byte_util::packLEf(v), out);
}

void CyberGear::buildSetCurrentKi(float v, struct can_frame & out) const noexcept
{
  buildWriteParam(CURRENT_KI, byte_util::packLEf(v), out);
}

void CyberGear::buildSetCurrentFilterGain(float v, struct can_frame & out) const noexcept
{
  buildWriteParam(CURRENT_FILTER_GAIN, byte_util::packLEf(v), out);
}

void CyberGear::buildSetPositionReference(float rad, struct can_frame & out) const noexcept
{
  buildWriteParam(POSITION_REFERENCE, byte_util::packLEf(rad), out);
}

void CyberGear::buildSetSpeedLimit(float rad_s, struct can_frame & out) const noexcept
{
  buildWriteParam(SPEED_LIMIT, byte_util::packLEf(rad_s), out);
}

void CyberGear::buildSetCurrentLimit(float ampere, struct can_frame & out) const noexcept
{
  buildWriteParam(CURRENT_LIMIT, byte_util::packLEf(ampere), out);
}

void CyberGear::buildSetMechanicalPosition(float rad, struct can_frame & out) const noexcept
{
  buildWriteParam(MECHANICAL_POSITION, byte_util::packLEf(rad), out);
}

void CyberGear::buildSetIqFilter(float ampere, struct can_frame & out) const noexcept
{
  buildWriteParam(IQ_FILTER, byte_util::packLEf(ampere), out);
}

void CyberGear::buildSetMechanicalVelocity(float rad_s, struct can_frame & out) const noexcept
{
  buildWriteParam(MECHANICAL_VELOCITY, byte_util::packLEf(rad_s), out);
}

void CyberGear::buildSetBusVoltage(float volt, struct can_frame & out) const noexcept
{
  buildWriteParam(BUS_VOLTAGE, byte_util::packLEf(volt), out);
}

void CyberGear::buildSetRotationTurns(int16_t turns, struct can_frame & out) const noexcept
{
  buildWriteParam(ROTATION_TURNS, byte_util::packLE16s(turns), out);
}

void CyberGear::buildSetPositionKp(float v, struct can_frame & out) const noexcept
{
  buildWriteParam(POSITION_KP, byte_util::packLEf(v), out);
}

void CyberGear::buildSetSpeedKp(float v, struct can_frame & out) const noexcept
{
  buildWriteParam(SPEED_KP, byte_util::packLEf(v), out);
}

void CyberGear::buildSetSpeedKi(float v, struct can_frame & out) const noexcept
{
  buildWriteParam(SPEED_KI, byte_util::packLEf(v), out);
}

bool CyberGear::updateFromDeviceIdResp(const struct can_frame & in, bool type_check) noexcept
{
  std::array<uint8_t, yy_cybergear::can_dlc::DeviceIdResp> uid{};
  if (!data_frame_handler::parseDeviceIdResp(in, host_id_, motor_id_, uid, type_check)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    uid_ = uid;
    uid_initialized_ = true;
  }
  return true;
}

bool CyberGear::updateFromStatusFrame(const struct can_frame & in, bool type_check) noexcept
{
  data_frame_handler::Status s{};
  if (!data_frame_handler::parseStatus(in, host_id_, motor_id_, s, type_check)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    status_ = s;
    status_initialized_ = true;
  }
  return true;
}

bool CyberGear::updateFromReadParamResp(const struct can_frame & in, bool type_check) noexcept
{
  uint16_t index = 0;
  std::array<uint8_t, 4> data{};
  if (!data_frame_handler::parseReadParamResp(in, host_id_, motor_id_, index, data, type_check)) {
    return false;
  }

  // Interpret based on known parameter index
  std::lock_guard<std::mutex> lk(mu_);
  switch (index) {
    case RUN_MODE:
      run_mode_ = static_cast<RunMode>(byte_util::readLE32(data.data()));
      initialized_params_.insert(index);
      return true;
    case IQ_REFERENCE:
      iq_reference_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case SPEED_REFERENCE:
      speed_reference_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case TORQUE_LIMIT:
      torque_limit_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case CURRENT_KP:
      current_kp_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case CURRENT_KI:
      current_ki_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case CURRENT_FILTER_GAIN:
      current_filter_gain_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case POSITION_REFERENCE:
      position_reference_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case SPEED_LIMIT:
      speed_limit_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case CURRENT_LIMIT:
      current_limit_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case MECHANICAL_POSITION:
      mechanical_position_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case IQ_FILTER:
      iq_filter_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case MECHANICAL_VELOCITY:
      mechanical_velocity_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case BUS_VOLTAGE:
      bus_voltage_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case ROTATION_TURNS:
      rotation_turns_ = byte_util::readLE16s(data.data());
      initialized_params_.insert(index);
      return true;
    case POSITION_KP:
      position_kp_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case SPEED_KP:
      speed_kp_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    case SPEED_KI:
      speed_ki_ = byte_util::readLEf(data.data());
      initialized_params_.insert(index);
      return true;
    default:
      break;
  }
  return false;
}

bool CyberGear::updateFromFaultWarningResp(const struct can_frame & in, bool type_check) noexcept
{
  data_frame_handler::FaultWarning fw{};
  if (!data_frame_handler::parseFaultWarningResp(in, host_id_, motor_id_, fw, type_check)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    fault_bits_agg_ = fw.faults;
    warning_bits_agg_ = fw.warnings;
  }
  return true;
}

CyberGear::UpdateKind CyberGear::dispatchAndUpdate(const struct can_frame & in) noexcept
{
  namespace yycdf = data_frame_handler;
  const auto t = yycdf::getFrameType(in);
  struct Entry
  {
    yycdf::DataFrameType type;
    UpdateKind kind;
    bool (CyberGear::*fn)(const struct can_frame &, bool) noexcept;
  };
  // clang-format off
  static constexpr Entry table[] = {
    {
      yycdf::DataFrameType::Type2_Status,
      UpdateKind::Status,
      &CyberGear::updateFromStatusFrame
    },
    {
      yycdf::DataFrameType::Type0_DeviceId,
      UpdateKind::DeviceId,
      &CyberGear::updateFromDeviceIdResp
    },
    {
      yycdf::DataFrameType::Type21_FaultWarning,
      UpdateKind::FaultWarning,
      &CyberGear::updateFromFaultWarningResp
    },
    {
      yycdf::DataFrameType::Type17_ReadParam,
      UpdateKind::ReadParam,
      &CyberGear::updateFromReadParamResp
    },
  };
  // clang-format on
  for (const auto & e : table) {
    if (e.type == t) {
      return (this->*(e.fn))(in, false) ? e.kind : UpdateKind::Ignored;
    }
  }
  return UpdateKind::None;
}

}  // namespace yy_cybergear
