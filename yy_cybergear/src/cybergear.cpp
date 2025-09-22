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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "yy_cybergear/data_frame_handler.hpp"

namespace yy_cybergear
{
// Short alias for data frame handler namespace
namespace dfh = data_frame_handler;

namespace
{
std::string hex2(uint32_t v)
{
  std::ostringstream os;
  os << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (v & 0xFFu);
  return os.str();
}

std::string hex8(uint32_t v)
{
  std::ostringstream os;
  os << std::uppercase << std::hex << std::setw(8) << std::setfill('0') << (v & CAN_EFF_MASK);
  return os.str();
}
}  // namespace

CyberGear::CyberGear(
  std::string ifname, uint8_t host_id, uint8_t motor_id, bool verbose, int rcvbuf_bytes,
  int sndbuf_bytes) noexcept
: can_(ifname, false, false, rcvbuf_bytes, sndbuf_bytes),
  ifname_(std::move(ifname)),
  host_id_(host_id),
  motor_id_(motor_id),
  verbose_(verbose),
  rcvbuf_bytes_(rcvbuf_bytes),
  sndbuf_bytes_(sndbuf_bytes)
{
}

void CyberGear::open()
{
  can_.open();
  can_.setLoopback(false);
  can_.setRecvOwnMsgs(false);
}

void CyberGear::close() noexcept { can_.close(); }

void CyberGear::debugPrintFrame(const struct can_frame & f) const
{
  if (!verbose_) return;
  const bool eff = data_frame_handler::isExtended(f.can_id);
  const uint32_t id = eff ? (f.can_id & CAN_EFF_MASK) : (f.can_id & CAN_SFF_MASK);
  std::ostringstream os;
  os << "can " << (eff ? "EFF" : "SFF") << " id=0x" << std::uppercase << std::hex
     << std::setw(eff ? 8 : 3) << std::setfill('0') << id << std::dec
     << " dlc=" << static_cast<int>(f.can_dlc) << " data:";
  os << std::uppercase << std::hex << std::setfill('0');
  for (int i = 0; i < f.can_dlc; ++i) {
    os << ' ' << std::setw(2) << static_cast<unsigned>(f.data[i]);
  }
  std::cout << os.str() << '\n';
}

Result<std::array<uint8_t, kUidLen>> CyberGear::getMcuId(int timeout_ms)
{
  if (!can_.isOpen()) return Result<std::array<uint8_t, kUidLen>>::failure(ErrorCode::NotOpen);

  struct can_frame tx
  {
  };
  dfh::buildGetDeviceIdReq(host_id_, motor_id_, tx);

  if (verbose_) {
    std::cout << "Sending request: if=" << ifname_ << " host_id=0x" << hex2(host_id_)
              << " motor_id=0x" << hex2(motor_id_) << " can_id=0x" << hex8(tx.can_id) << '\n';
  }
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    std::array<uint8_t, kUidLen> uid{};
    if (dfh::parseDeviceIdResp(rx, motor_id_, uid)) {
      if (verbose_) debugPrintFrame(rx);
      return Result<std::array<uint8_t, kUidLen>>::success(uid);
    }
  }
  return Result<std::array<uint8_t, kUidLen>>::failure(ErrorCode::Timeout);
}

Result<Status> CyberGear::sendOperationCommand(const OpCommand & cmd, int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  struct can_frame tx
  {
  };
  dfh::buildOpCtrlReq(motor_id_, cmd, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 50)) continue;
    Status st{};
    if (dfh::parseStatus(rx, st)) {
      if (verbose_) debugPrintFrame(rx);
      if (st.motor_can_id == motor_id_) return Result<Status>::success(st);
    }
  }
  return Result<Status>::failure(ErrorCode::Timeout);
}

Result<Status> CyberGear::clearFaults(int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  struct can_frame tx
  {
  };
  dfh::buildClearFaultsReq(host_id_, motor_id_, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    Status st{};
    if (dfh::parseStatus(rx, st)) {
      if (verbose_) debugPrintFrame(rx);
      if (st.motor_can_id == motor_id_) return Result<Status>::success(st);
    }
  }
  return Result<Status>::failure(ErrorCode::Timeout);
}

Result<Status> CyberGear::stopMotor(int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  struct can_frame tx
  {
  };
  dfh::buildStopReq(host_id_, motor_id_, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    Status st{};
    if (dfh::parseStatus(rx, st)) {
      if (verbose_) debugPrintFrame(rx);
      if (st.motor_can_id == motor_id_) return Result<Status>::success(st);
    }
  }
  return Result<Status>::failure(ErrorCode::Timeout);
}

Result<Status> CyberGear::enableMotor(int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  struct can_frame tx
  {
  };
  dfh::buildEnableReq(host_id_, motor_id_, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);

  // After enabling, the motor should respond with a type 2 status frame.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    Status st{};
    if (dfh::parseStatus(rx, st)) {
      if (verbose_) debugPrintFrame(rx);
      if (st.motor_can_id == motor_id_) return Result<Status>::success(st);
    }
  }
  return Result<Status>::failure(ErrorCode::Timeout);
}

Result<Status> CyberGear::setMechanicalZero(int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  struct can_frame tx
  {
  };
  dfh::buildSetMechanicalZeroReq(host_id_, motor_id_, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    Status st{};
    if (dfh::parseStatus(rx, st)) {
      if (verbose_) debugPrintFrame(rx);
      return Result<Status>::success(st);
    }
  }
  return Result<Status>::failure(ErrorCode::Timeout);
}

Result<std::array<uint8_t, kUidLen>> CyberGear::changeMotorId(
  uint8_t new_motor_id, int timeout_ms) noexcept
{
  if (!can_.isOpen()) return Result<std::array<uint8_t, kUidLen>>::failure(ErrorCode::NotOpen);
  struct can_frame tx
  {
  };
  dfh::buildChangeMotorIdReq(host_id_, motor_id_, new_motor_id, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);
  // Wait for communication type 0 response that includes the new motor ID.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    std::array<uint8_t, kUidLen> uid{};
    if (dfh::parseDeviceIdResp(rx, new_motor_id, uid)) {
      if (verbose_) debugPrintFrame(rx);
      motor_id_ = new_motor_id;
      return Result<std::array<uint8_t, kUidLen>>::success(uid);
    }
  }
  return Result<std::array<uint8_t, kUidLen>>::failure(ErrorCode::Timeout);
}

// NOTE: Baud rate change is disabled intentionally.
// Please refer to the document process to modify it carefully.
// Operation errors may cause problems such as being unable to connect to
// the motor and being unable to upgrade.
// Result<std::array<uint8_t, kUidLen>> CyberGear::setBaudRate(uint8_t code)
// {
//   if (!can_.isOpen()) return Result<std::array<uint8_t, kUidLen>>::failure(ErrorCode::NotOpen);
//   struct can_frame tx
//   {
//   };
//   dfh::buildSetBaudRateReq(host_id_, motor_id_, code, tx);
//   if (verbose_) debugPrintFrame(tx);
//   can_.send(tx);
//   const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
//   while (std::chrono::steady_clock::now() < deadline) {
//     struct can_frame rx
//     {
//     };
//     if (!can_.recv(rx, 100)) continue;
//     std::array<uint8_t, kUidLen> uid{};
//     if (dfh::parseDeviceIdResp(rx, motor_id_, uid)) {
//       if (verbose_) debugPrintFrame(rx);
//       return Result<std::array<uint8_t, kUidLen>>::success(uid);
//     }
//   }
//   return Result<std::array<uint8_t, kUidLen>>::failure(ErrorCode::Timeout);
// }

Result<std::array<uint8_t, 4>> CyberGear::readParamRaw(uint16_t index, int timeout_ms)
{
  if (!can_.isOpen()) return Result<std::array<uint8_t, 4>>::failure(ErrorCode::NotOpen);
  struct can_frame tx
  {
  };
  dfh::buildReadParamReq(host_id_, motor_id_, index, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    std::array<uint8_t, 4> out{};
    if (dfh::parseReadParamResp(rx, host_id_, index, out)) {
      if (verbose_) debugPrintFrame(rx);
      return Result<std::array<uint8_t, 4>>::success(out);
    }
  }
  return Result<std::array<uint8_t, 4>>::failure(ErrorCode::Timeout);
}

Result<Status> CyberGear::writeParamRaw(
  uint16_t index, const std::array<uint8_t, 4> & data, int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);

  struct can_frame tx
  {
  };
  dfh::buildWriteParamReq(host_id_, motor_id_, index, data, tx);
  if (verbose_) debugPrintFrame(tx);
  can_.send(tx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx
    {
    };
    if (!can_.recv(rx, 100)) continue;
    Status st{};
    if (dfh::parseStatus(rx, st)) {
      if (st.motor_can_id == motor_id_) {
        if (verbose_) debugPrintFrame(rx);
        return Result<Status>::success(st);
      }
    }
  }
  return Result<Status>::failure(ErrorCode::Timeout);
}

Result<float> CyberGear::readParamFloat(uint16_t index, int timeout_ms)
{
  auto rr = readParamRaw(index, timeout_ms);
  if (!rr.ok()) {
    return Result<float>::failure(rr.error().value());
  }
  float out = 0.0f;
  std::memcpy(&out, rr.value()->data(), 4);
  return Result<float>::success(out);
}

Result<Status> CyberGear::writeParamFloat(uint16_t index, float value, int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  std::array<uint8_t, 4> raw{};
  std::memcpy(raw.data(), &value, 4);
  return writeParamRaw(index, raw, timeout_ms);
}

Result<Status> CyberGear::setRunMode(RunMode mode, int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  std::array<uint8_t, 4> data{static_cast<uint8_t>(mode), 0, 0, 0};
  return writeParamRaw(RUN_MODE, data, timeout_ms);
}

Result<uint8_t> CyberGear::getRunMode(int timeout_ms)
{
  auto rr = readParamRaw(RUN_MODE, timeout_ms);
  if (!rr.ok()) return Result<uint8_t>::failure(rr.error().value());
  return Result<uint8_t>::success((*rr.value())[0]);
}

Result<Status> CyberGear::setIqReference(float amps, int timeout_ms)
{
  return writeParamFloat(IQ_REFERENCE, amps, timeout_ms);
}

Result<float> CyberGear::getIqReference(int timeout_ms)
{
  return readParamFloat(IQ_REFERENCE, timeout_ms);
}

Result<Status> CyberGear::setSpeedReference(float rad_s, int timeout_ms)
{
  return writeParamFloat(SPEED_REFERENCE, rad_s, timeout_ms);
}

Result<float> CyberGear::getSpeedReference(int timeout_ms)
{
  return readParamFloat(SPEED_REFERENCE, timeout_ms);
}

Result<Status> CyberGear::setTorqueLimit(float nm, int timeout_ms)
{
  return writeParamFloat(TORQUE_LIMIT, nm, timeout_ms);
}

Result<float> CyberGear::getTorqueLimit(int timeout_ms)
{
  return readParamFloat(TORQUE_LIMIT, timeout_ms);
}

Result<Status> CyberGear::setCurrentKp(float v, int timeout_ms)
{
  return writeParamFloat(CURRENT_KP, v, timeout_ms);
}

Result<float> CyberGear::getCurrentKp(int timeout_ms)
{
  return readParamFloat(CURRENT_KP, timeout_ms);
}

Result<Status> CyberGear::setCurrentKi(float v, int timeout_ms)
{
  return writeParamFloat(CURRENT_KI, v, timeout_ms);
}

Result<float> CyberGear::getCurrentKi(int timeout_ms)
{
  return readParamFloat(CURRENT_KI, timeout_ms);
}

Result<Status> CyberGear::setCurrentFilterGain(float v, int timeout_ms)
{
  return writeParamFloat(CURRENT_FILTER_GAIN, v, timeout_ms);
}

Result<float> CyberGear::getCurrentFilterGain(int timeout_ms)
{
  return readParamFloat(CURRENT_FILTER_GAIN, timeout_ms);
}

Result<Status> CyberGear::setPositionReference(float rad, int timeout_ms)
{
  return writeParamFloat(POSITION_REFERENCE, rad, timeout_ms);
}

Result<float> CyberGear::getPositionReference(int timeout_ms)
{
  return readParamFloat(POSITION_REFERENCE, timeout_ms);
}

Result<Status> CyberGear::setSpeedLimit(float rad_s, int timeout_ms)
{
  return writeParamFloat(SPEED_LIMIT, rad_s, timeout_ms);
}

Result<float> CyberGear::getSpeedLimit(int timeout_ms)
{
  return readParamFloat(SPEED_LIMIT, timeout_ms);
}

Result<Status> CyberGear::setCurrentLimit(float amps, int timeout_ms)
{
  return writeParamFloat(CURRENT_LIMIT, amps, timeout_ms);
}

Result<float> CyberGear::getCurrentLimit(int timeout_ms)
{
  return readParamFloat(CURRENT_LIMIT, timeout_ms);
}

Result<Status> CyberGear::setRotationTurns(int16_t turns, int timeout_ms)
{
  if (!can_.isOpen()) return Result<Status>::failure(ErrorCode::NotOpen);
  std::array<uint8_t, 4> raw{0, 0, 0, 0};
  std::memcpy(raw.data(), &turns, 2);
  return writeParamRaw(ROTATION_TURNS, raw, timeout_ms);
}

Result<int16_t> CyberGear::getRotationTurns(int timeout_ms)
{
  auto rr = readParamRaw(ROTATION_TURNS, timeout_ms);
  if (!rr.ok()) return Result<int16_t>::failure(rr.error().value());
  int16_t turns = 0;
  std::memcpy(&turns, rr.value()->data(), 2);
  return Result<int16_t>::success(turns);
}

Result<Status> CyberGear::setPositionKp(float v, int timeout_ms)
{
  return writeParamFloat(POSITION_KP, v, timeout_ms);
}

Result<float> CyberGear::getPositionKp(int timeout_ms)
{
  return readParamFloat(POSITION_KP, timeout_ms);
}

Result<Status> CyberGear::setSpeedKp(float v, int timeout_ms)
{
  return writeParamFloat(SPEED_KP, v, timeout_ms);
}

Result<float> CyberGear::getSpeedKp(int timeout_ms)
{
  return readParamFloat(SPEED_KP, timeout_ms);
}

Result<Status> CyberGear::setSpeedKi(float v, int timeout_ms)
{
  return writeParamFloat(SPEED_KI, v, timeout_ms);
}

Result<float> CyberGear::getSpeedKi(int timeout_ms) { return readParamFloat(SPEED_KI, timeout_ms); }

// The following parameters are read-only.
Result<float> CyberGear::getMechanicalPosition(int timeout_ms)
{
  return readParamFloat(MECHANICAL_POSITION, timeout_ms);
}

Result<float> CyberGear::getIqFilter(int timeout_ms)
{
  return readParamFloat(IQ_FILTER, timeout_ms);
}

Result<float> CyberGear::getMechanicalVelocity(int timeout_ms)
{
  return readParamFloat(MECHANICAL_VELOCITY, timeout_ms);
}

Result<float> CyberGear::getBusVoltage(int timeout_ms)
{
  return readParamFloat(BUS_VOLTAGE, timeout_ms);
}

}  // namespace yy_cybergear
