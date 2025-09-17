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

#ifndef YY_CYBERGEAR__CYBERGEAR_HPP_
#define YY_CYBERGEAR__CYBERGEAR_HPP_

#include <linux/can.h>

#include <array>
#include <cstdint>
#include <string>

#include "yy_cybergear/data_frame_codec.hpp"
#include "yy_cybergear/protocol_types.hpp"
#include "yy_cybergear/result.hpp"
#include "yy_socket_can/socket_can.hpp"

namespace yy_cybergear
{

// SocketCAN-based CyberGear driver.
class CyberGear
{
public:
  static constexpr auto kUidLen = yy_cybergear::kUidLen;
  static constexpr int kDefaultTimeoutMs = 200;

  explicit CyberGear(
    std::string ifname = "can0", uint8_t host_id = 0x01, uint8_t motor_id = 0x01,
    bool verbose = false, int rcvbuf_bytes = -1, int sndbuf_bytes = -1) noexcept;

  CyberGear(const CyberGear &) = delete;
  CyberGear & operator=(const CyberGear &) = delete;

  CyberGear(CyberGear &&) noexcept = default;
  CyberGear & operator=(CyberGear &&) noexcept = default;

  ~CyberGear() = default;

  void open();
  void close() noexcept;
  [[nodiscard]] bool isOpen() const noexcept { return can_.isOpen(); }

  const std::string & ifname() const noexcept { return ifname_; }
  uint8_t hostId() const noexcept { return host_id_; }
  uint8_t motorId() const noexcept { return motor_id_; }
  void setHostId(uint8_t id) noexcept { host_id_ = id; }
  void setMotorId(uint8_t id) noexcept { motor_id_ = id; }
  void setVerbose(bool v) noexcept { verbose_ = v; }
  bool verbose() const noexcept { return verbose_; }

  // MCU UID (8 bytes). Returns UID on success.
  // Timeout is specified in milliseconds for consistency across the API.
  [[nodiscard]] Result<std::array<uint8_t, kUidLen>> getMcuId(int timeout_ms = kDefaultTimeoutMs);

  // Run mode for 0x7005 parameter
  enum class RunMode : uint8_t { Operation = 0, Position = 1, Speed = 2, Current = 3 };

  // Send op command (type 1). Torque in CAN ID bits 23..8. Returns type-2 status on success.
  [[nodiscard]] Result<Status> sendOperationCommand(
    const OpCommand & cmd, int timeout_ms = kDefaultTimeoutMs);

  // ========= Maintenance / control commands =========
  // Clear faults (type 4, Byte[0]=1). Returns first type-2 status on success.
  [[nodiscard]] Result<Status> clearFaults(int timeout_ms = kDefaultTimeoutMs);

  // Stop/reset motor (type 4). Returns first type-2 status on success.
  [[nodiscard]] Result<Status> stopMotor(int timeout_ms = kDefaultTimeoutMs);

  // Enable motor (type 3). Returns first type-2 status on success.
  [[nodiscard]] Result<Status> enableMotor(int timeout_ms = kDefaultTimeoutMs);

  // Set mechanical zero (type 6, Byte[0]=1). Returns first type-2 status on success.
  [[nodiscard]] Result<Status> setMechanicalZero(int timeout_ms = kDefaultTimeoutMs);

  // Set motor CAN_ID immediately on device (type 7). Returns device UID on success.
  // Local `motor_id_` is updated only on success.
  [[nodiscard]] Result<std::array<uint8_t, kUidLen>> changeMotorId(
    uint8_t new_motor_id, int timeout_ms = kDefaultTimeoutMs) noexcept;

  // Request fault/warning snapshot (type 21). Returns snapshot on success.
  [[nodiscard]] Result<FaultWarning> requestFaultWarning(int timeout_ms = kDefaultTimeoutMs);

  // NOTE: Baud rate change is disabled intentionally.
  // Please refer to the document process to modify it carefully.
  // Operation errors may cause problems such as being unable to connect to
  // the motor and being unable to upgrade.
  //  Baud rate change (type 22).
  //  [[nodiscard]] Result<std::array<uint8_t, kUidLen>> setBaudRate(uint8_t code);

  // ========= Generic parameter R/W (type 17/18) =========
  [[nodiscard]] Result<std::array<uint8_t, 4>> readParamRaw(
    uint16_t index, int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> writeParamRaw(
    uint16_t index, const std::array<uint8_t, 4> & data, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> readParamFloat(uint16_t index, int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> writeParamFloat(
    uint16_t index, float value, int timeout_ms = kDefaultTimeoutMs);

  // Convenience setters via parameter indices (4.2)
  [[nodiscard]] Result<uint8_t> getRunMode(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setRunMode(RunMode mode, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getIqReference(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setIqReference(float amps, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getSpeedReference(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setSpeedReference(float rad_s, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getTorqueLimit(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setTorqueLimit(float nm, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getCurrentKp(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setCurrentKp(float v, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getCurrentKi(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setCurrentKi(float v, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getCurrentFilterGain(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setCurrentFilterGain(float v, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getPositionReference(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setPositionReference(float rad, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getSpeedLimit(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setSpeedLimit(float rad_s, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getCurrentLimit(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setCurrentLimit(float amps, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<int16_t> getRotationTurns(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setRotationTurns(int16_t turns, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getPositionKp(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setPositionKp(float v, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getSpeedKp(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setSpeedKp(float v, int timeout_ms = kDefaultTimeoutMs);

  [[nodiscard]] Result<float> getSpeedKi(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<Status> setSpeedKi(float v, int timeout_ms = kDefaultTimeoutMs);

  // The following are read-only parameters
  [[nodiscard]] Result<float> getMechanicalPosition(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<float> getIqFilter(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<float> getMechanicalVelocity(int timeout_ms = kDefaultTimeoutMs);
  [[nodiscard]] Result<float> getBusVoltage(int timeout_ms = kDefaultTimeoutMs);

private:
  yy_socket_can::SocketCAN can_;
  std::string ifname_{};
  uint8_t host_id_{0x01};
  uint8_t motor_id_{0x01};
  bool verbose_{false};
  int rcvbuf_bytes_{-1};
  int sndbuf_bytes_{-1};

  void debugPrintFrame(const struct can_frame & f) const;
};

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__CYBERGEAR_HPP_
