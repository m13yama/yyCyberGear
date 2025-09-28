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

#ifndef YY_CYBERGEAR__CYBERGEAR_V2_HPP_
#define YY_CYBERGEAR__CYBERGEAR_V2_HPP_

#include <linux/can.h>

#include <array>
#include <cstdint>
#include <mutex>

#include "yy_cybergear/protocol_types.hpp"

namespace yy_cybergear
{

// Lightweight builder that holds Host/Motor IDs and constructs CyberGear V2 CAN frames.
// This class wraps the free-function builders in data_frame_handler and provides
// per-parameter get/set helpers for convenience.
class CyberGear
{
public:
  explicit CyberGear(uint8_t host_id, uint8_t motor_id) : host_id_(host_id), motor_id_(motor_id) {}

  // IDs
  uint8_t host_id() const { return host_id_; }
  uint8_t motor_id() const { return motor_id_; }
  void set_host_id(uint8_t v) { host_id_ = v; }
  void set_motor_id(uint8_t v) { motor_id_ = v; }

  // ===== Basic requests =====
  void buildGetDeviceId(struct can_frame & out) const;
  void buildEnable(struct can_frame & out) const;
  void buildStop(struct can_frame & out) const;
  void buildClearFaults(struct can_frame & out) const;
  void buildSetMechanicalZero(struct can_frame & out) const;
  void buildChangeMotorId(uint8_t new_motor_id, struct can_frame & out) const;
  void buildOpControl(const OpCommand & cmd, struct can_frame & out) const;
  void buildFaultWarning(struct can_frame & out) const;  // request snapshot
  void buildSetBaudRate(uint8_t code, struct can_frame & out) const;

  // ===== Generic parameter get/set =====
  void buildReadParam(uint16_t index, struct can_frame & out) const;
  void buildWriteParam(
    uint16_t index, const std::array<uint8_t, 4> & data, struct can_frame & out) const;

  // ===== Per-parameter helpers =====
  // RUN_MODE (0x7005): 0 reset, 1 cali, 2 run
  void buildGetRunMode(struct can_frame & out) const { buildReadParam(RUN_MODE, out); }
  void buildSetRunMode(uint32_t mode, struct can_frame & out) const;

  // IQ_REFERENCE (A)
  void buildGetIqReference(struct can_frame & out) const { buildReadParam(IQ_REFERENCE, out); }
  void buildSetIqReference(float ampere, struct can_frame & out) const;

  // SPEED_REFERENCE (rad/s)
  void buildGetSpeedReference(struct can_frame & out) const
  {
    buildReadParam(SPEED_REFERENCE, out);
  }
  void buildSetSpeedReference(float rad_s, struct can_frame & out) const;

  // TORQUE_LIMIT (Nm)
  void buildGetTorqueLimit(struct can_frame & out) const { buildReadParam(TORQUE_LIMIT, out); }
  void buildSetTorqueLimit(float nm, struct can_frame & out) const;

  // CURRENT loop gains / filter
  void buildGetCurrentKp(struct can_frame & out) const { buildReadParam(CURRENT_KP, out); }
  void buildSetCurrentKp(float v, struct can_frame & out) const;
  void buildGetCurrentKi(struct can_frame & out) const { buildReadParam(CURRENT_KI, out); }
  void buildSetCurrentKi(float v, struct can_frame & out) const;
  void buildGetCurrentFilterGain(struct can_frame & out) const
  {
    buildReadParam(CURRENT_FILTER_GAIN, out);
  }
  void buildSetCurrentFilterGain(float v, struct can_frame & out) const;

  // Position/Speed/Current references and limits
  void buildGetPositionReference(struct can_frame & out) const
  {
    buildReadParam(POSITION_REFERENCE, out);
  }
  void buildSetPositionReference(float rad, struct can_frame & out) const;
  void buildGetSpeedLimit(struct can_frame & out) const { buildReadParam(SPEED_LIMIT, out); }
  void buildSetSpeedLimit(float rad_s, struct can_frame & out) const;
  void buildGetCurrentLimit(struct can_frame & out) const { buildReadParam(CURRENT_LIMIT, out); }
  void buildSetCurrentLimit(float ampere, struct can_frame & out) const;

  // Telemetry values (also provide set in case firmware allows writing)
  void buildGetMechanicalPosition(struct can_frame & out) const
  {
    buildReadParam(MECHANICAL_POSITION, out);
  }
  void buildSetMechanicalPosition(float rad, struct can_frame & out) const;
  void buildGetIqFilter(struct can_frame & out) const { buildReadParam(IQ_FILTER, out); }
  void buildSetIqFilter(float ampere, struct can_frame & out) const;
  void buildGetMechanicalVelocity(struct can_frame & out) const
  {
    buildReadParam(MECHANICAL_VELOCITY, out);
  }
  void buildSetMechanicalVelocity(float rad_s, struct can_frame & out) const;
  void buildGetBusVoltage(struct can_frame & out) const { buildReadParam(BUS_VOLTAGE, out); }
  void buildSetBusVoltage(float volt, struct can_frame & out) const;
  void buildGetRotationTurns(struct can_frame & out) const { buildReadParam(ROTATION_TURNS, out); }
  void buildSetRotationTurns(int16_t turns, struct can_frame & out) const;

  // Position/Speed loop gains
  void buildGetPositionKp(struct can_frame & out) const { buildReadParam(POSITION_KP, out); }
  void buildSetPositionKp(float v, struct can_frame & out) const;
  void buildGetSpeedKp(struct can_frame & out) const { buildReadParam(SPEED_KP, out); }
  void buildSetSpeedKp(float v, struct can_frame & out) const;
  void buildGetSpeedKi(struct can_frame & out) const { buildReadParam(SPEED_KI, out); }
  void buildSetSpeedKi(float v, struct can_frame & out) const;

  // ===== Frame-driven updaters (no existing structs used) =====
  // Parse and update internal status fields from a status frame (type==2)
  bool updateFromStatusFrame(const struct can_frame & in, bool type_check = true);
  // Parse and update UID from device-id response (type==0, low8==0xFE); uses motor_id_
  bool updateFromDeviceIdResp(const struct can_frame & in, bool type_check = true);
  // Parse and update fault/warning snapshot from type==21 response (uses host_id_)
  bool updateFromFaultWarningResp(const struct can_frame & in, bool type_check = true);
  // Parse and update any known parameter value from type==17 read-param response (uses host_id_)
  bool updateFromReadParamResp(const struct can_frame & in, bool type_check = true);

  // Dispatch a received frame by its type and update internal state accordingly.
  // Returns which kind of update was applied (or None if not applicable).
  enum class UpdateKind : uint8_t { None = 0, DeviceId, Status, FaultWarning, ReadParam, Ignored };
  UpdateKind dispatchAndUpdate(const struct can_frame & in);

  // ===== Status getters =====
  struct StatusSnapshot
  {
    float angle_rad{0.0f};
    float vel_rad_s{0.0f};
    float torque_Nm{0.0f};
    float temperature_c{0.0f};
    uint8_t motor_can_id{0};
    uint8_t mode{0};
    uint8_t fault_bits{0};
    uint32_t raw_eff_id{0};
  };

  // Thread-safe snapshot of status fields
  StatusSnapshot getStatus() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return StatusSnapshot{angle_rad_,    vel_rad_s_, torque_Nm_,  temperature_c_,
                          motor_can_id_, mode_,      fault_bits_, raw_eff_id_};
  }

  // Backward-compatible per-field getters (thread-safe)
  float angle_rad() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return angle_rad_;
  }
  float vel_rad_s() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return vel_rad_s_;
  }
  float torque_Nm() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return torque_Nm_;
  }
  float temperature_c() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return temperature_c_;
  }
  uint8_t motor_can_id() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return motor_can_id_;
  }
  uint8_t mode() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return mode_;
  }
  uint8_t fault_bits() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return fault_bits_;
  }
  uint32_t raw_eff_id() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return raw_eff_id_;
  }

  // ===== Fault/Warning getters =====
  uint32_t faults_bits() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return faults_bits_;
  }
  uint32_t warnings_bits() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return warnings_bits_;
  }

  // ===== UID getter =====
  std::array<uint8_t, kUidLen> uid() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return uid_;
  }

  // ===== Parameter getters (mirror) =====
  uint32_t run_mode() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return run_mode_;
  }
  float iq_reference() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return iq_reference_;
  }
  float speed_reference() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_reference_;
  }
  float torque_limit() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return torque_limit_;
  }
  float current_kp() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_kp_;
  }
  float current_ki() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_ki_;
  }
  float current_filter_gain() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_filter_gain_;
  }
  float position_reference() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return position_reference_;
  }
  float speed_limit() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_limit_;
  }
  float current_limit() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_limit_;
  }
  float mechanical_position() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return mechanical_position_;
  }
  float iq_filter() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return iq_filter_;
  }
  float mechanical_velocity() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return mechanical_velocity_;
  }
  float bus_voltage() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return bus_voltage_;
  }
  int16_t rotation_turns() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return rotation_turns_;
  }
  float position_kp() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return position_kp_;
  }
  float speed_kp() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_kp_;
  }
  float speed_ki() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_ki_;
  }

private:
  // Synchronization for concurrent readers/writers (RX thread vs user thread)
  mutable std::mutex mu_;

  // IDs
  uint8_t host_id_{};
  uint8_t motor_id_{};

  // Status mirror (from status frames)
  float angle_rad_{0.0f};
  float vel_rad_s_{0.0f};
  float torque_Nm_{0.0f};
  float temperature_c_{0.0f};
  uint8_t motor_can_id_{0};
  uint8_t mode_{0};
  uint8_t fault_bits_{0};
  uint32_t raw_eff_id_{0};

  // Fault/Warning snapshot
  uint32_t faults_bits_{0};
  uint32_t warnings_bits_{0};

  // UID
  std::array<uint8_t, kUidLen> uid_{};

  // Parameter mirrors
  uint32_t run_mode_{0};
  float iq_reference_{0.0f};
  float speed_reference_{0.0f};
  float torque_limit_{0.0f};
  float current_kp_{0.0f};
  float current_ki_{0.0f};
  float current_filter_gain_{0.0f};
  float position_reference_{0.0f};
  float speed_limit_{0.0f};
  float current_limit_{0.0f};
  float mechanical_position_{0.0f};
  float iq_filter_{0.0f};
  float mechanical_velocity_{0.0f};
  float bus_voltage_{0.0f};
  int16_t rotation_turns_{0};
  float position_kp_{0.0f};
  float speed_kp_{0.0f};
  float speed_ki_{0.0f};

  // Local helpers
  static constexpr float kPi_ = 3.14159265358979323846f;
};

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__CYBERGEAR_V2_HPP_
