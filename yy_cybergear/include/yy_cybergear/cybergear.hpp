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
#include <unordered_set>
#include <vector>

#include "yy_cybergear/protocol_types.hpp"

namespace yy_cybergear
{

// Lightweight builder that holds Host/Motor IDs and constructs CyberGear V2 CAN frames.
// This class wraps the free-function builders in data_frame_handler and provides
// per-parameter get/set helpers for convenience.
class CyberGear
{
public:
  // Backward-compatible alias
  using RunMode = yy_cybergear::RunMode;
  explicit CyberGear(uint8_t host_id, uint8_t motor_id) noexcept
  : host_id_(host_id), motor_id_(motor_id)
  {
  }

  // Non-copyable due to mutex member
  CyberGear(const CyberGear &) = delete;
  CyberGear & operator=(const CyberGear &) = delete;

  // Move-constructible/assignable to allow storage in std::vector
  CyberGear(CyberGear && other) noexcept
  {
    // It's safe to copy raw fields while holding other's lock
    std::lock_guard<std::mutex> lk(other.mu_);
    host_id_ = other.host_id_;
    motor_id_ = other.motor_id_;

    angle_rad_ = other.angle_rad_;
    vel_rad_s_ = other.vel_rad_s_;
    torque_Nm_ = other.torque_Nm_;
    temperature_c_ = other.temperature_c_;
    motor_can_id_ = other.motor_can_id_;
    status_mode_ = other.status_mode_;
    status_fault_bits_ = other.status_fault_bits_;
    raw_eff_id_ = other.raw_eff_id_;

    fault_bits_agg_ = other.fault_bits_agg_;
    warning_bits_agg_ = other.warning_bits_agg_;

    uid_ = other.uid_;

    run_mode_ = other.run_mode_;
    iq_reference_ = other.iq_reference_;
    speed_reference_ = other.speed_reference_;
    torque_limit_ = other.torque_limit_;
    current_kp_ = other.current_kp_;
    current_ki_ = other.current_ki_;
    current_filter_gain_ = other.current_filter_gain_;
    position_reference_ = other.position_reference_;
    speed_limit_ = other.speed_limit_;
    current_limit_ = other.current_limit_;
    mechanical_position_ = other.mechanical_position_;
    iq_filter_ = other.iq_filter_;
    mechanical_velocity_ = other.mechanical_velocity_;
    bus_voltage_ = other.bus_voltage_;
    rotation_turns_ = other.rotation_turns_;
    position_kp_ = other.position_kp_;
    speed_kp_ = other.speed_kp_;
    speed_ki_ = other.speed_ki_;
  }

  CyberGear & operator=(CyberGear && other) noexcept
  {
    if (this != &other) {
      std::scoped_lock lk(mu_, other.mu_);
      host_id_ = other.host_id_;
      motor_id_ = other.motor_id_;

      angle_rad_ = other.angle_rad_;
      vel_rad_s_ = other.vel_rad_s_;
      torque_Nm_ = other.torque_Nm_;
      temperature_c_ = other.temperature_c_;
      motor_can_id_ = other.motor_can_id_;
      status_mode_ = other.status_mode_;
      status_fault_bits_ = other.status_fault_bits_;
      raw_eff_id_ = other.raw_eff_id_;

      fault_bits_agg_ = other.fault_bits_agg_;
      warning_bits_agg_ = other.warning_bits_agg_;

      uid_ = other.uid_;

      run_mode_ = other.run_mode_;
      iq_reference_ = other.iq_reference_;
      speed_reference_ = other.speed_reference_;
      torque_limit_ = other.torque_limit_;
      current_kp_ = other.current_kp_;
      current_ki_ = other.current_ki_;
      current_filter_gain_ = other.current_filter_gain_;
      position_reference_ = other.position_reference_;
      speed_limit_ = other.speed_limit_;
      current_limit_ = other.current_limit_;
      mechanical_position_ = other.mechanical_position_;
      iq_filter_ = other.iq_filter_;
      mechanical_velocity_ = other.mechanical_velocity_;
      bus_voltage_ = other.bus_voltage_;
      rotation_turns_ = other.rotation_turns_;
      position_kp_ = other.position_kp_;
      speed_kp_ = other.speed_kp_;
      speed_ki_ = other.speed_ki_;
    }
    return *this;
  }

  // IDs
  uint8_t host_id() const noexcept { return host_id_; }
  uint8_t motor_id() const noexcept { return motor_id_; }
  void set_host_id(uint8_t v) noexcept { host_id_ = v; }
  void set_motor_id(uint8_t v) noexcept { motor_id_ = v; }

  // ===== Basic requests =====
  void buildGetDeviceId(struct can_frame & out) const noexcept;
  void buildEnable(struct can_frame & out) const noexcept;
  void buildStop(struct can_frame & out) const noexcept;
  void buildClearFaults(struct can_frame & out) const noexcept;
  void buildSetMechanicalZero(struct can_frame & out) const noexcept;
  void buildChangeMotorId(uint8_t new_motor_id, struct can_frame & out) const noexcept;
  void buildOpControl(const OpCommand & cmd, struct can_frame & out) const noexcept;
  void buildSetBaudRate(uint8_t code, struct can_frame & out) const noexcept;

  // ===== Generic parameter get/set =====
  void buildReadParam(uint16_t index, struct can_frame & out) const noexcept;
  void buildWriteParam(
    uint16_t index, const std::array<uint8_t, 4> & data, struct can_frame & out) const noexcept;

  // ===== Per-parameter helpers =====
  // RUN_MODE (0x7005): Control mode enum (0 OperationControl, 1 Position, 2 Speed, 3 Current)
  void buildGetRunMode(struct can_frame & out) const noexcept { buildReadParam(RUN_MODE, out); }
  void buildSetRunMode(RunMode mode, struct can_frame & out) const noexcept;

  // IQ_REFERENCE (A)
  void buildGetIqReference(struct can_frame & out) const noexcept
  {
    buildReadParam(IQ_REFERENCE, out);
  }
  void buildSetIqReference(float ampere, struct can_frame & out) const noexcept;

  // SPEED_REFERENCE (rad/s)
  void buildGetSpeedReference(struct can_frame & out) const noexcept
  {
    buildReadParam(SPEED_REFERENCE, out);
  }
  void buildSetSpeedReference(float rad_s, struct can_frame & out) const noexcept;

  // TORQUE_LIMIT (Nm)
  void buildGetTorqueLimit(struct can_frame & out) const noexcept
  {
    buildReadParam(TORQUE_LIMIT, out);
  }
  void buildSetTorqueLimit(float nm, struct can_frame & out) const noexcept;

  // CURRENT loop gains / filter
  void buildGetCurrentKp(struct can_frame & out) const noexcept
  {
    buildReadParam(CURRENT_KP, out);
  }
  void buildSetCurrentKp(float v, struct can_frame & out) const noexcept;
  void buildGetCurrentKi(struct can_frame & out) const noexcept
  {
    buildReadParam(CURRENT_KI, out);
  }
  void buildSetCurrentKi(float v, struct can_frame & out) const noexcept;
  void buildGetCurrentFilterGain(struct can_frame & out) const noexcept
  {
    buildReadParam(CURRENT_FILTER_GAIN, out);
  }
  void buildSetCurrentFilterGain(float v, struct can_frame & out) const noexcept;

  // Position/Speed/Current references and limits
  void buildGetPositionReference(struct can_frame & out) const noexcept
  {
    buildReadParam(POSITION_REFERENCE, out);
  }
  void buildSetPositionReference(float rad, struct can_frame & out) const noexcept;
  void buildGetSpeedLimit(struct can_frame & out) const noexcept
  {
    buildReadParam(SPEED_LIMIT, out);
  }
  void buildSetSpeedLimit(float rad_s, struct can_frame & out) const noexcept;
  void buildGetCurrentLimit(struct can_frame & out) const noexcept
  {
    buildReadParam(CURRENT_LIMIT, out);
  }
  void buildSetCurrentLimit(float ampere, struct can_frame & out) const noexcept;

  // Telemetry values (also provide set in case firmware allows writing)
  void buildGetMechanicalPosition(struct can_frame & out) const noexcept
  {
    buildReadParam(MECHANICAL_POSITION, out);
  }
  void buildSetMechanicalPosition(float rad, struct can_frame & out) const noexcept;
  void buildGetIqFilter(struct can_frame & out) const noexcept { buildReadParam(IQ_FILTER, out); }
  void buildSetIqFilter(float ampere, struct can_frame & out) const noexcept;
  void buildGetMechanicalVelocity(struct can_frame & out) const noexcept
  {
    buildReadParam(MECHANICAL_VELOCITY, out);
  }
  void buildSetMechanicalVelocity(float rad_s, struct can_frame & out) const noexcept;
  void buildGetBusVoltage(struct can_frame & out) const noexcept
  {
    buildReadParam(BUS_VOLTAGE, out);
  }
  void buildSetBusVoltage(float volt, struct can_frame & out) const noexcept;
  void buildGetRotationTurns(struct can_frame & out) const noexcept
  {
    buildReadParam(ROTATION_TURNS, out);
  }
  void buildSetRotationTurns(int16_t turns, struct can_frame & out) const noexcept;

  // Position/Speed loop gains
  void buildGetPositionKp(struct can_frame & out) const noexcept
  {
    buildReadParam(POSITION_KP, out);
  }
  void buildSetPositionKp(float v, struct can_frame & out) const noexcept;
  void buildGetSpeedKp(struct can_frame & out) const noexcept { buildReadParam(SPEED_KP, out); }
  void buildSetSpeedKp(float v, struct can_frame & out) const noexcept;
  void buildGetSpeedKi(struct can_frame & out) const noexcept { buildReadParam(SPEED_KI, out); }
  void buildSetSpeedKi(float v, struct can_frame & out) const noexcept;

  // ===== Frame-driven updaters (no existing structs used) =====
  // Parse and update internal status fields from a status frame (type==2)
  [[nodiscard]] bool updateFromStatusFrame(
    const struct can_frame & in, bool type_check = true) noexcept;
  // Parse and update UID from device-id response (type==0, low8==0xFE); uses motor_id_
  [[nodiscard]] bool updateFromDeviceIdResp(
    const struct can_frame & in, bool type_check = true) noexcept;
  // Parse and update fault/warning snapshot from type==21 response (uses host_id_)
  [[nodiscard]] bool updateFromFaultWarningResp(
    const struct can_frame & in, bool type_check = true) noexcept;
  // Parse and update any known parameter value from type==17 read-param response (uses host_id_)
  [[nodiscard]] bool updateFromReadParamResp(
    const struct can_frame & in, bool type_check = true) noexcept;

  // Dispatch a received frame by its type and update internal state accordingly.
  // Returns which kind of update was applied (or None if not applicable).
  enum class UpdateKind : uint8_t { None = 0, DeviceId, Status, FaultWarning, ReadParam, Ignored };
  [[nodiscard]] UpdateKind dispatchAndUpdate(const struct can_frame & in) noexcept;

  // ===== Status getters =====
  // Thread-safe snapshot of status fields, using the shared Status type
  [[nodiscard]] Status getStatus() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return Status{angle_rad_,    vel_rad_s_,   torque_Nm_,         temperature_c_,
                  motor_can_id_, status_mode_, status_fault_bits_, raw_eff_id_};
  }

  // Backward-compatible per-field getters (thread-safe)
  float angle_rad() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return angle_rad_;
  }
  float vel_rad_s() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return vel_rad_s_;
  }
  float torque_Nm() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return torque_Nm_;
  }
  float temperature_c() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return temperature_c_;
  }
  uint8_t motor_can_id() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return motor_can_id_;
  }
  Status::StatusMode status_mode() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return status_mode_;
  }
  uint8_t fault_bits() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return status_fault_bits_;
  }
  uint32_t raw_eff_id() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return raw_eff_id_;
  }

  // ===== Fault/Warning getters =====
  uint32_t faults_bits() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return fault_bits_agg_;
  }
  uint32_t warnings_bits() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return warning_bits_agg_;
  }

  // New descriptive accessors (non-breaking: keep old ones above)
  uint32_t fault_bits_agg() const noexcept { return faults_bits(); }
  uint32_t warning_bits_agg() const noexcept { return warnings_bits(); }

  // ===== UID getter =====
  std::array<uint8_t, yy_cybergear::can_dlc::DeviceIdResp> uid() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return uid_;
  }
  bool isUidInitialized() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return uid_initialized_;
  }

  // ===== Parameter getters (mirror) =====
  RunMode run_mode() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return run_mode_;
  }
  float iq_reference() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return iq_reference_;
  }
  float speed_reference() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_reference_;
  }
  float torque_limit() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return torque_limit_;
  }
  float current_kp() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_kp_;
  }
  float current_ki() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_ki_;
  }
  float current_filter_gain() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_filter_gain_;
  }
  float position_reference() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return position_reference_;
  }
  float speed_limit() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_limit_;
  }
  float current_limit() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return current_limit_;
  }
  float mechanical_position() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return mechanical_position_;
  }
  float iq_filter() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return iq_filter_;
  }
  float mechanical_velocity() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return mechanical_velocity_;
  }
  float bus_voltage() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return bus_voltage_;
  }
  int16_t rotation_turns() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return rotation_turns_;
  }
  float position_kp() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return position_kp_;
  }
  float speed_kp() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_kp_;
  }
  float speed_ki() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_ki_;
  }

  // ===== Initialization flags helpers =====
  // True after the first status frame has been applied
  [[nodiscard]] bool isStatusInitialized() const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return status_initialized_;
  }
  // True if a given parameter index has been received at least once via ReadParam response
  [[nodiscard]] bool isParamInitialized(uint16_t index) const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    return initialized_params_.find(index) != initialized_params_.end();
  }
  // Convenience: check a list of indices; optionally require UID as well
  [[nodiscard]] bool isInitializedFor(
    const std::vector<uint16_t> & indices, bool require_uid = true) const noexcept
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (require_uid && !uid_initialized_) return false;
    for (uint16_t idx : indices) {
      if (initialized_params_.find(idx) == initialized_params_.end()) return false;
    }
    return true;
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
  Status::StatusMode status_mode_{Status::StatusMode::Reset};
  uint8_t status_fault_bits_{0};
  uint32_t raw_eff_id_{0};

  // Fault/Warning snapshot
  uint32_t fault_bits_agg_{0};
  uint32_t warning_bits_agg_{0};

  // UID
  std::array<uint8_t, yy_cybergear::can_dlc::DeviceIdResp> uid_{};
  bool uid_initialized_{false};

  // Parameter mirrors
  RunMode run_mode_{RunMode::OperationControl};
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

  // Initialization bookkeeping
  bool status_initialized_{false};
  std::unordered_set<uint16_t> initialized_params_{};  // indices seen in ReadParam responses

  // Local helpers
  static constexpr float kPi_ = 3.14159265358979323846f;
};

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__CYBERGEAR_V2_HPP_
