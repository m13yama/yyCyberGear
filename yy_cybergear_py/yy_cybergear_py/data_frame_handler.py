"""Pure helper functions for building/parsing CyberGear CAN frames."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional

from .protocol_types import FaultWarning, OpCommand, Status, kUidLen

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF


@dataclass(slots=True)
class CanFrame:
    """Minimal representation of a SocketCAN data frame."""

    can_id: int
    data: bytes = b''
    dlc: int = 0

    def __post_init__(self) -> None:
        if self.dlc < 0 or self.dlc > 8:
            raise ValueError('can_dlc must be in [0, 8]')
        if len(self.data) < self.dlc:
            raise ValueError('data payload shorter than DLC')
        if len(self.data) > 8:
            raise ValueError('data payload longer than 8 bytes')


def build_eff_id(frame_type: int, host_id: int, motor_id: int) -> int:
    return ((frame_type & 0x1F) << 24) | ((host_id & 0xFF) << 8) | (motor_id & 0xFF)


def is_extended(can_id: int) -> bool:
    return (can_id & CAN_EFF_FLAG) != 0


def _clip(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def _f2u16(value: float, vmin: float, vmax: float) -> int:
    v = _clip(value, vmin, vmax)
    n = (v - vmin) / (vmax - vmin)
    q = round(n * 65535.0)
    if q < 0:
        return 0
    if q > 65535:
        return 65535
    return int(q)


def _u16_to_f(raw: int, vmin: float, vmax: float) -> float:
    return vmin + (raw / 65535.0) * (vmax - vmin)


def _pack_be16(value: int) -> bytes:
    return struct.pack('>H', value & 0xFFFF)


def _read_be16(payload: bytes, offset: int) -> int:
    return struct.unpack_from('>H', payload, offset)[0]


def _pack_le16(value: int) -> bytes:
    return struct.pack('<H', value & 0xFFFF)


def _read_le16(payload: bytes, offset: int) -> int:
    return struct.unpack_from('<H', payload, offset)[0]


def _read_le32(payload: bytes, offset: int) -> int:
    return struct.unpack_from('<I', payload, offset)[0]


def build_get_device_id_req(host_id: int, motor_id: int) -> CanFrame:
    req_id = build_eff_id(0, host_id, motor_id)
    return CanFrame((req_id & CAN_EFF_MASK) | CAN_EFF_FLAG, b'', 0)


def build_op_ctrl_req(motor_id: int, cmd: OpCommand) -> CanFrame:
    tor = _f2u16(cmd.torque_Nm, -12.0, 12.0)
    can_id = ((1 & 0x1F) << 24) | (tor << 8) | (motor_id & 0xFF)
    payload = bytearray(8)
    struct.pack_into('>H', payload, 0, _f2u16(cmd.pos_rad, -4.0 * math.pi, 4.0 * math.pi))
    struct.pack_into('>H', payload, 2, _f2u16(cmd.vel_rad_s, -30.0, 30.0))
    struct.pack_into('>H', payload, 4, _f2u16(cmd.kp, 0.0, 500.0))
    struct.pack_into('>H', payload, 6, _f2u16(cmd.kd, 0.0, 5.0))
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(payload), 8)


def build_stop_req(host_id: int, motor_id: int) -> CanFrame:
    can_id = build_eff_id(4, host_id, motor_id)
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(8), 8)


def build_clear_faults_req(host_id: int, motor_id: int) -> CanFrame:
    can_id = build_eff_id(4, host_id, motor_id)
    payload = bytearray(8)
    payload[0] = 0x01
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(payload), 8)


def build_enable_req(host_id: int, motor_id: int) -> CanFrame:
    can_id = build_eff_id(3, host_id, motor_id)
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(8), 8)


def build_change_motor_id_req(host_id: int, motor_id: int, new_motor_id: int) -> CanFrame:
    can_id = (
        ((7 & 0x1F) << 24)
        | ((new_motor_id & 0xFF) << 16)
        | ((host_id & 0xFF) << 8)
        | (motor_id & 0xFF)
    )
    payload = bytearray(8)
    payload[0] = 0x01
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(payload), 8)


def build_set_mechanical_zero_req(host_id: int, motor_id: int) -> CanFrame:
    can_id = build_eff_id(6, host_id, motor_id)
    payload = bytearray(8)
    payload[0] = 0x01
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(payload), 8)


def build_fault_warning_req(host_id: int, motor_id: int) -> CanFrame:
    can_id = build_eff_id(21, host_id, motor_id)
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(8), 8)


def build_set_baud_rate_req(host_id: int, motor_id: int, code: int) -> CanFrame:
    can_id = build_eff_id(22, host_id, motor_id)
    payload = bytearray(8)
    payload[0] = code & 0xFF
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(payload), 8)


def build_read_param_req(host_id: int, motor_id: int, index: int) -> CanFrame:
    can_id = build_eff_id(17, host_id, motor_id)
    payload = bytearray(8)
    payload[:2] = _pack_le16(index)
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(payload), 8)


def build_write_param_req(host_id: int, motor_id: int, index: int, data: bytes) -> CanFrame:
    can_id = build_eff_id(18, host_id, motor_id)
    payload = bytearray(8)
    payload[:2] = _pack_le16(index)
    payload[4:8] = data[:4].ljust(4, b'\x00')
    return CanFrame((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG, bytes(payload), 8)


def parse_device_id_resp(frame: CanFrame, expect_motor_id: int) -> Optional[bytes]:
    if not is_extended(frame.can_id):
        return None
    can_id = frame.can_id & CAN_EFF_MASK
    if ((can_id >> 24) & 0x1F) != 0:
        return None
    if (can_id & 0xFF) != 0xFE:
        return None
    mid_lo = (can_id >> 8) & 0xFF
    mid_hi = (can_id >> 16) & 0xFF
    expect = expect_motor_id & 0xFF
    if mid_lo != expect and mid_hi != expect:
        return None
    if frame.dlc != kUidLen:
        return None
    return frame.data[:kUidLen]


def parse_status(frame: CanFrame) -> Optional[Status]:
    if not is_extended(frame.can_id):
        return None
    can_id = frame.can_id & CAN_EFF_MASK
    if ((can_id >> 24) & 0x1F) != 2:
        return None
    if frame.dlc != 8:
        return None
    payload = frame.data.ljust(8, b'\x00')
    status = Status()
    status.motor_can_id = (can_id >> 8) & 0xFF
    status.fault_bits = (can_id >> 16) & 0x3F
    status.mode = (can_id >> 22) & 0x03
    status.raw_eff_id = can_id
    status.angle_rad = _u16_to_f(_read_be16(payload, 0), -4.0 * math.pi, 4.0 * math.pi)
    status.vel_rad_s = _u16_to_f(_read_be16(payload, 2), -30.0, 30.0)
    status.torque_Nm = _u16_to_f(_read_be16(payload, 4), -12.0, 12.0)
    status.temperature_c = _read_be16(payload, 6) / 10.0
    return status


def parse_fault_warning_resp(frame: CanFrame, expect_host_id: int) -> Optional[FaultWarning]:
    if not is_extended(frame.can_id):
        return None
    can_id = frame.can_id & CAN_EFF_MASK
    if ((can_id >> 24) & 0x1F) != 21:
        return None
    if ((can_id >> 8) & 0xFF) != (expect_host_id & 0xFF):
        return None
    if frame.dlc != 8:
        return None
    payload = frame.data.ljust(8, b'\x00')
    return FaultWarning(
        faults=_read_le32(payload, 0),
        warnings=_read_le32(payload, 4),
    )


def parse_read_param_resp(
    frame: CanFrame,
    expect_host_id: int,
    expect_index: int,
) -> Optional[bytes]:
    if not is_extended(frame.can_id):
        return None
    can_id = frame.can_id & CAN_EFF_MASK
    if ((can_id >> 24) & 0x1F) != 17:
        return None
    if (can_id & 0xFF) != (expect_host_id & 0xFF):
        return None
    if frame.dlc != 8:
        return None
    payload = frame.data.ljust(8, b'\x00')
    idx = _read_le16(payload, 0)
    if idx != (expect_index & 0xFFFF):
        return None
    return bytes(payload[4:8])
