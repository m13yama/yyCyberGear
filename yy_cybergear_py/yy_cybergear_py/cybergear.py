"""SocketCAN-based CyberGear driver implemented in Python."""

from __future__ import annotations

import struct
import time
from typing import Callable, Optional, TypeVar

from . import data_frame_handler as dfh
from .data_frame_handler import CanFrame
from .error_code import ErrorCode
from .protocol_types import (
    FaultWarning,
    OpCommand,
    Status,
    BUS_VOLTAGE,
    CURRENT_FILTER_GAIN,
    CURRENT_KI,
    CURRENT_LIMIT,
    CURRENT_KP,
    IQ_FILTER,
    IQ_REFERENCE,
    MECHANICAL_POSITION,
    MECHANICAL_VELOCITY,
    POSITION_KP,
    POSITION_REFERENCE,
    ROTATION_TURNS,
    RUN_MODE,
    SPEED_KI,
    SPEED_KP,
    SPEED_LIMIT,
    SPEED_REFERENCE,
    TORQUE_LIMIT,
)
from .protocol_types import kUidLen as _UID_LEN
from .result import Result
from .socket_can import SocketCAN


T = TypeVar('T')


class CyberGear:
    """Feature parity wrapper with the C++ CyberGear class."""

    kUidLen = _UID_LEN
    kDefaultTimeoutMs = 200

    def __init__(
        self,
        ifname: str = 'can0',
        host_id: int = 0x01,
        motor_id: int = 0x01,
        verbose: bool = False,
        rcvbuf_bytes: int = -1,
        sndbuf_bytes: int = -1,
    ) -> None:
        self._can = SocketCAN(
            ifname,
            non_blocking=False,
            enable_can_fd=False,
            rcvbuf_bytes=rcvbuf_bytes,
            sndbuf_bytes=sndbuf_bytes,
        )
        self._ifname = ifname
        self._host_id = host_id & 0xFF
        self._motor_id = motor_id & 0xFF
        self._verbose = verbose

    # ========== lifecycle ==========
    def open(self) -> None:
        self._can.open()
        # Disable loopback/self reception just like the C++ client
        self._can.set_loopback(False)
        self._can.set_recv_own_msgs(False)

    def close(self) -> None:
        self._can.close()

    def is_open(self) -> bool:
        return self._can.is_open()

    @property
    def ifname(self) -> str:
        return self._ifname

    @property
    def host_id(self) -> int:
        return self._host_id

    @host_id.setter
    def host_id(self, value: int) -> None:
        self._host_id = value & 0xFF

    @property
    def motor_id(self) -> int:
        return self._motor_id

    @motor_id.setter
    def motor_id(self, value: int) -> None:
        self._motor_id = value & 0xFF

    def set_verbose(self, enable: bool) -> None:
        self._verbose = bool(enable)

    def verbose(self) -> bool:
        return self._verbose

    # ========== helper primitives ==========
    def _send_frame(self, frame: CanFrame) -> Optional[ErrorCode]:
        try:
            if self._verbose:
                self._debug_print_frame('TX', frame)
            self._can.send(frame)
        except OSError:
            return ErrorCode.IO
        return None

    def _wait_for(self, timeout_ms: int, parser: Callable[[CanFrame], Optional[T]]) -> Result[T]:
        deadline = time.monotonic() + max(timeout_ms, 0) / 1000.0
        wait_ms = 100
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return Result.failure(ErrorCode.TIMEOUT)
            poll_ms = min(wait_ms, max(int(remaining * 1000), 1))
            try:
                frame = self._can.recv(poll_ms)
            except OSError:
                return Result.failure(ErrorCode.IO)
            if frame is None:
                continue
            if self._verbose:
                self._debug_print_frame('RX', frame)
            parsed = parser(frame)
            if parsed is not None:
                return Result.success(parsed)

    def _debug_print_frame(self, label: str, frame: CanFrame) -> None:
        eff = dfh.is_extended(frame.can_id)
        mask = dfh.CAN_EFF_MASK if eff else dfh.CAN_SFF_MASK
        can_id = frame.can_id & mask
        id_width = 8 if eff else 3
        dump = ' '.join(f'{byte:02X}' for byte in frame.data[:frame.dlc])
        print(
            f'{label} can {"EFF" if eff else "SFF"} id=0x{can_id:0{id_width}X} '
            f'dlc={frame.dlc} data: {dump}'
        )

    # ========== public API ==========
    def get_mcu_id(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[bytes]:
        if not self.is_open():
            return Result.failure(ErrorCode.NOT_OPEN)
        frame = dfh.build_get_device_id_req(self._host_id, self._motor_id)
        error = self._send_frame(frame)
        if error:
            return Result.failure(error)

        def parser(rx: CanFrame) -> Optional[bytes]:
            return dfh.parse_device_id_resp(rx, self._motor_id)

        return self._wait_for(timeout_ms, parser)

    def send_operation_command(
        self, cmd: OpCommand, timeout_ms: int = kDefaultTimeoutMs
    ) -> Result[Status]:
        if not self.is_open():
            return Result.failure(ErrorCode.NOT_OPEN)
        frame = dfh.build_op_ctrl_req(self._motor_id, cmd)
        error = self._send_frame(frame)
        if error:
            return Result.failure(error)

        def parser(rx: CanFrame) -> Optional[Status]:
            parsed = dfh.parse_status(rx)
            if parsed and parsed.motor_can_id == self._motor_id:
                return parsed
            return None

        return self._wait_for(timeout_ms, parser)

    def clear_faults(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self._send_simple(dfh.build_clear_faults_req, timeout_ms)

    def stop_motor(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self._send_simple(dfh.build_stop_req, timeout_ms)

    def enable_motor(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self._send_simple(dfh.build_enable_req, timeout_ms)

    def set_mechanical_zero(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self._send_simple(dfh.build_set_mechanical_zero_req, timeout_ms)

    def change_motor_id(
        self, new_motor_id: int, timeout_ms: int = kDefaultTimeoutMs
    ) -> Result[bytes]:
        if not self.is_open():
            return Result.failure(ErrorCode.NOT_OPEN)
        frame = dfh.build_change_motor_id_req(self._host_id, self._motor_id, new_motor_id)
        error = self._send_frame(frame)
        if error:
            return Result.failure(error)

        def parser(rx: CanFrame) -> Optional[bytes]:
            uid = dfh.parse_device_id_resp(rx, new_motor_id)
            if uid is not None:
                return uid
            return None

        result = self._wait_for(timeout_ms, parser)
        if result.ok:
            self._motor_id = new_motor_id & 0xFF
        return result

    def request_fault_warning(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[FaultWarning]:
        if not self.is_open():
            return Result.failure(ErrorCode.NOT_OPEN)
        frame = dfh.build_fault_warning_req(self._host_id, self._motor_id)
        error = self._send_frame(frame)
        if error:
            return Result.failure(error)

        def parser(rx: CanFrame) -> Optional[FaultWarning]:
            return dfh.parse_fault_warning_resp(rx, self._host_id)

        return self._wait_for(timeout_ms, parser)

    def read_param_raw(self, index: int, timeout_ms: int = kDefaultTimeoutMs) -> Result[bytes]:
        if not self.is_open():
            return Result.failure(ErrorCode.NOT_OPEN)
        frame = dfh.build_read_param_req(self._host_id, self._motor_id, index)
        error = self._send_frame(frame)
        if error:
            return Result.failure(error)

        def parser(rx: CanFrame) -> Optional[bytes]:
            return dfh.parse_read_param_resp(rx, self._host_id, index)

        return self._wait_for(timeout_ms, parser)

    def write_param_raw(
        self, index: int, data: bytes, timeout_ms: int = kDefaultTimeoutMs
    ) -> Result[Status]:
        if not self.is_open():
            return Result.failure(ErrorCode.NOT_OPEN)
        frame = dfh.build_write_param_req(self._host_id, self._motor_id, index, data)
        error = self._send_frame(frame)
        if error:
            return Result.failure(error)

        def parser(rx: CanFrame) -> Optional[Status]:
            parsed = dfh.parse_status(rx)
            if parsed and parsed.motor_can_id == self._motor_id:
                return parsed
            return None

        return self._wait_for(timeout_ms, parser)

    def read_param_float(self, index: int, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        raw = self.read_param_raw(index, timeout_ms)
        if not raw.ok or raw.value is None:
            return Result.failure(raw.error or ErrorCode.INVALID_FRAME)
        value = struct.unpack('<f', raw.value)[0]
        return Result.success(value)

    def write_param_float(
        self, index: int, value: float, timeout_ms: int = kDefaultTimeoutMs
    ) -> Result[Status]:
        data = struct.pack('<f', value)
        return self.write_param_raw(index, data, timeout_ms)

    def set_run_mode(self, mode: int, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        payload = bytes([mode & 0xFF, 0, 0, 0])
        return self.write_param_raw(RUN_MODE, payload, timeout_ms)

    def get_run_mode(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[int]:
        raw = self.read_param_raw(RUN_MODE, timeout_ms)
        if not raw.ok or raw.value is None:
            return Result.failure(raw.error or ErrorCode.INVALID_FRAME)
        return Result.success(raw.value[0])

    def set_iq_reference(self, amps: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(IQ_REFERENCE, amps, timeout_ms)

    def get_iq_reference(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(IQ_REFERENCE, timeout_ms)

    def set_speed_reference(self, rad_s: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(SPEED_REFERENCE, rad_s, timeout_ms)

    def get_speed_reference(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(SPEED_REFERENCE, timeout_ms)

    def set_torque_limit(self, nm: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(TORQUE_LIMIT, nm, timeout_ms)

    def get_torque_limit(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(TORQUE_LIMIT, timeout_ms)

    def set_current_kp(self, value: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(CURRENT_KP, value, timeout_ms)

    def get_current_kp(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(CURRENT_KP, timeout_ms)

    def set_current_ki(self, value: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(CURRENT_KI, value, timeout_ms)

    def get_current_ki(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(CURRENT_KI, timeout_ms)

    def set_current_filter_gain(
        self, value: float, timeout_ms: int = kDefaultTimeoutMs
    ) -> Result[Status]:
        return self.write_param_float(CURRENT_FILTER_GAIN, value, timeout_ms)

    def get_current_filter_gain(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(CURRENT_FILTER_GAIN, timeout_ms)

    def set_position_reference(self, rad: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(POSITION_REFERENCE, rad, timeout_ms)

    def get_position_reference(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(POSITION_REFERENCE, timeout_ms)

    def set_speed_limit(self, rad_s: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(SPEED_LIMIT, rad_s, timeout_ms)

    def get_speed_limit(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(SPEED_LIMIT, timeout_ms)

    def set_current_limit(self, amps: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(CURRENT_LIMIT, amps, timeout_ms)

    def get_current_limit(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(CURRENT_LIMIT, timeout_ms)

    def set_rotation_turns(self, turns: int, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        payload = struct.pack('<h', turns)
        payload += b'\x00\x00'
        return self.write_param_raw(ROTATION_TURNS, payload, timeout_ms)

    def get_rotation_turns(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[int]:
        raw = self.read_param_raw(ROTATION_TURNS, timeout_ms)
        if not raw.ok or raw.value is None:
            return Result.failure(raw.error or ErrorCode.INVALID_FRAME)
        turns = struct.unpack('<h', raw.value[:2])[0]
        return Result.success(turns)

    def set_position_kp(self, value: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(POSITION_KP, value, timeout_ms)

    def get_position_kp(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(POSITION_KP, timeout_ms)

    def set_speed_kp(self, value: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(SPEED_KP, value, timeout_ms)

    def get_speed_kp(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(SPEED_KP, timeout_ms)

    def set_speed_ki(self, value: float, timeout_ms: int = kDefaultTimeoutMs) -> Result[Status]:
        return self.write_param_float(SPEED_KI, value, timeout_ms)

    def get_speed_ki(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(SPEED_KI, timeout_ms)

    def get_mechanical_position(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(MECHANICAL_POSITION, timeout_ms)

    def get_iq_filter(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(IQ_FILTER, timeout_ms)

    def get_mechanical_velocity(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(MECHANICAL_VELOCITY, timeout_ms)

    def get_bus_voltage(self, timeout_ms: int = kDefaultTimeoutMs) -> Result[float]:
        return self.read_param_float(BUS_VOLTAGE, timeout_ms)

    # ====== internal helpers ======
    def _send_simple(
        self,
        builder: Callable[[int, int], CanFrame],
        timeout_ms: int,
    ) -> Result[Status]:
        if not self.is_open():
            return Result.failure(ErrorCode.NOT_OPEN)
        frame = builder(self._host_id, self._motor_id)
        error = self._send_frame(frame)
        if error:
            return Result.failure(error)

        def parser(rx: CanFrame) -> Optional[Status]:
            parsed = dfh.parse_status(rx)
            if parsed and parsed.motor_can_id == self._motor_id:
                return parsed
            return None

        return self._wait_for(timeout_ms, parser)
