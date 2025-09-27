"""Lightweight SocketCAN wrapper used by the Python CyberGear driver."""

from __future__ import annotations

import select
import socket
import struct
from typing import Optional

from .data_frame_handler import CAN_EFF_MASK, CanFrame

_CAN_FRAME_FMT = '=IB3x8s'
_CAN_FRAME_SIZE = struct.calcsize(_CAN_FRAME_FMT)


class SocketCAN:
    """Minimal SocketCAN wrapper emulating the C++ helper."""

    def __init__(
        self,
        ifname: str,
        non_blocking: bool = False,
        enable_can_fd: bool = False,
        rcvbuf_bytes: int = -1,
        sndbuf_bytes: int = -1,
    ) -> None:
        self._ifname = ifname
        self._non_blocking = non_blocking
        self._enable_can_fd = enable_can_fd
        self._rcvbuf_bytes = rcvbuf_bytes
        self._sndbuf_bytes = sndbuf_bytes
        self._sock: Optional[socket.socket] = None
        self._loopback = True
        self._recv_own_msgs = True

    def open(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            sock.setblocking(not self._non_blocking)
            if self._enable_can_fd and hasattr(socket, 'CAN_RAW_FD_FRAMES'):
                sock.setsockopt(
                    socket.SOL_CAN_RAW,
                    socket.CAN_RAW_FD_FRAMES,
                    struct.pack('=I', 1),
                )
            if self._rcvbuf_bytes > 0:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self._rcvbuf_bytes)
            if self._sndbuf_bytes > 0:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self._sndbuf_bytes)
            sock.bind((self._ifname,))
            self._sock = sock
            self.set_loopback(self._loopback)
            self.set_recv_own_msgs(self._recv_own_msgs)
        except Exception:
            sock.close()
            self._sock = None
            raise

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def is_open(self) -> bool:
        return self._sock is not None

    def set_loopback(self, enable: bool) -> None:
        self._loopback = bool(enable)
        if self._sock is None:
            return
        sockopt = struct.pack('=I', 1 if enable else 0)
        self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, sockopt)

    def set_recv_own_msgs(self, enable: bool) -> None:
        self._recv_own_msgs = bool(enable)
        if self._sock is None:
            return
        sockopt = struct.pack('=I', 1 if enable else 0)
        self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, sockopt)

    def send(self, frame: CanFrame) -> int:
        if self._sock is None:
            raise RuntimeError('SocketCAN is not open')
        payload = frame.data.ljust(8, b'\x00')
        packed = struct.pack(_CAN_FRAME_FMT, frame.can_id, frame.dlc, payload)
        return self._sock.send(packed)

    def recv(self, timeout_ms: int = -1) -> Optional[CanFrame]:
        if self._sock is None:
            raise RuntimeError('SocketCAN is not open')
        timeout = None if timeout_ms < 0 else timeout_ms / 1000.0
        if timeout is not None:
            ready, _, _ = select.select([self._sock], [], [], timeout)
            if not ready:
                return None
        try:
            data = self._sock.recv(_CAN_FRAME_SIZE)
        except BlockingIOError:
            return None
        if len(data) < _CAN_FRAME_SIZE:
            return None
        can_id, dlc, payload = struct.unpack(_CAN_FRAME_FMT, data)
        dlc = min(dlc, 8)
        return CanFrame(can_id=can_id, data=payload[:dlc], dlc=dlc)

    def set_filters(self, filters: bytes) -> None:
        if self._sock is None:
            raise RuntimeError('SocketCAN is not open')
        self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)

    def set_error_mask(self, mask: int) -> None:
        if self._sock is None:
            raise RuntimeError('SocketCAN is not open')
        self._sock.setsockopt(
            socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack('=I', mask & CAN_EFF_MASK)
        )
