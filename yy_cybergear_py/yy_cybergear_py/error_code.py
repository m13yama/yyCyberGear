"""Error code definitions mirroring the C++ driver variants."""

from __future__ import annotations

from enum import Enum


class ErrorCode(Enum):
    """Concise error categories for CyberGear operations."""

    NONE = 0
    NOT_OPEN = 1
    TIMEOUT = 2
    IO = 3
    INVALID_FRAME = 4

    def __str__(self) -> str:  # pragma: no cover - trivial formatting
        return to_string(self)


_ERROR_STRINGS = {
    ErrorCode.NONE: 'None',
    ErrorCode.NOT_OPEN: 'NotOpen',
    ErrorCode.TIMEOUT: 'Timeout',
    ErrorCode.IO: 'Io',
    ErrorCode.INVALID_FRAME: 'InvalidFrame',
}


def to_string(error: ErrorCode) -> str:
    """Return user facing representation similar to the C++ helper."""

    return _ERROR_STRINGS.get(error, 'Unknown')
