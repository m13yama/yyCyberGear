"""Simple Result container mirroring the C++ convenience wrapper."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, Optional, TypeVar

from .error_code import ErrorCode


T = TypeVar('T')


@dataclass(slots=True)
class Result(Generic[T]):
    """Represents either a value or an :class:`ErrorCode`."""

    value: Optional[T] = None
    error: Optional[ErrorCode] = None

    @classmethod
    def success(cls, value: T) -> 'Result[T]':
        return cls(value=value)

    @classmethod
    def failure(cls, error: ErrorCode) -> 'Result[T]':
        return cls(error=error)

    @property
    def ok(self) -> bool:
        return self.error is None

    def unwrap(self) -> T:
        if self.error is not None:
            raise RuntimeError(f'result has error: {self.error}')
        return self.value  # type: ignore[return-value]

    def __bool__(self) -> bool:  # pragma: no cover - convenience alias
        return self.ok
