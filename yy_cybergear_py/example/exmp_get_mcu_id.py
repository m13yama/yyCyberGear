"""Minimal example to query the CyberGear MCU UID."""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from yy_cybergear_py import CyberGear, ErrorCode


def _parse_uint8(text: str) -> int:
    try:
        value = int(text, 0)
    except ValueError as exc:  # pragma: no cover - CLI error handling
        raise argparse.ArgumentTypeError(f'invalid integer literal: {text}') from exc
    if not 0 <= value <= 0xFF:
        raise argparse.ArgumentTypeError('value must be in range 0..255')
    return value


def format_uid(uid: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in uid)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-i', '--interface', default='can0', help='CAN interface (default: can0)')
    parser.add_argument('-H', '--host-id', default='0x01', type=_parse_uint8, help='Host CAN ID')
    parser.add_argument('-M', '--motor-id', default='0x01', type=_parse_uint8, help='Motor CAN ID')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print CAN frames')
    args = parser.parse_args(argv)

    dev = CyberGear(
        ifname=args.interface,
        host_id=args.host_id,
        motor_id=args.motor_id,
        verbose=args.verbose,
    )

    try:
        dev.open()
    except OSError as exc:
        parser.error(f'failed to open interface {args.interface!r}: {exc}')

    try:
        result = dev.get_mcu_id()
        if not result.ok or result.value is None:
            error = result.error or ErrorCode.INVALID_FRAME
            print(f'Failed to read MCU ID: {error}', file=sys.stderr)
            return 1
        uid = result.value
        print(
            'MCU UID:',
            format_uid(uid),
            f'(motor 0x{args.motor_id:02X} on {args.interface})',
        )
        return 0
    finally:
        dev.close()


if __name__ == '__main__':  # pragma: no cover - CLI entry point
    sys.exit(main())
