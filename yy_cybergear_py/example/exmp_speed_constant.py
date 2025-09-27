"""Constant speed control loop using the Python CyberGear driver."""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from typing import Optional

from yy_cybergear_py import (
    CyberGear,
    ErrorCode,
    fault_bits_to_string,
    mode_to_string,
)

_DEFAULT_RATE_HZ = 100


def _parse_uint8(text: str) -> int:
    try:
        value = int(text, 0)
    except ValueError as exc:  # pragma: no cover - CLI error handling
        raise argparse.ArgumentTypeError(f'invalid integer literal: {text}') from exc
    if not 0 <= value <= 0xFF:
        raise argparse.ArgumentTypeError('value must be in range 0..255')
    return value


def _register_sigint(stop_event: threading.Event) -> None:
    def _handler(_signum, _frame) -> None:  # pragma: no cover - signal path
        stop_event.set()

    signal.signal(signal.SIGINT, _handler)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-i', '--interface', default='can0', help='CAN interface (default: can0)')
    parser.add_argument('-H', '--host-id', default='0x01', type=_parse_uint8, help='Host CAN ID')
    parser.add_argument('-M', '--motor-id', default='0x01', type=_parse_uint8, help='Motor CAN ID')
    parser.add_argument('-s', '--speed', default='2.0', type=float, help='Target speed [rad/s]')
    parser.add_argument(
        '-r',
        '--rate',
        default=_DEFAULT_RATE_HZ,
        type=float,
        help=f'Loop frequency in Hz (default: {_DEFAULT_RATE_HZ})',
    )
    parser.add_argument('-v', '--verbose', action='store_true', help='Print CAN frames')
    args = parser.parse_args(argv)

    if args.rate <= 0:
        parser.error('loop rate must be positive')

    stop_event = threading.Event()
    _register_sigint(stop_event)

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

    exit_code = 0
    period = 1.0 / args.rate

    try:
        cleared = dev.clear_faults()
        if not cleared.ok:
            error = cleared.error or ErrorCode.INVALID_FRAME
            print(f'Failed to clear faults: {error}', file=sys.stderr)
            return 1
        if not dev.set_run_mode(2).ok:  # 2 == speed mode per protocol
            print('Failed to set run mode to speed.', file=sys.stderr)
            return 1
        if not dev.enable_motor().ok:
            print('Failed to enable motor.', file=sys.stderr)
            return 1

        print(
            f'Constant speed control on motor 0x{args.motor_id:02X} via {args.interface}: '
            f'speed={args.speed:.3f} rad/s',
        )

        start_time = time.monotonic()
        next_deadline = start_time
        while not stop_event.is_set():
            now = time.monotonic()
            if now > next_deadline + period:
                print('Control loop overrun detected. Stopping.', file=sys.stderr)
                exit_code = 1
                break

            result = dev.set_speed_reference(float(args.speed))
            if not result.ok or result.value is None:
                error = result.error or ErrorCode.INVALID_FRAME
                print(f'Set speed failed: {error}. Stopping motor.', file=sys.stderr)
                exit_code = 1
                break

            status = result.value
            mode_str = mode_to_string(status.mode)
            faults = fault_bits_to_string(status.fault_bits)
            faults_str = 'none' if not faults else ', '.join(faults)
            elapsed = now - start_time
            print(
                f't={elapsed:8.3f}s '
                f'ang={status.angle_rad:7.4f}rad '
                f'vel={status.vel_rad_s:7.4f}rad/s '
                f'tau={status.torque_Nm:6.3f}Nm '
                f'T={status.temperature_c:6.2f}C '
                f'mode={mode_str:<12s} '
                f'faults={faults_str} '
                f'mid=0x{status.motor_can_id:02X}'
            )

            if status.fault_bits:
                print('Fault detected. Stopping motor.', file=sys.stderr)
                exit_code = 1
                break

            next_deadline += period
            sleep_time = next_deadline - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        try:
            dev.stop_motor()
        except Exception:  # pragma: no cover - best-effort cleanup
            pass
        dev.close()

    return exit_code


if __name__ == '__main__':  # pragma: no cover - CLI entry point
    sys.exit(main())
