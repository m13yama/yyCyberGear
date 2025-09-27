"""Protocol level constants and typed payload containers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

kUidLen = 8

# Parameter indices (section 4.2 of the vendor documentation)
RUN_MODE = 0x7005
IQ_REFERENCE = 0x7006
SPEED_REFERENCE = 0x700A
TORQUE_LIMIT = 0x700B
CURRENT_KP = 0x7010
CURRENT_KI = 0x7011
CURRENT_FILTER_GAIN = 0x7014
POSITION_REFERENCE = 0x7016
SPEED_LIMIT = 0x7017
CURRENT_LIMIT = 0x7018
MECHANICAL_POSITION = 0x7019
IQ_FILTER = 0x701A
MECHANICAL_VELOCITY = 0x701B
BUS_VOLTAGE = 0x701C
ROTATION_TURNS = 0x701D
POSITION_KP = 0x701E
SPEED_KP = 0x701F
SPEED_KI = 0x7020


@dataclass(slots=True)
class OpCommand:
    """Operation control command used by type-1 frames."""

    pos_rad: float = 0.0
    vel_rad_s: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque_Nm: float = 0.0


@dataclass(slots=True)
class Status:
    """Decoded status feedback from the actuator."""

    angle_rad: float = 0.0
    vel_rad_s: float = 0.0
    torque_Nm: float = 0.0
    temperature_c: float = 0.0
    motor_can_id: int = 0
    mode: int = 0
    fault_bits: int = 0
    raw_eff_id: int = 0


@dataclass(slots=True)
class FaultWarning:
    """Snapshot of device faults and warnings."""

    faults: int = 0
    warnings: int = 0


def mode_to_string(mode: int) -> str:
    label = mode & 0x03
    if label == 0:
        return 'Reset'
    if label == 1:
        return 'Calibration'
    if label == 2:
        return 'Run'
    return f'Unknown({mode})'


def fault_bits_to_string(fault_bits: int) -> List[str]:
    fb = fault_bits & 0x3F
    out: List[str] = []
    if fb & (1 << 0):
        out.append('Undervoltage fault')
    if fb & (1 << 1):
        out.append('Overcurrent fault')
    if fb & (1 << 2):
        out.append('Over temperature fault')
    if fb & (1 << 3):
        out.append('Magnetic encoding failure')
    if fb & (1 << 4):
        out.append('HALL encoding failure')
    if fb & (1 << 5):
        out.append('not calibrated')
    return out
