"""Shared arm PWM ↔ angle mapping (mission + tests).

Map: 0° → PWM 500, 90° → 1500, 180° → 2500.
"""

from __future__ import annotations

import math

PWM_MIN = 500
PWM_MAX = 2500
PWM_CENTER = 1500


def deg_to_pwm(deg: float) -> int:
    """Map joint angle [0, 180]° → mission PWM [500, 2500]."""
    deg = max(0.0, min(180.0, float(deg)))
    span = PWM_MAX - PWM_MIN
    return int(round(PWM_MIN + deg * span / 180.0))


def pwm_to_rad(pwm: float) -> float:
    """Map mission PWM [500, 2500] → joint angle [0, π] rad."""
    pwm = max(PWM_MIN, min(PWM_MAX, float(pwm)))
    deg = (pwm - PWM_MIN) * 180.0 / (PWM_MAX - PWM_MIN)
    return math.radians(deg)


def pwm_to_deg(pwm: float) -> float:
    """Map mission PWM [500, 2500] → joint angle [0, 180]°."""
    pwm = max(PWM_MIN, min(PWM_MAX, float(pwm)))
    return (pwm - PWM_MIN) * 180.0 / (PWM_MAX - PWM_MIN)


# CSV schemas written by path_logger
PATH_CSV_HEADER = [
    'time_sec', 'time_nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
]
JOINT_CSV_HEADER = ['time_sec', 'time_nsec', 'j1', 'j2']
