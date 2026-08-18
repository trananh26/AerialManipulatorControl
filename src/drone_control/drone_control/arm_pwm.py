"""Shared arm PWM ↔ angle mapping (mission + tests).

Linear map over the bench-calibrated servo range:
  0° → PWM 500,  90° → 1167,  180° → 1833,  270° → 2500.

Mid-span PWM 1500 is 135°, not 90°. Use PWM_90 for elbow init / SERVO10_TRIM.
"""

from __future__ import annotations

import math

PWM_MIN = 500
PWM_MAX = 2500
DEG_MAX = 270.0
RAD_MAX = math.radians(DEG_MAX)  # 3π/2


def deg_to_pwm(deg: float) -> int:
    """Map joint angle [0, DEG_MAX]° → mission PWM [500, 2500]."""
    deg = max(0.0, min(DEG_MAX, float(deg)))
    span = PWM_MAX - PWM_MIN
    return int(round(PWM_MIN + deg * span / DEG_MAX))


def pwm_to_rad(pwm: float) -> float:
    """Map mission PWM [500, 2500] → joint angle [0, 3π/2] rad."""
    return math.radians(pwm_to_deg(pwm))


def pwm_to_deg(pwm: float) -> float:
    """Map mission PWM [500, 2500] → joint angle [0, DEG_MAX]°."""
    pwm = max(PWM_MIN, min(PWM_MAX, float(pwm)))
    return (pwm - PWM_MIN) * DEG_MAX / (PWM_MAX - PWM_MIN)


# 90° command PWM (elbow CAD rest / init). Bench table listed 1166; round() → 1167.
PWM_90 = deg_to_pwm(90)


# CSV schemas written by path_logger
PATH_CSV_HEADER = [
    'time_sec', 'time_nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
]
JOINT_CSV_HEADER = ['time_sec', 'time_nsec', 'j1', 'j2']
