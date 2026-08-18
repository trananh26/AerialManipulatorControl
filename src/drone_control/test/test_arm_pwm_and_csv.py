"""Unit tests for arm PWM mapping and path_logger CSV schemas."""

import csv
import math
import os
import sys
import tempfile

import pytest

# Allow `python -m pytest` without sourcing install
_PKG = os.path.join(os.path.dirname(__file__), '..')
if _PKG not in sys.path:
    sys.path.insert(0, os.path.abspath(_PKG))

from drone_control.arm_pwm import (  # noqa: E402
    DEG_MAX,
    JOINT_CSV_HEADER,
    PATH_CSV_HEADER,
    PWM_90,
    PWM_MAX,
    PWM_MIN,
    RAD_MAX,
    deg_to_pwm,
    pwm_to_deg,
    pwm_to_rad,
)
from drone_control.path_logger import (  # noqa: E402
    PathLoggerNode,
    find_workspace_log_parent,
)


@pytest.mark.parametrize(
    'deg,pwm',
    [
        (0, 500),
        (90, 1167),
        (135, 1500),
        (180, 1833),
        (270, 2500),
    ],
)
def test_deg_to_pwm(deg, pwm):
    assert deg_to_pwm(deg) == pwm


def test_deg_to_pwm_clamps():
    assert deg_to_pwm(-10) == PWM_MIN
    assert deg_to_pwm(300) == PWM_MAX
    assert deg_to_pwm(DEG_MAX) == PWM_MAX


def test_pwm_to_rad_roundtrip():
    # Integer PWM → at most ~0.07° quantization error
    for deg in (0, 45, 90, 135, 180, 270):
        pwm = deg_to_pwm(deg)
        assert abs(pwm_to_deg(pwm) - deg) < 0.1
        assert abs(pwm_to_rad(pwm) - math.radians(deg)) < math.radians(0.1)


def test_pwm_90_is_90_deg():
    assert deg_to_pwm(90) == PWM_90 == 1167
    assert abs(pwm_to_rad(PWM_90) - math.pi / 2) < math.radians(0.1)
    assert abs(pwm_to_rad(PWM_MAX) - RAD_MAX) < 1e-9


def test_path_csv_header_schema():
    assert PATH_CSV_HEADER == [
        'time_sec', 'time_nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
    ]


def test_joint_csv_header_schema():
    assert JOINT_CSV_HEADER == ['time_sec', 'time_nsec', 'j1', 'j2']


def test_write_joint_csv_schema():
    class _Stamp:
        sec = 1
        nanosec = 500000000

    class _Header:
        stamp = _Stamp()

    class _Msg:
        header = _Header()
        name = ['shoulder_joint', 'elbow_joint']
        position = [0.0, math.pi / 2]

    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'planned_joint_path.csv')
        PathLoggerNode._write_joint_csv(path, [_Msg()])
        with open(path, newline='') as f:
            rows = list(csv.reader(f))
        assert rows[0] == JOINT_CSV_HEADER
        assert rows[1][0] == '1'
        assert abs(float(rows[1][2]) - 0.0) < 1e-9
        assert abs(float(rows[1][3]) - math.pi / 2) < 1e-9


def test_find_workspace_log_parent_contains_trajectory_runs():
    parent = find_workspace_log_parent()
    assert parent.endswith(os.path.join('logs', 'trajectory_runs'))
