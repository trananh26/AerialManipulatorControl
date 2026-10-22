import pytest

from drone_control.arm_mission_adrc import ADRCAltitudeController


def test_adrc_reduces_altitude_error_and_clamps_output():
    controller = ADRCAltitudeController(kp=1.6, kd=0.8, observer_gain=6.0, max_cmd=1.0)

    cmd = controller.update(reference=2.0, measurement=0.7, dt=0.1, arm_active=True)

    assert cmd > 0.0
    assert abs(cmd) <= controller.max_cmd + 1e-9
    assert controller.last_error > 0.0
