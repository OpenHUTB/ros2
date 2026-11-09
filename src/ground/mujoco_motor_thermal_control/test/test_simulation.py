from __future__ import annotations

from mujoco_motor_thermal_control.simulation import run_constant_command


def test_mujoco_drive_advances_pose_and_motor_temperature() -> None:
    states = run_constant_command(120, linear_mps=0.4)
    assert len(states) == 120
    assert states[-1].time_s > 0.0
    assert states[-1].x_m > 0.0
    assert states[-1].left_temperature_c > 25.0
    assert states[-1].right_temperature_c > 25.0
