from __future__ import annotations

from mujoco_motor_thermal_control.simulation import run_constant_command


def test_mujoco_drive_advances_pose_and_motor_temperature() -> None:
    states = run_constant_command(600, linear_mps=0.4)
    assert len(states) == 600
    assert states[-1].time_s > 0.0
    assert states[-1].x_m > 0.0
    assert states[-1].left_temperature_c > 25.0
    assert states[-1].right_temperature_c > 25.0
    assert abs(states[-1].pitch_rad) < 0.05


def test_high_angular_command_is_limited_without_disabling_drive() -> None:
    states = run_constant_command(600, linear_mps=0.0, angular_rps=15.0)
    assert any(abs(state.left_torque_nm) > 0.0 or abs(state.right_torque_nm) > 0.0 for state in states)
    assert abs(states[-1].yaw_rad) > 0.1
    assert abs(states[-1].pitch_rad) < 0.05


def test_motor_temperature_starts_at_ambient() -> None:
    from mujoco_motor_thermal_control.simulation import DifferentialDriveSimulation

    simulation = DifferentialDriveSimulation(ambient_c=35.0)
    assert simulation.left_motor.temperature_c == 35.0
    assert simulation.right_motor.temperature_c == 35.0


def test_120_second_drive_tracks_speed_and_remains_stable() -> None:
    target = 0.55
    states = run_constant_command(12000, linear_mps=target)
    assert max(abs(state.pitch_rad) for state in states) < 0.05
    assert abs(states[-1].speed_mps - target) / target < 0.05
