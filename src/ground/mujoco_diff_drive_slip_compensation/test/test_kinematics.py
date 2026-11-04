"""Tests for differential-drive kinematics and command safety."""

import math

import pytest

from mujoco_diff_drive_slip_compensation.kinematics import (
    CommandWatchdog,
    DriveGeometry,
)


def test_straight_command_produces_equal_wheel_speeds():
    geometry = DriveGeometry(0.1, 0.4, 20.0)

    left, right = geometry.wheel_speeds(1.0, 0.0)

    assert left == pytest.approx(10.0)
    assert right == pytest.approx(10.0)


def test_turn_command_preserves_ratio_when_clipped():
    geometry = DriveGeometry(0.1, 0.4, 5.0)

    left, right = geometry.wheel_speeds(1.0, 1.0)

    assert right == pytest.approx(5.0)
    assert left / right == pytest.approx(8.0 / 12.0)


@pytest.mark.parametrize('value', [0.0, -1.0, math.inf, math.nan])
def test_geometry_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        DriveGeometry(value, 0.4, 10.0)


def test_watchdog_stops_after_timeout():
    now = [10.0]
    watchdog = CommandWatchdog(0.5, clock=lambda: now[0])
    watchdog.update(0.8, -0.2)

    assert watchdog.current() == (0.8, -0.2)
    now[0] = 10.51
    assert watchdog.current() == (0.0, 0.0)


def test_watchdog_rejects_non_finite_command():
    watchdog = CommandWatchdog(0.5)

    with pytest.raises(ValueError):
        watchdog.update(math.nan, 0.0)
