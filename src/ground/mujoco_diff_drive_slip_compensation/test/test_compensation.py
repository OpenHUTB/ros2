"""Unit tests for online sensor checks and command limiting."""

import numpy as np
import pytest

from mujoco_diff_drive_slip_compensation.compensation import (
    SlipCompensationPolicy,
)


class FixedModel:
    def __init__(self, slip: float = 0.2, window_size: int = 2):
        self.slip = slip
        self.window_size = window_size

    def predict(self, window: np.ndarray) -> float:
        assert window.shape == (self.window_size, 12)
        return self.slip


def _update(policy, stamp_ns, **overrides):
    inputs = {
        'stamp_ns': stamp_ns,
        'imu_stamp_ns': stamp_ns,
        'odometry_stamp_ns': stamp_ns,
        'command_stamp_ns': stamp_ns,
        'wheel_velocities': (1.0, 1.0, 1.0, 1.0),
        'acceleration': (0.0, 0.0, 9.8),
        'angular_velocity': (0.0, 0.0, 0.0),
        'command_linear': 0.4,
        'command_angular': 0.1,
    }
    inputs.update(overrides)
    return policy.update(**inputs)


def test_policy_warms_window_then_bounds_linear_correction():
    policy = SlipCompensationPolicy(FixedModel(), max_linear_acceleration=1.0)

    warming = _update(policy, 1_000_000_000)
    result = _update(policy, 1_050_000_000)

    assert not warming.ready
    assert (warming.linear, warming.angular) == (0.0, 0.0)
    assert result.ready
    assert result.slip == pytest.approx(0.2)
    assert result.linear == pytest.approx(0.05)
    assert result.angular == pytest.approx(0.1)


def test_policy_stops_and_resets_on_stale_or_non_monotonic_data():
    policy = SlipCompensationPolicy(FixedModel())
    _update(policy, 1_000_000_000)
    stale = _update(policy, 1_050_000_000, imu_stamp_ns=800_000_000)

    assert not stale.ready
    assert stale.reason == 'stale sensor or command'
    assert stale.linear == 0.0

    restarted = _update(policy, 1_100_000_000)
    assert restarted.reason == 'warming window'


def test_policy_rejects_non_finite_values_and_invalid_limits():
    with pytest.raises(ValueError, match='positive'):
        SlipCompensationPolicy(FixedModel(), max_linear_scale=0.0)

    result = _update(
        SlipCompensationPolicy(FixedModel()),
        1_000_000_000,
        command_linear=float('nan'),
    )
    assert result.reason == 'non-finite input'
