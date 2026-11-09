import numpy as np
import pytest

from mujoco_msk_arm_fatigue_learning.learning import (
    AdaptiveFatigueCompensator,
)


def test_gain_increases_under_persistent_error():
    compensator = AdaptiveFatigueCompensator(learning_rate=100.0)
    activation = np.full(6, 0.2)
    error = np.array([0.05, 0.03])
    initial = compensator.gain.copy()
    for _ in range(20):
        gain = compensator.update(activation, error, 0.01)
    assert np.all(gain > initial)


def test_gain_decays_toward_one_without_error():
    compensator = AdaptiveFatigueCompensator()
    compensator.gain[:] = 1.5
    gain = compensator.update(np.full(6, 0.2), np.zeros(2), 0.1)
    assert np.all(gain < 1.5)
    assert np.all(gain >= 1.0)


def test_compensation_is_bounded():
    compensator = AdaptiveFatigueCompensator(maximum_gain=2.0)
    compensator.gain[:] = 2.0
    compensated = compensator.compensate(
        np.full(6, 0.8),
        np.full(6, 0.9),
    )
    assert np.all(compensated <= 1.0)


def test_invalid_learning_input_is_rejected():
    compensator = AdaptiveFatigueCompensator()
    with pytest.raises(ValueError):
        compensator.update([0.1] * 5, [0.0, 0.0], 0.01)
    with pytest.raises(ValueError):
        compensator.update([0.1] * 6, [0.0, 0.0], 0.0)
