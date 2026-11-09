import numpy as np
import pytest

from mujoco_msk_arm_fatigue_learning.fatigue import MuscleFatigueModel


def test_fatigue_accumulates_and_reduces_strength():
    model = MuscleFatigueModel()
    state = model.update(np.full(6, 0.8), 0.1)
    assert np.all(state.fatigue > 0.0)
    assert np.all(state.strength_retention < 1.0)
    assert np.all(
        state.effective_activation < np.full(6, 0.8)
    )


def test_fatigue_recovers_without_activation():
    model = MuscleFatigueModel()
    model.fatigue[:] = 0.6
    state = model.update(np.zeros(6), 1.0)
    assert np.all(state.fatigue < 0.6)


def test_fatigue_is_bounded():
    model = MuscleFatigueModel(maximum_fatigue=0.4)
    for _ in range(100):
        state = model.update(np.ones(6), 1.0)
    assert np.all(state.fatigue <= 0.4)


@pytest.mark.parametrize(
    ('activation', 'dt_s'),
    (
        ([0.1] * 5, 0.1),
        ([0.1] * 6, 0.0),
        ([0.1, 0.1, 0.1, 0.1, 0.1, float('nan')], 0.1),
    ),
)
def test_invalid_fatigue_input_is_rejected(activation, dt_s):
    with pytest.raises(ValueError):
        MuscleFatigueModel().update(activation, dt_s)
