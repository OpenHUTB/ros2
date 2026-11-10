import numpy as np
import pytest

from mujoco_msk_arm_proprioception.model import (
    MUSCLE_NAMES,
    ProprioceptionModel,
    coactivation_joint_stiffness,
)


def estimate(model, force_scale=1.0):
    return model.estimate(
        shoulder_rad=-0.2,
        elbow_rad=1.5,
        shoulder_velocity_rad_s=0.4,
        elbow_velocity_rad_s=-0.3,
        activations=np.full(len(MUSCLE_NAMES), 0.15),
        muscle_forces_n=np.full(len(MUSCLE_NAMES), 20.0 * force_scale),
        dt_s=0.01,
    )


def test_sample_has_six_finite_physical_channels():
    sample = estimate(ProprioceptionModel())
    assert sample.spindle_length_m.shape == (6,)
    assert sample.spindle_velocity_m_s.shape == (6,)
    assert sample.golgi_force_n.shape == (6,)
    assert sample.tendon_extension_m.shape == (6,)
    assert np.all(sample.spindle_length_m > 0.0)
    assert np.all(sample.tendon_extension_m > 0.0)


def test_tendon_extension_increases_with_force():
    low = estimate(ProprioceptionModel(), force_scale=1.0)
    high = estimate(ProprioceptionModel(), force_scale=2.0)
    assert np.all(high.tendon_extension_m > low.tendon_extension_m)
    assert np.all(high.spindle_length_m < low.spindle_length_m)


def test_tendon_force_and_activation_are_clipped():
    sample = ProprioceptionModel().estimate(
        shoulder_rad=0.0,
        elbow_rad=1.0,
        shoulder_velocity_rad_s=0.0,
        elbow_velocity_rad_s=0.0,
        activations=[-1.0] * 6,
        muscle_forces_n=[-4.0] * 6,
        dt_s=0.01,
    )
    assert np.all(sample.activation == 0.0)
    assert np.all(sample.golgi_force_n == 0.0)
    assert np.all(sample.tendon_extension_m == 0.0)


def test_stiffness_increases_monotonically_with_coactivation():
    levels = (0.02, 0.10, 0.25)
    minimum_eigenvalues = [
        np.linalg.eigvalsh(coactivation_joint_stiffness(level))[0]
        for level in levels
    ]
    assert minimum_eigenvalues[0] < minimum_eigenvalues[1]
    assert minimum_eigenvalues[1] < minimum_eigenvalues[2]


@pytest.mark.parametrize(
    'forces',
    (
        [1.0, 2.0],
        [1.0, 2.0, 3.0, 4.0, 5.0, float('nan')],
    ),
)
def test_invalid_force_vector_is_rejected(forces):
    with pytest.raises(ValueError):
        ProprioceptionModel().estimate(
            shoulder_rad=0.0,
            elbow_rad=1.0,
            shoulder_velocity_rad_s=0.0,
            elbow_velocity_rad_s=0.0,
            activations=[0.1] * 6,
            muscle_forces_n=forces,
            dt_s=0.01,
        )


def test_wrong_muscle_order_is_rejected():
    with pytest.raises(ValueError, match='muscle order'):
        ProprioceptionModel(
            muscle_names=(
                'shoulder_extensor',
                'shoulder_flexor',
                'elbow_flexor',
                'elbow_extensor',
                'biarticular_flexor',
                'biarticular_extensor',
            )
        )
