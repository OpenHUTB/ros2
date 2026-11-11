import numpy as np

from mujoco_msk_arm_proprioception.model import ProprioceptionModel
from mujoco_msk_arm_proprioception.sensor import NoisyDelayedProprioception


def sample(index):
    return ProprioceptionModel().estimate(
        shoulder_rad=0.01 * index,
        elbow_rad=1.0,
        shoulder_velocity_rad_s=0.2,
        elbow_velocity_rad_s=0.0,
        activations=[0.1] * 6,
        muscle_forces_n=[10.0 + index] * 6,
        dt_s=0.01,
    )


def test_transport_delay_returns_signal_after_buffer_fills():
    sensor = NoisyDelayedProprioception(
        sample_period_s=0.01,
        delay_s=0.02,
        length_noise_std_m=0.0,
        velocity_noise_std_m_s=0.0,
        force_noise_std_n=0.0,
        tendon_extension_noise_std_m=0.0,
    )
    first = sample(0)
    sensor.measure(first)
    assert not sensor.ready
    sensor.measure(sample(1))
    assert not sensor.ready
    delayed = sensor.measure(sample(2))
    assert sensor.ready
    assert np.allclose(delayed.golgi_force_n, first.golgi_force_n)


def test_noise_is_reproducible_for_the_same_seed():
    configuration = {
        'sample_period_s': 0.01,
        'delay_s': 0.0,
        'length_noise_std_m': 0.001,
        'velocity_noise_std_m_s': 0.01,
        'force_noise_std_n': 0.5,
        'tendon_extension_noise_std_m': 0.0002,
        'seed': 42,
    }
    first_sensor = NoisyDelayedProprioception(**configuration)
    second_sensor = NoisyDelayedProprioception(**configuration)
    first = first_sensor.measure(sample(0))
    second = second_sensor.measure(sample(0))
    assert np.allclose(first.golgi_force_n, second.golgi_force_n)


def test_reset_clears_delay_buffer():
    sensor = NoisyDelayedProprioception(delay_s=0.02)
    sensor.measure(sample(0))
    assert not sensor.ready
    sensor.reset()
    assert not sensor.ready
