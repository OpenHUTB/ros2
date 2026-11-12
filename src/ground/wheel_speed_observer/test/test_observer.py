"""wheel_speed_observer 的离线单元测试（不依赖 ROS 2）。"""

import math

import pytest

from wheel_speed_observer.observer import (DrivetrainObserver, ackermann_gain,
                                          error_settle_time, inverse2,
                                          eigenvalues2, transpose)


def test_system_is_observable():
    plant = DrivetrainObserver()
    assert plant.is_observable() is True
    assert abs(plant.observability_determinant()) > 1.0


def test_open_loop_poles_are_complex_and_stable():
    plant = DrivetrainObserver()
    poles = plant.open_loop_poles()
    assert all(pole.real < 0.0 for pole in poles)
    assert all(abs(pole.imag) > 0.0 for pole in poles)


def test_design_returns_column_gain_of_right_shape():
    plant = DrivetrainObserver()
    gain, poles = plant.design(wn=50.0, zeta=0.7)
    assert len(gain) == 2 and len(gain[0]) == 1
    assert poles[0].real == pytest.approx(-35.0)


def test_observer_poles_match_target():
    plant = DrivetrainObserver()
    gain, target = plant.design(wn=60.0, zeta=0.8)
    poles = plant.observer_poles(gain)
    reals = sorted(pole.real for pole in poles)
    assert reals[0] == pytest.approx(-48.0, rel=1e-6)
    assert abs(poles[0].imag) == pytest.approx(60.0 * math.sqrt(1 - 0.64), rel=1e-6)


def test_ackermann_rejects_uncontrollable_system():
    a = [[0.0, 0.0], [0.0, 0.0]]
    b = [[0.0], [0.0]]
    with pytest.raises(ValueError):
        ackermann_gain(a, b, (complex(-1, 1), complex(-1, -1)))


def test_estimation_error_decays():
    plant = DrivetrainObserver()
    gain, _poles = plant.design(wn=60.0, zeta=0.7)
    samples = plant.simulate(gain, duration=0.4)
    initial = abs(samples[0][3])
    assert initial > 0.01
    assert abs(samples[-1][3]) < 0.02 * initial


def test_higher_bandwidth_settles_faster():
    plant = DrivetrainObserver()
    slow, _ = plant.design(wn=30.0, zeta=0.7)
    fast, _ = plant.design(wn=120.0, zeta=0.7)
    slow_settle = error_settle_time(plant.simulate(slow, duration=0.6))
    fast_settle = error_settle_time(plant.simulate(fast, duration=0.6))
    assert slow_settle is not None and fast_settle is not None
    assert fast_settle < slow_settle


def test_noise_gain_grows_with_bandwidth():
    plant = DrivetrainObserver()
    norms = []
    for wn in (30.0, 60.0, 120.0):
        gain, _ = plant.design(wn=wn, zeta=0.7)
        norms.append(math.hypot(gain[0][0], gain[1][0]))
    assert norms[0] < norms[1] < norms[2]


def test_simulation_shapes_and_noise():
    plant = DrivetrainObserver()
    gain, _ = plant.design(wn=60.0)
    samples = plant.simulate(gain, duration=0.1, noise_std=0.05)
    assert len(samples) > 10
    assert all(len(sample) == 5 for sample in samples)
    assert samples[0][0] == pytest.approx(0.0)


def test_matrix_helpers():
    a = [[1.0, 2.0], [3.0, 4.0]]
    assert transpose(a) == [[1.0, 3.0], [2.0, 4.0]]
    product = [[1.0, 0.0], [0.0, 1.0]]
    inverse = inverse2(a)
    assert abs(a[0][0] * inverse[0][0] + a[0][1] * inverse[1][0] - product[0][0]) < 1e-12
    with pytest.raises(ValueError):
        inverse2([[1.0, 2.0], [2.0, 4.0]])
    poles = eigenvalues2([[0.0, 1.0], [-4.0, -1.0]])
    assert len(poles) == 2


def test_invalid_parameters_raise():
    with pytest.raises(ValueError):
        DrivetrainObserver(jw=0.0)
    with pytest.raises(ValueError):
        DrivetrainObserver(k=-5.0)
    with pytest.raises(ValueError):
        DrivetrainObserver(c=-1.0)
    with pytest.raises(ValueError):
        DrivetrainObserver().design(wn=50.0, zeta=1.5)
