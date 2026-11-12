"""drivetrain_torsion_state_feedback 的离线单元测试（不依赖 ROS 2）。"""

import math

import pytest

from drivetrain_torsion_state_feedback.drivetrain import (TwoInertiaDrivetrain,
                                                          eigenvalues2,
                                                          inverse2, mat_mul)


def test_natural_frequency_and_damping_formula():
    plant = TwoInertiaDrivetrain(jm=0.05, jw=0.6, k=1200.0, c=2.0)
    s = 1.0 / 0.05 + 1.0 / 0.6
    assert plant.natural_frequency == pytest.approx(math.sqrt(s * 1200.0))
    assert plant.damping_ratio == pytest.approx(s * 2.0 / (2.0 * plant.natural_frequency))
    assert plant.damping_ratio < 0.2


def test_open_loop_poles_are_complex_and_stable():
    plant = TwoInertiaDrivetrain()
    poles = plant.poles()
    assert all(pole.imag > 0.0 or pole.imag < 0.0 for pole in poles)
    assert all(pole.real < 0.0 for pole in poles)
    assert poles[0].real == pytest.approx(-plant.damping_ratio * plant.natural_frequency)


def test_system_is_controllable():
    plant = TwoInertiaDrivetrain()
    assert plant.is_controllable() is True
    assert abs(plant.controllability_determinant()) > 1.0


def test_ackermann_places_poles_at_target():
    plant = TwoInertiaDrivetrain()
    target = plant.desired_poles(plant.natural_frequency, 0.7)
    gain = plant.ackermann_gain(target)
    _a_cl, poles, zeta, wn = plant.closed_loop(gain)
    assert zeta == pytest.approx(0.7, abs=1e-6)
    assert wn == pytest.approx(plant.natural_frequency, rel=1e-6)
    assert poles[0].real == pytest.approx(target[0].real, rel=1e-6)


def test_gain_only_needs_relative_speed_when_keeping_natural_frequency():
    # 只补阻尼、不改固有频率时，扭转角那一项的增益约为 0
    plant = TwoInertiaDrivetrain()
    target = plant.desired_poles(plant.natural_frequency, 0.7)
    gain = plant.ackermann_gain(target)
    assert abs(gain[0][0]) < 1e-9
    assert gain[0][1] > 0.0


def test_feedback_reduces_settle_time():
    plant = TwoInertiaDrivetrain()
    gain = plant.ackermann_gain(plant.desired_poles(plant.natural_frequency, 0.7))
    open_loop = plant.simulate(gain=None, duration=0.30)
    closed_loop = plant.simulate(gain=gain, duration=0.30)

    def settle(samples, threshold=0.02):
        initial = abs(samples[0][1])
        band = threshold * initial
        last = max((t for t, theta, _o, _u in samples if abs(theta) > band),
                   default=0.0)
        return last

    assert settle(closed_loop) < settle(open_loop) * 0.6
    assert abs(closed_loop[-1][1]) < abs(open_loop[-1][1])


def test_simulation_starts_at_initial_twist():
    plant = TwoInertiaDrivetrain()
    samples = plant.simulate(duration=0.05)
    assert samples[0][1] == pytest.approx(0.01)
    assert samples[0][2] == pytest.approx(0.0)
    assert len(samples) > 5


def test_critical_damping_makes_ratio_one():
    plant = TwoInertiaDrivetrain()
    c_crit = plant.critical_damping()
    damped = TwoInertiaDrivetrain(c=c_crit)
    assert damped.damping_ratio == pytest.approx(1.0, rel=1e-9)
    assert c_crit > plant.c


def test_invalid_parameters_raise():
    with pytest.raises(ValueError):
        TwoInertiaDrivetrain(jm=0.0)
    with pytest.raises(ValueError):
        TwoInertiaDrivetrain(k=-1.0)
    with pytest.raises(ValueError):
        TwoInertiaDrivetrain(c=-0.1)
    with pytest.raises(ValueError):
        TwoInertiaDrivetrain.desired_poles(100.0, 1.0)


def test_matrix_helpers():
    a = [[2.0, 1.0], [1.0, 3.0]]
    product = mat_mul(a, inverse2(a))
    assert product[0][0] == pytest.approx(1.0)
    assert product[0][1] == pytest.approx(0.0, abs=1e-12)
    with pytest.raises(ValueError):
        inverse2([[1.0, 1.0], [1.0, 1.0]])
    poles = eigenvalues2([[-2.0, 0.0], [0.0, -5.0]])
    reals = sorted(pole.real for pole in poles)
    assert reals[0] == pytest.approx(-5.0)
    assert reals[1] == pytest.approx(-2.0)
