"""control_discretization 的离线单元测试（不依赖 ROS 2）。"""

import math

import pytest

from control_discretization.matrices import (add, eigenvalues2, expm, eye,
                                             inverse2, mul, scale, sub)
from control_discretization.systems import (METHODS, SecondOrderSystem,
                                           critical_period, discretize,
                                           discrete_poles, is_stable,
                                           simulate_discrete)


def test_expm_identity_and_diagonal():
    assert expm([[0.0, 0.0], [0.0, 0.0]]) == eye(2)
    result = expm([[-2.0, 0.0], [0.0, -3.0]])
    assert result[0][0] == pytest.approx(math.exp(-2.0), rel=1e-9)
    assert result[1][1] == pytest.approx(math.exp(-3.0), rel=1e-9)
    assert result[0][1] == pytest.approx(0.0, abs=1e-12)


def test_inverse_and_multiplication():
    a = [[4.0, 7.0], [2.0, 6.0]]
    product = mul(a, inverse2(a))
    assert product[0][0] == pytest.approx(1.0)
    assert product[1][1] == pytest.approx(1.0)
    assert product[0][1] == pytest.approx(0.0, abs=1e-12)
    with pytest.raises(ValueError):
        inverse2([[1.0, 2.0], [2.0, 4.0]])


def test_eigenvalues_are_complex_for_oscillatory_system():
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    poles = eigenvalues2(system.a)
    assert all(abs(pole.imag) > 0.0 for pole in poles)
    assert poles[0].real == pytest.approx(-system.zeta * system.wn)


def test_system_rejects_invalid_parameters():
    with pytest.raises(ValueError):
        SecondOrderSystem(wn=0.0)
    with pytest.raises(ValueError):
        SecondOrderSystem(wn=3.0, zeta=1.0)


def test_zoh_matches_continuous_at_samples():
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    for ts in (0.01, 0.05, 0.2):
        reference = system.sample_continuous(ts, 4.0)
        samples, diverged = simulate_discrete(system, 'zoh', ts, 4.0)
        assert not diverged
        worst = max(abs(samples[i][1] - reference[i][1]) for i in range(len(reference)))
        assert worst < 1e-9, f'Ts={ts} 的 ZOH 误差 {worst}'


def test_all_methods_reach_unit_steady_state_at_small_ts():
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    for method in METHODS:
        samples, diverged = simulate_discrete(system, method, 0.01, 8.0)
        assert not diverged
        assert samples[-1][1] == pytest.approx(1.0, abs=5e-3), method


def test_forward_euler_goes_unstable_first():
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    assert is_stable(system, 'forward', 0.1) is True
    assert is_stable(system, 'forward', 0.5) is False


def test_other_methods_stay_stable_at_large_ts():
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    for method in ('backward', 'tustin', 'zoh'):
        assert is_stable(system, method, 0.5) is True, method


def test_forward_euler_diverges_in_simulation():
    # Ts=0.5 时前向欧拉不稳定：要么触发发散保护，要么响应已经明显跑飞
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    samples, diverged = simulate_discrete(system, 'forward', 0.5, 10.0)
    assert diverged or abs(samples[-1][1]) > 5.0


def test_critical_period_matches_forward_euler_limit():
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    ts_critical = critical_period(system, 'forward')
    assert ts_critical is not None
    assert 0.2 < ts_critical < 0.4
    assert critical_period(system, 'backward') is None
    assert critical_period(system, 'tustin') is None
    assert critical_period(system, 'zoh') is None


def test_discretize_rejects_bad_input():
    system = SecondOrderSystem()
    with pytest.raises(ValueError):
        discretize(system, 'rk4', 0.1)
    with pytest.raises(ValueError):
        discretize(system, 'zoh', 0.0)


def test_tustin_is_more_accurate_than_forward_euler():
    system = SecondOrderSystem(wn=3.0, zeta=0.4)
    ts = 0.1
    reference = system.sample_continuous(ts, 6.0)

    def worst_error(method):
        samples, _ = simulate_discrete(system, method, ts, 6.0)
        return max(abs(samples[i][1] - reference[i][1])
                   for i in range(len(reference)))

    assert worst_error('tustin') < worst_error('forward')


def test_poles_of_forward_euler_are_identity_plus_a_ts():
    system = SecondOrderSystem(wn=2.0, zeta=0.5)
    ts = 0.05
    f, g = discretize(system, 'forward', ts)
    expected = add(eye(2), scale(system.a, ts))
    assert f == expected
    assert g == scale(system.b, ts)
    poles = discrete_poles(system, 'forward', ts)
    assert len(poles) == 2
    assert sub(f, eye(2))[1][0] == pytest.approx(-(system.wn ** 2) * ts)
