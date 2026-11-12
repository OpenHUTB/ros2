"""阶跃响应指标计算的单元测试。"""

import math

import pytest

from humanoid_knee_impedance.metrics import (
    compute_metrics,
    damping_ratio,
    final_value,
    initial_value,
    natural_frequency,
    overshoot,
    peak_torque,
    peak_velocity,
    predicted_steady_state_error,
    rise_time,
    settling_time,
    steady_state_error,
)

DT = 0.001


def make_sample(t, q, qd=0.0, tau=0.0, q_cmd=1.0):
    return {'t': t, 'q': q, 'qd': qd, 'tau': tau, 'q_cmd': q_cmd}


def make_first_order(final=1.0, tau=0.1, duration=1.0):
    """一阶惯性响应 q = final*(1-exp(-t/tau))，无超调。"""
    count = int(round(duration / DT)) + 1
    return [make_sample(i * DT,
                        final * (1.0 - math.exp(-i * DT / tau)))
            for i in range(count)]


def make_overshoot_series(peak=1.2, final=1.0, duration=0.6):
    """先冲到 peak，再回到 final：超调 = (peak-final)/final。"""
    series = []
    for i in range(int(round(duration / DT)) + 1):
        t = i * DT
        if t <= 0.2:
            value = peak * t / 0.2
        else:
            value = final + (peak - final) * math.exp(-(t - 0.2) / 0.05)
        series.append(make_sample(t, value))
    return series


def test_initial_and_final_value():
    series = make_first_order()
    assert initial_value(series) == 0.0
    assert final_value(series) == pytest.approx(1.0, abs=1e-3)


def test_steady_state_error_uses_final_value():
    series = make_first_order(final=0.9)
    assert steady_state_error(series, 1.0) == pytest.approx(0.1, abs=2e-3)


def test_predicted_steady_state_error_matches_intact_pd_balance():
    # kp·e = mgl·sin(q_des - e)：kp=2 时 e≈0.203，kp=10 时 e≈0.054
    assert predicted_steady_state_error(2.0, 0.6) == pytest.approx(
        0.203, abs=2e-3)
    assert predicted_steady_state_error(10.0, 0.6) == pytest.approx(
        0.054, abs=2e-3)
    with pytest.raises(ValueError):
        predicted_steady_state_error(0.0, 0.6)


def test_rise_time_of_first_order_system():
    series = make_first_order(final=1.0, tau=0.1)
    # 一阶系统 10%→90% 上升时间为 2.197·tau
    assert rise_time(series) == pytest.approx(0.2197, abs=3e-3)


def test_settling_time_of_first_order_system():
    series = make_first_order(final=1.0, tau=0.1)
    # 2% 误差带对应 t = -tau*ln(0.02) = 0.391 s
    assert settling_time(series) == pytest.approx(0.391, abs=3e-3)


def test_settling_time_returns_none_when_never_settled():
    series = make_first_order(final=1.0, tau=1.0, duration=0.5)
    assert settling_time(series) is None


def test_monotonic_response_has_no_overshoot():
    # 稳态值取尾段均值，单调上升序列的最大值略高于该均值，允许 0.05% 的数值余量
    assert overshoot(make_first_order()) == pytest.approx(0.0, abs=0.05)


def test_overshoot_of_peaked_response():
    series = make_overshoot_series(peak=1.2, final=1.0)
    assert overshoot(series) == pytest.approx(20.0, abs=1.0)


def test_peak_velocity_and_torque():
    series = [make_sample(0.0, 0.0, qd=0.0, tau=0.0),
              make_sample(0.001, 0.01, qd=-3.5, tau=4.2),
              make_sample(0.002, 0.02, qd=1.0, tau=-0.5)]
    assert peak_velocity(series) == pytest.approx(3.5)
    assert peak_torque(series) == pytest.approx(4.2)


def test_compute_metrics_aggregates_all_fields():
    metrics = compute_metrics(make_first_order(final=0.95), 1.0)
    expected_keys = {
        'initial_value', 'final_value', 'steady_state_error', 'rise_time',
        'peak_value', 'overshoot', 'settling_time', 'peak_velocity',
        'peak_torque',
    }
    assert expected_keys.issubset(metrics.keys())
    assert metrics['steady_state_error'] == pytest.approx(0.05, abs=2e-3)


def test_empty_series_raises_value_error():
    with pytest.raises(ValueError):
        compute_metrics([], 1.0)


def test_natural_frequency_and_damping_ratio():
    inertia = 0.0296
    assert natural_frequency(10.0, inertia) == pytest.approx(
        math.sqrt(10.0 / inertia))
    critical = 2.0 * math.sqrt(10.0 * inertia)
    assert damping_ratio(10.0, critical, inertia) == pytest.approx(1.0)
    assert damping_ratio(10.0, critical / 2.0, inertia) == pytest.approx(0.5)
    with pytest.raises(ValueError):
        natural_frequency(0.0, inertia)
