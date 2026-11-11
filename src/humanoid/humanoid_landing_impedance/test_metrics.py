"""落地指标计算的单元测试。"""

import pytest

from humanoid_landing_impedance.metrics import (
    active_peak_grf,
    compute_metrics,
    find_touchdown,
    first_peak_grf,
    loading_impulse,
    max_knee_flexion,
    max_pitch,
    settle_time,
)

ROBOT_MASS = 3.0
STATIC_WEIGHT = ROBOT_MASS * 9.81


def make_sample(t, grf, vz=0.0, z=0.8, pitch=0.0, knee=0.3):
    return {
        't': t,
        'grf': grf,
        'base_vz': vz,
        'base_z': z,
        'base_pitch': pitch,
        'knee_flexion': knee,
    }


def make_landing_series():
    """构造: 0-0.5s 自由落体，0.5s 触地尖峰 800N，0.6s 后回到静重。"""
    series = [make_sample(t * 0.01, 0.0, vz=-1.0) for t in range(50)]
    series.append(make_sample(0.50, 800.0, vz=-0.2, knee=0.35))
    series.append(make_sample(0.51, 300.0, vz=-0.05, knee=0.38))
    series.append(make_sample(0.52, 120.0, vz=0.0, knee=0.40))
    series.extend(
        make_sample(0.53 + i * 0.01, STATIC_WEIGHT, vz=0.0, knee=0.40)
        for i in range(50))
    return series


def test_find_touchdown_returns_first_contact_time():
    series = make_landing_series()
    assert find_touchdown(series) == pytest.approx(0.50)


def test_find_touchdown_returns_none_without_contact():
    series = [make_sample(t * 0.1, 0.0) for t in range(10)]
    assert find_touchdown(series) is None


def test_first_peak_picks_global_maximum():
    series = make_landing_series()
    assert first_peak_grf(series) == pytest.approx(800.0)


def test_first_peak_rejects_empty_series():
    with pytest.raises(ValueError):
        first_peak_grf([])


def test_active_peak_skips_initial_spike():
    series = make_landing_series()
    touchdown = find_touchdown(series)
    # 尖峰在触地瞬间，主动窗口应从 0.55s 开始，只剩静重
    assert active_peak_grf(series, touchdown) == pytest.approx(STATIC_WEIGHT)


def test_active_peak_returns_none_without_touchdown():
    series = [make_sample(t * 0.1, 0.0) for t in range(10)]
    assert active_peak_grf(series, None) is None


def test_loading_impulse_integrates_contact_window():
    series = make_landing_series()
    touchdown = find_touchdown(series)
    impulse = loading_impulse(series, touchdown)
    assert impulse is not None
    # 梯形积分: 尖峰段约 8.35 N·s + 27 个静重区间约 7.95 N·s
    assert impulse == pytest.approx(16.29, abs=0.5)


def test_max_knee_flexion_and_pitch():
    series = make_landing_series()
    assert max_knee_flexion(series) == pytest.approx(0.40)
    series[52]['base_pitch'] = -0.12
    assert max_pitch(series) == pytest.approx(0.12)


def test_settle_time_covers_unsettled_tail():
    series = make_landing_series()
    # 在 0.8s 处制造一次超差扰动
    series.append(make_sample(0.80, STATIC_WEIGHT * 1.5, vz=0.2))
    touchdown = find_touchdown(series)
    assert settle_time(series, touchdown, ROBOT_MASS) == pytest.approx(0.80)


def test_settle_time_returns_none_without_touchdown():
    series = [make_sample(t * 0.1, 0.0, vz=1.0) for t in range(10)]
    assert settle_time(series, None, ROBOT_MASS) is None


def test_compute_metrics_aggregates_all_fields():
    metrics = compute_metrics(make_landing_series(), ROBOT_MASS)
    expected_keys = {
        'touchdown', 'first_peak_grf', 'active_peak_grf',
        'loading_impulse', 'max_knee_flexion', 'max_pitch', 'settle_time',
    }
    assert expected_keys.issubset(metrics.keys())
    assert metrics['touchdown'] == pytest.approx(0.50)
    assert metrics['first_peak_grf'] == pytest.approx(800.0)
