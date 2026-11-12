# -*- coding: utf-8 -*-
"""雷达高度表：延迟、量化、噪声、丢帧与偏置不变性。"""
import numpy as np
import pytest

from uav_terrain_follow_agl.core.altimeter import (AltimeterConfig, RadarAltimeter,
                                                   latency_error_m)

DT = 0.01


def _run(cfg, agl_fn, t_end, seed=0):
    alt = RadarAltimeter(cfg, seed=seed)
    out = []
    for i in range(int(t_end / DT)):
        t = i * DT
        alt.update(t, agl_fn(t))
        out.append((t, alt.reading, alt.reading_time))
    return alt, out


def test_reading_is_none_before_latency():
    cfg = AltimeterConfig(latency_s=0.12, sigma_m=0.0, bias_m=0.0)
    alt = RadarAltimeter(cfg, seed=0)
    for i in range(10):                       # 0 ~ 0.09 s
        alt.update(i * DT, 10.0)
    assert alt.reading is None
    for i in range(10, 30):
        alt.update(i * DT, 10.0)
    assert alt.reading == pytest.approx(10.0)


def test_output_lags_by_latency_on_a_ramp():
    """高度线性变化时，读数应落后约 latency（采样保持再多半拍）。"""
    cfg = AltimeterConfig(latency_s=0.20, sigma_m=0.0, bias_m=0.0, quant_m=0.0,
                          dropout_p=0.0)
    rate = 2.0                                 # m/s
    alt, rows = _run(cfg, lambda t: 10.0 + rate * t, 3.0)
    for t, reading, _ in rows:
        if reading is None or t < 0.6:
            continue
        lag = (10.0 + rate * t - reading) / rate
        # 20 Hz 采样保持 → 0.2 s 传输延迟 + 最多 0.05 s 采样间隔
        assert 0.19 <= lag <= 0.27


def test_quantisation_snaps_to_grid():
    cfg = AltimeterConfig(sigma_m=0.0, bias_m=0.0, quant_m=0.10, dropout_p=0.0)
    alt, rows = _run(cfg, lambda t: 12.37, 1.0)
    vals = [r for _, r, _ in rows if r is not None]
    assert vals
    for v in vals:
        assert v == pytest.approx(round(v / 0.10) * 0.10, abs=1e-9)


def test_bias_is_fixed_for_the_whole_flight():
    cfg = AltimeterConfig(sigma_m=0.0, bias_m=0.5, quant_m=0.0, dropout_p=0.0)
    alt, rows = _run(cfg, lambda t: 20.0, 2.0)
    vals = [r for _, r, _ in rows if r is not None]
    assert len(set(round(v, 9) for v in vals)) == 1
    assert abs(vals[0] - 20.0) <= 3.0 * 0.5     # 一次抽样的偏置，落在 3σ 内


def test_noise_std_matches_config():
    cfg = AltimeterConfig(sigma_m=0.30, bias_m=0.0, quant_m=0.0, dropout_p=0.0)
    alt = RadarAltimeter(cfg, seed=7)
    vals = []
    for i in range(4000):
        alt.update(i * 0.05, 15.0)
        if alt.reading is not None:
            vals.append(alt.reading)
    assert len(vals) > 3000
    assert np.std(vals) == pytest.approx(0.30, rel=0.15)


def test_sample_count_follows_rate():
    cfg = AltimeterConfig(rate_hz=20.0, dropout_p=0.0, latency_s=0.0)
    alt, _ = _run(cfg, lambda t: 15.0, 2.0)
    # 2 s × 20 Hz = 40 帧（含 t=0 那一帧）
    assert 39 <= alt.n_samples <= 41


def test_dropout_holds_previous_reading():
    cfg = AltimeterConfig(dropout_p=1.0, latency_s=0.0, sigma_m=0.0, bias_m=0.0)
    alt = RadarAltimeter(cfg, seed=0)
    for i in range(100):
        alt.update(i * DT, 15.0)
    assert alt.reading is None
    assert alt.n_samples == 0
    assert alt.n_dropout > 0


def test_reset_clears_state_but_keeps_bias():
    cfg = AltimeterConfig(sigma_m=0.0, bias_m=0.4, latency_s=0.0)
    alt = RadarAltimeter(cfg, seed=3)
    bias = alt.bias
    for i in range(50):
        alt.update(i * DT, 15.0)
    alt.reset()
    assert alt.reading is None
    assert alt.n_samples == 0
    assert alt.bias == bias


@pytest.mark.parametrize("bad", [dict(rate_hz=0.0), dict(latency_s=-0.1),
                                 dict(sigma_m=-1.0), dict(dropout_p=1.5)])
def test_config_rejects_bad_values(bad):
    kwargs = dict()
    kwargs.update(bad)
    with pytest.raises(ValueError):
        AltimeterConfig(**kwargs)


def test_latency_error_formula():
    # 10 m/s、6° 坡、0.12 s → 10·0.1051·0.12 ≈ 0.126 m
    assert latency_error_m(10.0, np.tan(np.radians(6.0)), 0.12) == pytest.approx(
        10.0 * np.tan(np.radians(6.0)) * 0.12)


def test_update_handles_slow_calls():
    """控制周期比传感器周期长时，一次调用要补齐多帧。"""
    cfg = AltimeterConfig(rate_hz=20.0, latency_s=0.0, sigma_m=0.0, bias_m=0.0,
                          dropout_p=0.0)
    alt = RadarAltimeter(cfg, seed=0)
    alt.update(0.0, 15.0)
    alt.update(0.30, 15.0)                     # 0.30 / 0.05 → 应补 6 帧
    assert alt.n_samples >= 6
