"""指标口径的单元测试（纯 numpy，不依赖仿真器）。"""

import numpy as np
import pytest

from uav_rotor_fault_detector_nn.metrics import (
    DETECT_GAIN_THRESHOLD,
    confusion,
    detection_latency,
    faulty_mask,
    mean_absolute_error,
    per_rotor_mae,
    predicted_faulty_mask,
    summarize,
)


def _make_case(n_healthy: int = 50, n_faulty: int = 50, detect_after: int = 10):
    """真值：前 n_healthy 帧健康，之后 0 号桨归零；预测：故障后过 detect_after 帧才报警。"""
    n = n_healthy + n_faulty
    target = np.ones((n, 4))
    target[n_healthy:, 0] = 0.0

    pred = np.ones((n, 4))
    pred[n_healthy + detect_after:, 0] = 0.3

    ends = np.arange(n)
    return pred, target, ends, n_healthy, detect_after


def test_mae_and_per_rotor():
    pred, target, *_ = _make_case(detect_after=10)
    # 0 号桨：故障后 10 帧仍是 1.0（误差 1.0），之后 40 帧预测 0.3（误差 0.3）
    expected = (10 * 1.0 + 40 * 0.3) / 400
    assert mean_absolute_error(pred, target) == pytest.approx(expected)
    per_rotor = per_rotor_mae(pred, target)
    assert per_rotor.shape == (4,)
    assert per_rotor[0] == pytest.approx((10 * 1.0 + 40 * 0.3) / 100)
    np.testing.assert_allclose(per_rotor[1:], 0.0)


def test_masks_use_thresholds():
    pred, target, *_ = _make_case()
    truth = faulty_mask(target)
    guess = predicted_faulty_mask(pred)
    assert truth[:50].sum() == 0
    assert truth[50:].all()
    assert guess[50:60].sum() == 0
    assert guess[60:].all()
    # 阈值口径：恰好等于阈值不算故障
    edge = np.full((1, 4), DETECT_GAIN_THRESHOLD)
    assert not predicted_faulty_mask(edge)[0]


def test_confusion_counts_and_ratios():
    pred, target, *_ = _make_case(n_healthy=50, n_faulty=50, detect_after=10)
    result = confusion(pred, target)
    assert result["tp"] == 40
    assert result["fn"] == 10
    assert result["fp"] == 0
    assert result["tn"] == 50
    assert result["recall"] == pytest.approx(0.8)
    assert result["precision"] == pytest.approx(1.0)
    assert result["false_alarm_rate"] == pytest.approx(0.0)
    assert result["accuracy"] == pytest.approx(0.9)


def test_confusion_no_fault_predicted():
    pred = np.ones((10, 4))
    target = np.ones((10, 4))
    result = confusion(pred, target)
    assert result["tp"] == 0 and result["fn"] == 0
    assert result["recall"] == 0.0  # 没有故障样本时按 0 处理，不做除零


def test_detection_latency_counts_from_onset():
    pred, target, ends, n_healthy, detect_after = _make_case(detect_after=10)
    latency = detection_latency(pred, target, ends, dt=0.01)
    assert latency == pytest.approx(detect_after * 0.01)


def test_detection_latency_inf_when_never_detected():
    pred, target, ends, *_ = _make_case(detect_after=10)
    pred[:, 0] = 1.0
    assert detection_latency(pred, target, ends, dt=0.01) == float("inf")


def test_detection_latency_nan_when_no_fault():
    pred = np.ones((20, 4))
    target = np.ones((20, 4))
    ends = np.arange(20)
    assert np.isnan(detection_latency(pred, target, ends, dt=0.01))


def test_summarize_keys_and_groups():
    pred, target, ends, *_ = _make_case(detect_after=10)
    result = summarize(pred, target, ends, dt=0.01)
    assert set(result) >= {"mae", "per_rotor_mae", "confusion", "n_samples"}
    assert result["n_samples"] == len(pred)

    groups = np.array(["loss"] * len(pred), dtype=object)
    grouped = summarize(pred, target, ends, dt=0.01, groups=groups)
    assert "loss" in grouped["per_group"]
    entry = grouped["per_group"]["loss"]
    assert entry["n_samples"] == len(pred)
    assert entry["latency"] == pytest.approx(0.1)
    assert entry["recall"] == pytest.approx(0.8)
