"""Unit tests for the pix2cmd module (no ROS, no CARLA required)."""

import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from pix2cmd.cnn import TinyCNN, col2im, im2col
from pix2cmd.dataset import IMG, label, make_dataset, render, road_profile
from pix2cmd.train import closed_loop_drive, train


def test_image_shape_and_range():
    rng = np.random.default_rng(0)
    image = render(0.001, 0.3, rng)
    assert image.shape == (IMG, IMG)
    assert image.dtype == np.float32
    assert image.min() >= 0.0 and image.max() <= 1.0


def test_label_is_bounded_and_signed():
    left, _ = label(0.002, 0.0)
    right, _ = label(-0.002, 0.0)
    assert -1.0 <= left <= 1.0 and -1.0 <= right <= 1.0
    assert left < 0 < right                     # 正曲率（道路左弯）-> 负转向


def test_road_on_the_right_steers_right():
    # offset > 0 表示道路中心在相机右侧，车应向右修正 -> 转向为负
    steer_right_offset, _ = label(0.0, 1.0)
    assert steer_right_offset < 0
    assert label(0.0, -1.0)[0] > 0


def test_throttle_decreases_with_steering():
    _, throttle_straight = label(0.0, 0.0)
    _, throttle_turn = label(0.004, 0.0)
    assert throttle_straight > throttle_turn


def test_im2col_roundtrip_with_stride_equal_to_kernel():
    """stride == kernel 且尺寸整除时 patch 互不重叠，im2col/col2im 必须可逆。"""
    x = np.random.rand(2, 9, 9, 3).astype(np.float32)
    cols, (shape, ho, wo) = im2col(x, stride=3)
    assert (ho, wo) == (3, 3)
    assert cols.shape == (2 * ho * wo, 9 * 3)
    back = col2im(cols, shape, ho, wo, stride=3)
    assert back.shape == x.shape
    assert np.allclose(back, x)


def test_im2col_overlapping_stride_shapes():
    x = np.random.rand(2, 8, 8, 3).astype(np.float32)
    cols, (shape, ho, wo) = im2col(x, stride=2)
    back = col2im(cols, shape, ho, wo, stride=2)
    assert back.shape == x.shape
    assert np.isfinite(back).all()


def test_forward_shapes_and_finite():
    net = TinyCNN(seed=1)
    x, y, _ = make_dataset(4, seed=2)
    pred, cache = net.forward(x)
    assert pred.shape == (4, 2)
    assert np.isfinite(pred).all()
    assert pred[:, 0].min() >= -1.0 and pred[:, 0].max() <= 1.0
    assert pred[:, 1].min() >= 0.0 and pred[:, 1].max() <= 1.0


def test_backward_gradients_finite_and_shaped():
    net = TinyCNN(seed=1)
    x, y, _ = make_dataset(3, seed=3)
    _, cache = net.forward(x)
    grads = net.backward(cache, y)
    for key, value in grads.items():
        assert value.shape == net.p[key].shape, key
        assert np.isfinite(value).all(), key


def test_training_reduces_loss():
    x, y, _ = make_dataset(48, seed=4)
    net = TinyCNN(seed=4)
    before = float(np.mean((net.predict(x) - y) ** 2))
    for step in range(40):
        idx = np.random.default_rng(step).choice(48, 16, replace=False)
        pred, cache = net.forward(x[idx])
        net.step(net.backward(cache, y[idx]), lr=3e-3)
    after = float(np.mean((net.predict(x) - y) ** 2))
    assert after < before * 0.5, (before, after)


def test_short_training_run_learns_steering():
    net, history, (xva, yva) = train(epochs=3, n_train=120, n_val=40, lr=2e-3, log=lambda *a, **k: None)
    mae = float(np.abs(net.predict(xva)[:, 0] - yva[:, 0]).mean())
    assert mae < 0.25, mae
    assert history[-1]["train_mse"] < history[0]["train_mse"]


def test_closed_loop_stays_in_lane():
    net, _, _ = train(epochs=3, n_train=160, n_val=40, lr=2e-3, log=lambda *a, **k: None)
    frames, metrics = closed_loop_drive(net, steps=60)
    assert len(frames) == 60
    assert metrics["max_abs_lateral_error_m"] < 3.0
    assert 0.0 <= metrics["within_lane_ratio"] <= 1.0


def test_road_profile_is_bounded():
    for step in (0, 10, 30, 59):
        curvature, offset = road_profile(step, 60)
        assert abs(curvature) <= 0.005
        assert abs(offset) <= 1.2


# ---------------------------------------------------------------- 结构对比相关

def test_mlp_baseline_param_count_matches_formula():
    from pix2cmd.arch_sweep import MLPBaseline

    net = MLPBaseline(in_hw=IMG, hidden=64)
    expected = IMG * IMG * 64 + 64 + 64 * 2 + 2
    assert net.config()["params"] == expected


def test_mlp_forward_backward_shapes():
    from pix2cmd.arch_sweep import MLPBaseline

    net = MLPBaseline(hidden=8, seed=0)
    x, y, _ = make_dataset(3, seed=5)
    pred, cache = net.forward(x)
    assert pred.shape == (3, 2)
    assert -1.0 <= pred[:, 0].min() and pred[:, 0].max() <= 1.0
    assert 0.0 <= pred[:, 1].min() and pred[:, 1].max() <= 1.0
    grads = net.backward(cache, y)
    for key, value in grads.items():
        assert value.shape == net.p[key].shape, key
        assert np.isfinite(value).all(), key


def test_mlp_gradient_matches_finite_difference():
    """数值梯度校验：MLP 的反向传播必须与中心差分一致（相对误差 < 1e-3）。"""
    from pix2cmd.arch_sweep import MLPBaseline

    net = MLPBaseline(hidden=8, seed=1)
    x, y, _ = make_dataset(2, seed=6)
    _, cache = net.forward(x)
    grads = net.backward(cache, y)

    def loss():
        # 与 cnn.py 中反向传播的目标函数一致：J = (1/n) * Σ_i Σ_j (pred_ij - y_ij)^2
        return float(np.mean(((net.predict(x) - y) ** 2).sum(axis=1)))

    eps = 1e-2
    for key in ("w2", "b2"):
        before = net.p[key].copy()
        for idx in [(0, 0), (1, 1)] if before.ndim == 2 else [(0,)]:
            orig = before[idx]
            net.p[key][idx] = orig + eps
            plus = loss()
            net.p[key][idx] = orig - eps
            minus = loss()
            net.p[key][idx] = orig
            numeric = (plus - minus) / (2 * eps)
            assert abs(numeric - grads[key][idx]) < 1e-3 * max(1.0, abs(numeric))


def test_sweep_returns_one_record_per_architecture():
    from pix2cmd.arch_sweep import VARIANTS, sweep

    records = sweep(epochs=1, n_train=48, n_val=16, log=lambda *a, **k: None)
    assert [r["arch"] for r in records] == ["mlp", *VARIANTS]
    required = {"params", "val_mae_steering", "steady_abs_lateral_error_m",
                "settle_seconds", "overshoot_m", "train_seconds"}
    for rec in records:
        assert required <= set(rec), rec
        assert rec["params"] > 0
        assert 0.0 <= rec["within_lane_ratio"] <= 1.0


def test_recovery_test_reports_finite_values():
    from pix2cmd.arch_sweep import MLPBaseline, recovery_test

    net = MLPBaseline(hidden=8, seed=2)
    out = recovery_test(net, steps=20)
    assert out["final_offset_m"] >= 0.0
    assert out["overshoot_m"] >= 0.0
    assert out["settle_seconds"] >= -1.0
