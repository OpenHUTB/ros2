"""IMU 速度网络的单元测试。

不需要 AirSim / ROS，直接 ``python3 test/test_mlp.py`` 即可运行。

覆盖：
    1. 前向形状（5 帧 × 6 轴 = 30 维 -> 1 维速度）
    2. tanh 输出范围 [-1, 1]
    3. ``MLP.load()`` 确实把权重读进了实例（防止写错调用方式）
    4. 5 帧滑动窗拼接后仍是 30 维
    5. 用真实数据集的 z-score 推理，预测与真值同向（相关系数 > 0.3）
"""
from __future__ import annotations

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from uav_airsim_imu_velocity_estimator.mlp import MLP  # noqa: E402

WEIGHTS = os.path.join(ROOT, "models_trained", "imu_vel.npz")
DATA = os.path.join(ROOT, "models_trained", "imu_vel_data.npz")
WINDOW = 5
DIM = 6


def _build_net():
    net = MLP(n_in=DIM * WINDOW, hidden=(32,), n_out=1)
    net.load(WEIGHTS)
    return net


def test_forward_shape():
    net = _build_net()
    x = np.zeros((1, DIM * WINDOW), dtype=np.float32)
    y = net.forward(x)
    assert np.asarray(y).shape == (1, 1), "前向输出形状应为 (1,1)，实际 %s" % (np.asarray(y).shape,)


def test_forward_range():
    net = _build_net()
    rng = np.random.RandomState(0)
    for _ in range(8):
        x = rng.randn(1, DIM * WINDOW).astype(np.float32)
        y = float(np.asarray(net.forward(x)).ravel()[0])
        assert -1.0 <= y <= 1.0, "tanh 输出越界：%f" % y


def test_load_changes_weights():
    fresh = MLP(n_in=DIM * WINDOW, hidden=(32,), n_out=1, seed=0)
    loaded = _build_net()
    d = float(np.abs(np.asarray(loaded.W[0]) - np.asarray(fresh.W[0])).max())
    assert d > 1e-3, "load 之后 W0 与随机初始化几乎相同，权重没读进来"
    x = np.ones((1, DIM * WINDOW), dtype=np.float32)
    y_fresh = float(np.asarray(fresh.forward(x)).ravel()[0])
    y_loaded = float(np.asarray(loaded.forward(x)).ravel()[0])
    assert abs(y_fresh - y_loaded) > 1e-3, "加载前后预测一致，权重可能没生效"


def test_window_concat_shape():
    """复刻 node.py 的滑动窗：不足 5 帧时用首帧补齐。"""
    buf = []
    vec = np.arange(DIM, dtype=np.float32)
    for _ in range(WINDOW):
        buf.append(vec)
        if len(buf) > WINDOW:
            buf.pop(0)
        while len(buf) < WINDOW:
            buf.insert(0, vec)
    x = np.concatenate(buf, axis=0)
    assert x.shape == (DIM * WINDOW,), "拼接后应为 30 维，实际 %s" % (x.shape,)


def test_prediction_correlates_with_truth():
    """在真实数据集上跑一遍，预测应跟真值同向；R² 0.45 时相关系数约 0.67。"""
    net = _build_net()
    d = np.load(DATA)
    # dataset.py 存盘时写的是 X=(X - x_mean)/x_std，X 已经是标准化后的，
    # 这里不能再减一次均值，否则等于二次标准化，相关系数会被压到 0.5 以下。
    xn = np.asarray(d["X"], dtype=np.float32)
    Y = np.asarray(d["Y"], dtype=np.float32).ravel()

    preds = np.array([float(np.asarray(net.forward(xn[i:i + 1])).ravel()[0])
                      for i in range(xn.shape[0])])
    corr = float(np.corrcoef(preds, Y)[0, 1])
    assert corr > 0.3, "预测与真值相关系数只有 %.3f，疑似权重未加载" % corr
    print("  (数据集 %d 条，预测-真值相关系数 %.3f，对应 R² %.3f)"
          % (xn.shape[0], corr, corr ** 2))


def main():
    tests = [
        test_forward_shape,
        test_forward_range,
        test_load_changes_weights,
        test_window_concat_shape,
        test_prediction_correlates_with_truth,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print("PASS  %s" % t.__name__)
        except Exception as e:  # noqa: BLE001
            failed += 1
            print("FAIL  %s: %s" % (t.__name__, e))
    print("\n%d/%d 通过" % (len(tests) - failed, len(tests)))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
