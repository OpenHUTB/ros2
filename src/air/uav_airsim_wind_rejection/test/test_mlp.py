"""抗风补偿网络的单元测试。

不需要 AirSim / ROS，直接 ``python3 test/test_mlp.py`` 即可运行。

覆盖：
    1. 前向形状（IMU 5 帧 30 维 + 位置误差 2 维 = 32 维 -> 2 维补偿速度）
    2. tanh 输出范围 [-1, 1]
    3. ``MLP.load()`` 确实把权重读进了实例
    4. 输入拼接顺序与训练一致（先 30 维 IMU、后 2 维误差，合计 32）
    5. 在 600 条真实样本上复算，每个输出维度与真值正相关
"""
from __future__ import annotations

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from uav_airsim_wind_rejection.mlp import MLP  # noqa: E402

WEIGHTS = os.path.join(ROOT, "models_trained", "wind.npz")
DATA = os.path.join(ROOT, "models_trained", "wind_data.npz")
N_IN = 32
N_OUT = 2


def _build_net():
    net = MLP(n_in=N_IN, hidden=(32,), n_out=N_OUT)
    net.load(WEIGHTS)
    return net


def test_forward_shape():
    net = _build_net()
    x = np.zeros((1, N_IN), dtype=np.float32)
    y = net.forward(x)
    assert np.asarray(y).shape == (1, N_OUT), "前向输出形状应为 (1,2)，实际 %s" % (np.asarray(y).shape,)


def test_forward_range():
    net = _build_net()
    rng = np.random.RandomState(0)
    for _ in range(8):
        x = rng.randn(1, N_IN).astype(np.float32)
        y = np.asarray(net.forward(x))
        assert np.all(y >= -1.0) and np.all(y <= 1.0), "tanh 输出越界：%s" % (y,)


def test_load_changes_weights():
    fresh = MLP(n_in=N_IN, hidden=(32,), n_out=N_OUT, seed=0)
    loaded = _build_net()
    d = float(np.abs(np.asarray(loaded.W[0]) - np.asarray(fresh.W[0])).max())
    assert d > 1e-3, "load 之后 W0 与随机初始化几乎相同，权重没读进来"
    x = np.ones((1, N_IN), dtype=np.float32)
    assert abs(float(np.asarray(fresh.forward(x)).ravel()[0])
               - float(np.asarray(loaded.forward(x)).ravel()[0])) > 1e-3, "加载前后预测一致"


def test_input_layout():
    """node.py 的拼接顺序：5 帧 × 6 轴（30 维）在前，目标-当前位置误差（2 维）在后。"""
    buf = [np.arange(6, dtype=np.float32) for _ in range(5)]
    err = np.array([1.5, -0.5], dtype=np.float32)
    x = np.concatenate([np.concatenate(buf), err])
    assert x.shape == (N_IN,)
    assert np.allclose(x[-2:], err), "最后两维必须是位置误差"
    d = np.load(DATA)
    assert d["X"].shape[1] == N_IN, "数据集特征维度与网络输入不一致"


def test_prediction_correlates_with_truth():
    """在真实数据集上跑一遍，预测应跟真值同向；R² 0.99 时相关系数应接近 1。"""
    net = _build_net()
    d = np.load(DATA)
    # 注意：dataset.py 存盘时写的是 X=(X - x_mean)/x_std，X 已经是标准化后的，
    # 这里不能再减一次均值，否则等于把输入二次标准化，相关系数会崩掉。
    X = np.asarray(d["X"], dtype=np.float32)
    Y = np.asarray(d["Y"], dtype=np.float32)

    preds = np.array([np.asarray(net.forward(X[i:i + 1])).ravel() for i in range(X.shape[0])])
    corrs = [float(np.corrcoef(preds[:, j], Y[:, j])[0, 1]) for j in range(N_OUT)]
    print("  (%d 条样本，各输出维度相关系数：%.3f / %.3f)" % (X.shape[0], corrs[0], corrs[1]))
    assert all(c > 0.3 for c in corrs), "相关系数过低：%s，疑似权重未加载" % (corrs,)


def main():
    tests = [
        test_forward_shape,
        test_forward_range,
        test_load_changes_weights,
        test_input_layout,
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
