"""姿态估计网络的单元测试。

不需要 AirSim / ROS，直接 ``python3 test/test_mlp.py`` 即可运行。

覆盖：
    1. 前向形状（5 帧 × 6 轴 = 30 维 -> 2 维姿态角）
    2. tanh 输出范围 [-1, 1]
    3. ``load_state()`` 确实把权重读进了实例
    4. 5 帧滑动窗拼接后仍是 30 维
    5. 在 660 条真实样本上复算（先 x 标准化、后 y 反标准化），预测与真值正相关

注意与本仓库另外两个 MLP 的差别：``attitude_data.npz`` 存的是**原始未标准化**的
``X``/``Y``，标准化参数单独存在 ``attitude.npz`` 里；而 imu / wind 的数据集存盘时
``X`` 已经标准化过。复算时不要混用两种约定。
"""
from __future__ import annotations

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from uav_airsim_attitude_estimator.mlp import MLP  # noqa: E402

WEIGHTS = os.path.join(ROOT, "models_trained", "attitude.npz")
DATA = os.path.join(ROOT, "models_trained", "attitude_data.npz")
WINDOW = 5
DIM = 6
N_IN = DIM * WINDOW
N_OUT = 2


def _build_net():
    """复刻 node.py 的加载方式：np.load 后取 W/b 交给 load_state。"""
    w = np.load(WEIGHTS)
    net = MLP(n_in=N_IN, hidden=(32,), n_out=N_OUT, seed=0)
    net.load_state({k: w[k] for k in ("W0", "b0", "W1", "b1")})
    return net, w


def test_forward_shape():
    net, _ = _build_net()
    x = np.zeros((1, N_IN), dtype=np.float32)
    y = net.forward(x)
    assert np.asarray(y).shape == (1, N_OUT), "前向输出形状应为 (1,2)，实际 %s" % (np.asarray(y).shape,)


def test_hidden_tanh_bounded_output_finite():
    """本模块输出层是**线性**（见 mlp.py 头部：避免姿态角集中在 0 时 tanh 饱和），
    所以不能像 imu / wind 那样断言输出落在 [-1,1]。这里改为：
    隐层必须被 tanh 压进 [-1,1]，最终输出必须有限。"""
    net, _ = _build_net()
    rng = np.random.RandomState(0)
    for _ in range(8):
        x = rng.randn(1, N_IN).astype(np.float32)
        y = np.asarray(net.forward(x))
        hidden = np.asarray(net.cache[-2])   # cache = [输入, 隐层, 输出]
        assert np.all(np.abs(hidden) <= 1.0), "隐层 tanh 输出越界：%s" % (hidden,)
        assert np.all(np.isfinite(y)), "前向输出出现 NaN/Inf：%s" % (y,)


def test_load_state_changes_weights():
    w = np.load(WEIGHTS)
    fresh = MLP(n_in=N_IN, hidden=(32,), n_out=N_OUT, seed=0)
    loaded = MLP(n_in=N_IN, hidden=(32,), n_out=N_OUT, seed=0)
    loaded.load_state({k: w[k] for k in ("W0", "b0", "W1", "b1")})
    d = float(np.abs(np.asarray(loaded.W[0]) - np.asarray(fresh.W[0])).max())
    assert d > 1e-3, "load_state 之后 W0 与随机初始化几乎相同，权重没读进来"
    x = np.ones((1, N_IN), dtype=np.float32)
    assert abs(float(np.asarray(fresh.forward(x)).ravel()[0])
               - float(np.asarray(loaded.forward(x)).ravel()[0])) > 1e-3, "加载前后预测一致"


def test_window_concat_shape():
    buf = []
    vec = np.arange(DIM, dtype=np.float32)
    for _ in range(5):
        buf.append(vec)
        if len(buf) > WINDOW:
            buf.pop(0)
        while len(buf) < WINDOW:
            buf.insert(0, vec)
    x = np.concatenate(buf, axis=0)[None, :]
    assert x.shape == (1, N_IN), "拼接后应为 (1,30)，实际 %s" % (x.shape,)


def test_prediction_correlates_with_truth():
    net, w = _build_net()
    d = np.load(DATA)
    X = np.asarray(d["X"], dtype=np.float32)   # 原始，未标准化
    Y = np.asarray(d["Y"], dtype=np.float32)   # 原始角度

    xn = ((X - w["x_mean"]) / w["x_std"]).astype(np.float32)
    preds = np.array([np.asarray(net.forward(xn[i:i + 1])).ravel() for i in range(xn.shape[0])])
    preds = preds * w["y_std"] + w["y_mean"]   # 反标准化回角度

    corrs = [float(np.corrcoef(preds[:, j], Y[:, j])[0, 1]) for j in range(N_OUT)]
    print("  (%d 条样本，两个角度分量相关系数：%.3f / %.3f)" % (X.shape[0], corrs[0], corrs[1]))
    assert all(c > 0.3 for c in corrs), "相关系数过低：%s，疑似权重未加载或标准化参数用错" % (corrs,)


def main():
    tests = [
        test_forward_shape,
        test_hidden_tanh_bounded_output_finite,
        test_load_state_changes_weights,
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
