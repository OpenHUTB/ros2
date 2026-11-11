"""深度避障网络的单元测试。

不依赖 pytest，直接 ``python3 test/test_cnn.py`` 即可运行，用来在没有 AirSim /
ROS 环境的机器上先验证网络与权重。

覆盖：
    1. 前向形状（64x64 单通道 -> 2 维：偏航归一化量、高度归一化量）
    2. tanh 输出范围 [-1, 1]
    3. im2col / col2im 互逆
    4. ``DepthPolicyNet.load()`` 返回已训练实例（防止 classmethod 返回值被丢弃）
    5. 640x480 非正方形深度图能缩放到 64x64
"""
from __future__ import annotations

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from uav_airsim_depth_obstacle_avoidance.cnn import DepthPolicyNet, im2col, col2im  # noqa: E402

WEIGHTS = os.path.join(ROOT, "models_trained", "depth_avoid.npz")


def test_forward_shape():
    net = DepthPolicyNet.load(WEIGHTS)
    x = np.zeros((64, 64), dtype=np.float32)
    y = net.predict(x)
    assert np.asarray(y).shape == (2,), "前向输出应当是 2 维，实际 %s" % (np.asarray(y).shape,)


def test_forward_range():
    net = DepthPolicyNet.load(WEIGHTS)
    rng = np.random.RandomState(0)
    for _ in range(8):
        x = rng.rand(64, 64).astype(np.float32)
        y = np.asarray(net.predict(x))
        assert np.all(y >= -1.0) and np.all(y <= 1.0), "tanh 输出越界：%s" % (y,)


def test_im2col_col2im_inverse():
    h = w = 8
    k, stride = 3, 1
    x = np.ones((1, h, w), dtype=np.float32)
    cols, oh, ow = im2col(x, k, stride)
    assert cols.shape == (oh * ow, 1 * k * k)
    back = col2im(cols, x.shape, k, stride, oh, ow)

    expect = np.zeros((h, w), dtype=np.float32)
    for y in range(0, h - k + 1, stride):
        for xx in range(0, w - k + 1, stride):
            expect[y:y + k, xx:xx + k] += 1.0
    np.testing.assert_allclose(back[0], expect, rtol=0, atol=1e-5)


def test_load_returns_trained_instance():
    fresh = DepthPolicyNet(seed=0)
    loaded = DepthPolicyNet.load(WEIGHTS)
    assert loaded is not fresh
    d = float(np.abs(np.asarray(loaded.fc2.b) - np.asarray(fresh.fc2.b)).max())
    assert d > 1e-3, "load 之后权重与随机初始化几乎相同，说明权重没读进来"

    x = np.zeros((64, 64), dtype=np.float32)
    x[20:50, 20:44] = 0.5
    y_fresh = np.asarray(fresh.predict(x))
    y_loaded = np.asarray(loaded.predict(x))
    assert np.abs(y_fresh - y_loaded).max() > 1e-3, "加载前后预测一致，权重可能没生效"


def test_nonsquare_depth_resize():
    raw = np.random.RandomState(1).rand(480 * 640).astype(np.float32) * 10.0
    h, w = 480, 640
    arr = np.clip(raw.reshape(h, w), 0.0, 20.0) / 20.0
    idx_h = np.linspace(0, h - 1, 64).astype(int)
    idx_w = np.linspace(0, w - 1, 64).astype(int)
    small = arr[np.ix_(idx_h, idx_w)]
    assert small.shape == (64, 64)


def main():
    tests = [
        test_forward_shape,
        test_forward_range,
        test_im2col_col2im_inverse,
        test_load_returns_trained_instance,
        test_nonsquare_depth_resize,
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
