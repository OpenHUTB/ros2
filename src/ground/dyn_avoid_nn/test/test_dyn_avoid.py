"""`dyn_avoid_nn` 的 smoke test（不依赖 MuJoCo）。

覆盖评审建议的三类内容：
1. 神经网络前向形状、确定性与保存/加载一致性；
2. 路径生成（稠密折线、逼近目标、不穿过静态障碍）；
3. 专家控制器输出有界，且在正前方有障碍时会让行。

`oracle.py` 通过 `consts.py` 取场地常量、`nn.py` 是纯 numpy 实现，
因此 CI 里没有 mujoco 也能跑。

运行：
    cd src/ground/dyn_avoid_nn
    python3 -m unittest discover -s test -v
"""

import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from nn import MLP                        # noqa: E402
from oracle import build_path, DynExpert  # noqa: E402

START = (-7.0, -5.0)
GOAL = (6.8, 5.0)


class NeuralNet(unittest.TestCase):
    def test_forward_shape(self):
        net = MLP([6, 16, 16, 2], seed=0)
        y = net.predict(np.zeros((4, 6)))
        self.assertEqual(y.shape, (4, 2))

    def test_forward_is_deterministic(self):
        net = MLP([6, 16, 16, 2], seed=1)
        x = np.random.default_rng(0).normal(size=(3, 6))
        self.assertTrue(np.allclose(net.predict(x), net.predict(x)))

    def test_save_load_roundtrip(self):
        net = MLP([6, 16, 16, 2], seed=2)
        x = np.random.default_rng(3).normal(size=(5, 6))
        expected = net.predict(x)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "model.npz")
            net.save(path)
            loaded = MLP.load(path)
        self.assertTrue(np.allclose(expected, loaded.predict(x), atol=1e-6))


class Path(unittest.TestCase):
    def test_path_is_dense_and_reaches_goal(self):
        path = build_path([], START, GOAL)
        self.assertGreater(len(path), 2)
        self.assertLess(float(np.hypot(path[-1, 0] - GOAL[0], path[-1, 1] - GOAL[1])), 1.0)
        steps = np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1]))
        self.assertLess(float(np.max(steps)), 0.6)

    def test_path_keeps_clear_of_static_obstacle(self):
        obstacle = [{"x": -1.0, "y": -0.5, "hx": 0.8, "hy": 0.8}]
        path = build_path(obstacle, START, GOAL)
        inside = (np.abs(path[:, 0] - obstacle[0]["x"]) < obstacle[0]["hx"] + 0.25) & \
                 (np.abs(path[:, 1] - obstacle[0]["y"]) < obstacle[0]["hy"] + 0.25)
        self.assertFalse(bool(np.any(inside)))


class Expert(unittest.TestCase):
    def setUp(self):
        self.path = build_path([], START, GOAL)
        self.pose = (float(self.path[0, 0]), float(self.path[0, 1]), 0.0)

    def test_output_is_bounded_without_movers(self):
        expert = DynExpert(self.path)
        v, w = expert.act(self.pose, speed=0.0, yaw_rate=0.0, movers=[], scan=None)
        self.assertGreaterEqual(v, 0.0)
        self.assertLessEqual(v, 1.5)
        self.assertLessEqual(abs(w), 1.2)

    def test_yields_when_mover_blocks_ahead(self):
        """正前方 0.8 m 处有静止障碍时不应继续正常前进（应刹车或倒退）。"""
        expert = DynExpert(self.path)
        x, y, _ = self.pose
        v, _w = expert.act(self.pose, speed=0.0, yaw_rate=0.0,
                           movers=[(x + 0.8, y, 0.0, 0.0)], scan=None)
        self.assertLessEqual(v, 0.10, "前方有障碍时应当让行，实际 v=%.3f" % v)

    def test_no_brake_when_mover_is_far(self):
        expert = DynExpert(self.path)
        x, y, _ = self.pose
        v, _w = expert.act(self.pose, speed=0.0, yaw_rate=0.0,
                           movers=[(x + 40.0, y + 30.0, 0.0, 0.0)], scan=None)
        self.assertGreater(v, 0.0)


if __name__ == "__main__":
    unittest.main()
