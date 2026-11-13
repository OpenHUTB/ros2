"""靠泊模块的单元测试（需要 MuJoCo；未安装时自动跳过）。

覆盖评审提出的"没有断言级测试"问题：
- 专家控制器在若干固定种子上能完成靠泊，且满足横向/艏向/速度容差；
- 同一 seed 下结果完全可复现；
- 随包权重能被加载，前向输出有界；
- 规划路径的起点与仿真起点一致。

运行：
    cd src/water/usv_docking_nn
    python3 -m unittest discover -s test -v
"""

import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except Exception:            # 没有仿真器时仍可被发现，只是跳过
    HAVE_MUJOCO = False

from sim_core import TOL_Y, TOL_YAW, TOL_V          # noqa: E402
from nn import MLP                                  # noqa: E402

SEEDS = (0, 1, 2)


def _run_expert(seed, steps=1100):
    """用专家控制器跑一个回合，返回 (结果, 步数, 末态)。"""
    from sim_core import DockSim, thrust_from_twist
    from oracle import DockExpert, build_path

    sim = DockSim(seed=seed)
    sim.reset(seed=seed)
    expert = DockExpert(build_path(sim.start))
    for k in range(steps):
        if sim.collided:
            return "collision", k, sim.pose(), sim
        if sim.docked():
            return "docked", k, sim.pose(), sim
        v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate())
        tl, tr = thrust_from_twist(v, w)
        sim.step(tl, tr)
    return "timeout", steps, sim.pose(), sim


@unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
class ExpertDocking(unittest.TestCase):

    def test_docks_within_tolerance(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                result, steps, pose, sim = _run_expert(seed)
                self.assertEqual(result, "docked", "seed=%d 未能靠泊（%s）" % (seed, result))
                x, y, yaw = pose
                self.assertLessEqual(abs(y), TOL_Y, "seed=%d 横向偏差 %.3f 超限" % (seed, y))
                self.assertLessEqual(abs(yaw), TOL_YAW, "seed=%d 艏向偏差 %.3f 超限" % (seed, yaw))
                self.assertLessEqual(sim.speed(), TOL_V, "seed=%d 末速 %.3f 超限" % (seed, sim.speed()))

    def test_no_collision(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                result, _, _, sim = _run_expert(seed)
                self.assertNotEqual(result, "collision", "seed=%d 发生碰撞" % seed)
                self.assertFalse(sim.collided, "seed=%d 碰撞标志为真" % seed)

    def test_deterministic_for_same_seed(self):
        a = _run_expert(0)
        b = _run_expert(0)
        self.assertEqual(a[0], b[0])
        self.assertEqual(a[1], b[1], "同一种子步数不一致：%d vs %d" % (a[1], b[1]))
        self.assertTrue(np.allclose(a[2], b[2], atol=1e-9), "同一种子末态不一致")


@unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
class ShippedModels(unittest.TestCase):

    def test_planner_weights_load_and_output_is_bounded(self):
        path = os.path.join(ROOT, "models", "dock_planner.npz")
        self.assertTrue(os.path.exists(path), "缺少随包权重：%s" % path)
        net = MLP.load(path)
        z = np.load(os.path.join(ROOT, "models", "dock_norm.npz"))
        feature_dim = z["xm"].shape[0]
        out = net.predict(np.zeros((1, feature_dim), np.float32))
        self.assertEqual(out.shape, (1, 2))
        self.assertTrue(np.all(np.isfinite(out)), "前向输出含非有限值")
        v, w = float(out[0, 0]), float(out[0, 1])
        self.assertLessEqual(abs(v), 1.5)
        self.assertLessEqual(abs(w), 1.5)


@unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
class Planning(unittest.TestCase):

    def test_path_starts_at_sim_start(self):
        from sim_core import DockSim
        from oracle import build_path

        sim = DockSim(seed=0)
        sim.reset(seed=0)
        path = build_path(sim.start)
        self.assertGreater(len(path), 2)
        self.assertLess(float(np.hypot(path[0][0] - sim.start[0], path[0][1] - sim.start[1])), 1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
