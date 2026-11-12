"""沿墙清扫模块的单元测试（需要 MuJoCo；未安装时整组跳过）。

覆盖评审指出的两个"结论正确性"问题：
- 墙面几何：`wall_error()` / 沿墙栅格必须用**内壁**（±5.7 / ±4.7），而不是墙体中心线
  （±6.0 / ±5.0），否则恒定的 0.30 m 偏差会被当成跟踪误差；
- 成功判据：超时（覆盖率不足）不得计为成功，必须区分"达标 / 碰撞 / 超时"。

另外覆盖权重加载与专家闭环的基本行为。

运行：
    cd src/ground/wall_clean_nn
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
except Exception:
    HAVE_MUJOCO = False


@unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
class Geometry(unittest.TestCase):
    """墙面几何：内壁 = 墙体中心 - 半厚。"""

    def test_inner_wall_constants(self):
        from sim_core import HALF_X, HALF_Y, WALL_T, INNER_X, INNER_Y
        self.assertAlmostEqual(INNER_X, HALF_X - WALL_T)
        self.assertAlmostEqual(INNER_Y, HALF_Y - WALL_T)
        # 与 cover_world.xml 的 size="6 0.3 0.5" / pos="0 5 ..." 一致
        self.assertAlmostEqual(WALL_T, 0.3)

    def test_wall_error_uses_inner_wall(self):
        from sim_core import CoverSim, INNER_X
        sim = CoverSim(seed=0)
        sim.reset((-5.0, 0.0, 0.0))
        # 距东/西内壁 5.7-5.0 = 0.7；若仍用 6.0 会得到 1.0（旧 bug）
        self.assertAlmostEqual(sim.wall_error(), INNER_X - 5.0, places=6)

    def test_wall_mask_inside_inner_walls(self):
        from sim_core import CoverSim, INNER_X, INNER_Y
        sim = CoverSim(seed=0)
        mask = sim.wall_mask
        idx = np.argwhere(mask)
        self.assertGreater(len(idx), 0)
        for i, j in idx[:200]:
            x, y = sim.cell_center(i, j)
            self.assertLessEqual(abs(x), INNER_X + 1e-6)
            self.assertLessEqual(abs(y), INNER_Y + 1e-6)


class DummyPolicy(object):
    """原地不动的策略，用来验证"超时不得算成功"。"""

    def reset(self):
        pass

    def act(self, feat):
        return 0.0, 0.0


@unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
class SuccessCriterion(unittest.TestCase):

    def test_timeout_is_not_success(self):
        import evaluate
        r = evaluate.run(DummyPolicy(), seed=100, steps=300, cov_target=0.95)
        self.assertEqual(r["success"], 0, "停在原地不应算成功")
        self.assertEqual(r["reason"], "timeout")
        self.assertLess(r["wall_coverage"], 0.95)

    def test_success_requires_coverage_and_no_collision(self):
        import evaluate
        r = evaluate.run(DummyPolicy(), seed=100, steps=300, cov_target=0.10)
        # 阈值故意设低：只要没碰撞且覆盖达标即可成功
        self.assertEqual(r["collision"], 0)
        self.assertEqual(r["success"], int(r["wall_coverage"] >= 0.10))


@unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
class ExpertAndModels(unittest.TestCase):

    def test_expert_finishes_with_high_coverage(self):
        import evaluate
        r = evaluate.run(None, seed=100, steps=9000, cov_target=1.0)  # 不提前停，测上限
        self.assertEqual(r["collision"], 0, "专家不应碰撞")
        self.assertGreater(r["wall_coverage"], 0.90, "专家覆盖率应 > 90%")
        self.assertLess(r["wall_error"], 0.10, "贴墙误差应接近 0（修复 0.3 偏差后）")

    def test_shipped_weights_load(self):
        from nn import MLP
        path = os.path.join(ROOT, "models", "wall_planner.npz")
        self.assertTrue(os.path.exists(path), path)
        net = MLP.load(path)
        z = np.load(os.path.join(ROOT, "models", "wall_norm.npz"))
        dim = z["xm"].shape[0]
        out = net.predict(np.zeros((1, dim)))
        self.assertEqual(out.shape, (1, 2))
        self.assertTrue(np.all(np.isfinite(out)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
