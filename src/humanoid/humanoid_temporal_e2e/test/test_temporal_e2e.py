"""test_temporal_e2e.py —— 多帧时序端到端模块回归测试

覆盖本 PR 修掉的每一个「能跑但结论是假的」问题：
- 站姿是否真的站得住（平衡层是否真的在起作用）
- 标签是否退化成常数（原实现 y 恒为 DEFAULT_STAND_TARGET）
- 采集是否真的能产出样本（原实现 `sim.fallen` 漏括号 → samples=0）
- 权重是否真的可用（原权重输出与站姿差 1.5 rad）
"""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from humanoid_temporal_e2e.sim import (TemporalE2ESim, DEFAULT_STAND_TARGET,
                                       SQUAT_TARGET, INIT_QPOS, JOINT_LO, JOINT_HI,
                                       JOINT_NAMES)
from humanoid_temporal_e2e.dataset import feat_from_gray, N_FRAMES
from humanoid_temporal_e2e.nn import TemporalMLP, STD_FLOOR

MODEL = os.path.join(HERE, "models", "humanoid_temporal.xml")
WEIGHTS = os.path.join(HERE, "models_trained", "temporal_e2e.npz")


class TestSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = TemporalE2ESim(MODEL)

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def test_01_model_dims(self):
        self.assertEqual(self.sim.model.nq, 22)
        self.assertEqual(self.sim.model.nu, 15)
        self.assertAlmostEqual(self.sim.timestep, 0.002, places=6)

    def test_02_stand_pose_within_limits(self):
        for tgt in (DEFAULT_STAND_TARGET, SQUAT_TARGET, INIT_QPOS):
            self.assertEqual(tgt.shape, (15,))
            self.assertTrue(np.all(tgt >= JOINT_LO), "目标角越下界")
            self.assertTrue(np.all(tgt <= JOINT_HI), "目标角越上界")

    def test_03_balance_keeps_standing(self):
        """平衡层开着必须站住；这是原站姿 3 s 自倒 89.8° 的直接回归。"""
        self.sim.reset()
        mx = 0.0
        for _ in range(1500):
            self.sim.step(balance=True)
            mx = max(mx, abs(np.degrees(self.sim.torso_pose()[0])))
        self.assertLess(mx, 5.0, f"平衡开 3 s 最大倾角 {mx:.2f}°")
        self.assertFalse(self.sim.fallen())

    def test_04_balance_off_really_falls(self):
        """关掉平衡层必须露馅，否则说明开关没起作用。"""
        self.sim.reset()
        mx = 0.0
        for _ in range(1500):
            self.sim.step(balance=False)
            mx = max(mx, abs(np.degrees(self.sim.torso_pose()[0])))
        self.assertGreater(mx, 20.0, f"关平衡 3 s 最大倾角仅 {mx:.2f}°")

    def test_05_teacher_target_varies(self):
        """老师输出必须随状态变化；这是“标签恒定”的回归。"""
        self.sim.reset()
        self.sim.settle(300)
        a = self.sim.teacher_target()
        # 手工把躯干压到 25° 前倾，再算一次
        self.sim.data.qpos[3:7] = [np.cos(np.radians(12.5)), 0,
                                  np.sin(np.radians(12.5)), 0]
        self.sim.data.qpos[2] = 1.05
        import mujoco
        mujoco.mj_forward(self.sim.model, self.sim.data)
        b = self.sim.teacher_target()
        self.assertEqual(a.shape, (15,))
        self.assertEqual(b.shape, (15,))
        drift = float(np.abs(a - b).max())
        self.assertGreater(drift, 0.05, f"倾斜 25° 后老师输出几乎不变：{drift:.4f}")

    def test_06_fallen_is_callable(self):
        """fallen 是方法：原实现写成 `if sim.fallen:` 恒真，采集直接 0 样本。"""
        self.assertFalse(callable(self.sim.fallen) is False)
        self.assertIsInstance(self.sim.fallen(), bool)
        self.sim.reset()
        self.assertFalse(self.sim.fallen())

    def test_07_image_feature(self):
        self.sim.reset()
        img = self.sim.render_front()
        self.assertEqual(img.shape, (64, 64))
        f = feat_from_gray(img)
        self.assertEqual(f.shape, (256,))
        self.assertGreaterEqual(float(f.min()), 0.0)
        self.assertLessEqual(float(f.max()), 1.0)
        self.assertGreater(float(f.std()), 1e-4, "特征恒定")


class TestNet(unittest.TestCase):
    def test_08_std_floor_and_shapes(self):
        rng = np.random.default_rng(0)
        X = rng.random((64, 768)) * 0.1
        X[:, 100:] = 0.5            # 大量近似恒定维度
        y = rng.random((64, 15))
        m = TemporalMLP(768, 32, 16, 15)
        m.fit_normalizer(X, y)
        self.assertTrue(np.all(m.x_std >= STD_FLOOR))
        self.assertEqual(m.predict(X[:4]).shape, (4, 15))

    def test_09_learns_toy_mapping(self):
        """能被优化的最小验证：固定映射应被学到。"""
        rng = np.random.default_rng(0)
        X = rng.random((256, 768))
        W = rng.normal(0, 0.1, (768, 15))
        y = X @ W
        m = TemporalMLP(768, 32, 16, 15, seed=1)
        m.fit_normalizer(X[:200], y[:200])
        for _ in range(200):
            m.train_step(X[:200], y[:200], lr=1e-2)
        pred = m.predict(X[200:])
        self.assertLess(float(np.abs(pred - y[200:]).mean()),
                        float(np.abs(y[200:]).mean()),
                        "训练后误差没有低于“输出均值”基线")

    @unittest.skipUnless(os.path.exists(WEIGHTS), "缺训练权重")
    def test_10_weights_usable(self):
        """用真实相机帧喂权重，输出必须落在站姿附近（原权重差 1.5 rad）。"""
        from humanoid_temporal_e2e.predictor import TemporalPredictor
        self.assertTrue("x_mean" in np.load(WEIGHTS).files, "权重缺标准化参数，需重训")
        sim = TemporalE2ESim(MODEL)
        sim.reset()
        sim.settle(200)
        pred = TemporalPredictor(WEIGHTS)
        pred.reset()
        for _ in range(N_FRAMES):
            pred.push_frame(sim.render_front())
        out = pred.predict()
        sim.close()
        self.assertEqual(out.shape, (15,))
        drift = float(np.abs(out - DEFAULT_STAND_TARGET).max())
        self.assertLess(drift, 1.5, f"权重输出离站姿 {drift:.2f} rad，不可用")


if __name__ == "__main__":
    unittest.main()
