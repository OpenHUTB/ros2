"""核心测试：肌肉模型、夹持力单调性。"""

import unittest

from mujoco_muscle_gripper.core import HillMuscle, muscle_force


class TestMuscle(unittest.TestCase):
    def test_fl_peak(self):
        self.assertAlmostEqual(HillMuscle.fl(1.0), 1.0, places=6)

    def test_fv_direction(self):
        self.assertGreater(HillMuscle.fv(-0.5), 1.0)   # 拉长出力更大
        self.assertLess(HillMuscle.fv(0.5), 1.0)       # 缩短出力更小

    def test_finger_muscle_shortens_when_closing(self):
        m = HillMuscle()
        m.a = 0.5
        f_open = muscle_force(m, 0.0, 0.0)
        f_closed = muscle_force(m, 0.02, 0.0)
        self.assertNotAlmostEqual(f_open, f_closed, places=3)

    def test_activation_lag(self):
        m = HillMuscle()
        m.update_activation(1.0, 0.005, tau_act=0.01)
        self.assertLess(m.a, 1.0)
        self.assertGreater(m.a, 0.0)


if __name__ == "__main__":
    unittest.main()
