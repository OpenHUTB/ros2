"""Stanley 控制器的单元测试。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import stanley as st


class TestWrap(unittest.TestCase):
    def test_wrap_range(self):
        for a in (-7.0, -3.2, -math.pi, 0.0, math.pi, 3.2, 7.0, 100.0):
            w = st.wrap(a)
            self.assertGreater(w, -math.pi - 1e-12)
            self.assertLessEqual(w, math.pi + 1e-12)
            self.assertAlmostEqual(math.sin(w), math.sin(a), places=9)
            self.assertAlmostEqual(math.cos(w), math.cos(a), places=9)


class TestPathGeometry(unittest.TestCase):
    def test_nearest_and_error_sign(self):
        path = st.CirclePath(8.0)
        _, psi_ref, e = path.nearest(10.0, 0.0)     # 圆外
        self.assertAlmostEqual(e, 2.0, places=9)
        self.assertAlmostEqual(psi_ref, math.pi / 2.0, places=9)
        _, _, e_in = path.nearest(6.0, 0.0)         # 圆内
        self.assertAlmostEqual(e_in, -2.0, places=9)

    def test_curvature(self):
        self.assertAlmostEqual(st.CirclePath(8.0).curvature(), 0.125, places=12)


class TestSteerLaw(unittest.TestCase):
    def test_saturation(self):
        d = st.stanley_steer(e=100.0, psi_e=0.0, v=1.0, k=5.0)
        self.assertAlmostEqual(d, st.MAX_STEER, places=9)
        d2 = st.stanley_steer(e=-100.0, psi_e=0.0, v=1.0, k=5.0)
        self.assertAlmostEqual(d2, -st.MAX_STEER, places=9)

    def test_high_speed_reduces_correction(self):
        slow = st.stanley_steer(e=0.5, psi_e=0.0, v=1.0, k=1.0)
        fast = st.stanley_steer(e=0.5, psi_e=0.0, v=10.0, k=1.0)
        self.assertGreater(slow, fast)

    def test_feedforward_on_curve(self):
        # 弯道上即使误差为 0，也需要 atan(L*kappa) 的前轮转角
        d = st.stanley_steer(e=0.0, psi_e=0.0, v=2.0, k=1.2,
                             curvature=1.0 / 8.0)
        self.assertAlmostEqual(d, math.atan(2.5 / 8.0), places=9)
        d0 = st.stanley_steer(e=0.0, psi_e=0.0, v=2.0, k=1.2,
                              curvature=1.0 / 8.0, feedforward=False)
        self.assertAlmostEqual(d0, 0.0, places=9)


class TestClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = st.CirclePath(8.0)
        cls.r = st.simulate(path=cls.path)

    def test_converges(self):
        self.assertLess(abs(self.r["e"][-1]), 0.02)
        self.assertLess(abs(self.r["e"][-1]), abs(self.r["e"][0]))

    def test_steer_within_limit(self):
        self.assertLessEqual(max(abs(d) for d in self.r["delta"]),
                             st.MAX_STEER + 1e-12)

    def test_stays_near_reference(self):
        late = [abs(e) for t, e in zip(self.r["ts"], self.r["e"]) if t > 10.0]
        self.assertLess(max(late), 0.05)

    def test_k_term_matters(self):
        r0 = st.simulate(path=self.path, k=0.0)
        self.assertGreater(abs(r0["e"][-1]), 10 * abs(self.r["e"][-1]))

    def test_without_feedforward_has_offset(self):
        r2 = st.simulate(path=self.path, feedforward=False)
        theory = 2.5 / (1 + 1.2 * 8.0 / 2.0)       # L/(1+kR/v)
        self.assertAlmostEqual(r2["e"][-1], theory, delta=0.1)

    def test_radius_maintained(self):
        # 稳态后到圆心距离应回到 R=8
        late = [math.hypot(x, y) for x, y, t in
                zip(self.r["x"], self.r["y"], self.r["ts"]) if t > 15.0]
        self.assertAlmostEqual(sum(late) / len(late), 8.0, delta=0.05)


class TestLaunchFeedforward(unittest.TestCase):
    """评审指出的回归：文档命令写 use_feedforward:=false，launch 却没声明、
    也没把 feedforward 传给节点，导致前馈对照实验根本跑不起来。
    """
    @classmethod
    def setUpClass(cls):
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
        def read(rel):
            with open(os.path.join(root, rel), encoding="utf-8") as f:
                return f.read()
        cls.launch = read(os.path.join("launch", "stanley_tracking.launch.py"))
        cls.node = read(os.path.join("scripts", "plan_control_node.py"))
        cls.readme = read("README.md")

    def test_node_declares_feedforward(self):
        self.assertIn('declare_parameter("feedforward"', self.node)

    def test_launch_declares_and_passes_feedforward(self):
        self.assertIn('DeclareLaunchArgument("feedforward"', self.launch)
        self.assertIn('LaunchConfiguration("feedforward")', self.launch)
        self.assertIn('value_type=bool', self.launch)
        self.assertIn('"feedforward":', self.launch)

    def test_parameter_name_is_consistent(self):
        self.assertNotIn("use_feedforward", self.launch + self.node + self.readme)


if __name__ == "__main__":
    unittest.main(verbosity=2)
