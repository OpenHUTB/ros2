"""速度规划的单元测试。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import profile as pf


class TestTrapezoid(unittest.TestCase):
    def test_reaches_exact_distance(self):
        for d in (0.02, 0.1, 0.5, 1.0, 3.0):
            tp = pf.TrapezoidProfile(d, 0.5, 1.0)
            s, v, a = tp.eval(tp.T)
            self.assertAlmostEqual(s, d, places=9)
            self.assertAlmostEqual(v, 0.0, places=9)

    def test_limits_respected(self):
        tp = pf.TrapezoidProfile(1.0, 0.5, 1.0)
        _, _, vs, as_ = tp.samples()
        self.assertLessEqual(max(vs), 0.5 + 1e-9)
        self.assertLessEqual(max(abs(x) for x in as_), 1.0 + 1e-9)

    def test_triangular_when_short(self):
        tp = pf.TrapezoidProfile(0.05, 0.5, 1.0)
        self.assertTrue(tp.triangular)
        self.assertLess(tp.vpeak, 0.5)
        self.assertAlmostEqual(tp.vpeak, (1.0 * 0.05) ** 0.5, places=9)

    def test_cruise_exists_for_long_distance(self):
        tp = pf.TrapezoidProfile(1.0, 0.5, 1.0)
        self.assertFalse(tp.triangular)
        self.assertAlmostEqual(tp.vpeak, 0.5)
        self.assertGreater(tp.tc, 0.0)

    def test_position_monotonic(self):
        tp = pf.TrapezoidProfile(1.0, 0.5, 1.0)
        _, ss, _, _ = tp.samples()
        self.assertTrue(all(ss[i + 1] >= ss[i] - 1e-12 for i in range(len(ss) - 1)))


class TestSCurve(unittest.TestCase):
    def test_reaches_exact_distance(self):
        for d in (0.05, 0.2, 1.0, 2.5):
            sc = pf.SCurveProfile(d, 0.5, 1.0, 5.0)
            self.assertAlmostEqual(sc.ss[-1], d, places=6)
            self.assertAlmostEqual(sc.vs[-1], 0.0, delta=1e-6)

    def test_jerk_bounded(self):
        sc = pf.SCurveProfile(1.0, 0.5, 1.0, 5.0)
        self.assertLessEqual(sc.peak_jerk(), 5.0 + 1e-9)

    def test_accel_and_velocity_limits(self):
        sc = pf.SCurveProfile(1.0, 0.5, 1.0, 5.0)
        self.assertLessEqual(sc.peak_accel(), 1.0 + 1e-6)
        self.assertLessEqual(sc.peak_velocity(), 0.5 + 1e-6)

    def test_slower_than_trapezoid(self):
        tp = pf.TrapezoidProfile(1.0, 0.5, 1.0)
        sc = pf.SCurveProfile(1.0, 0.5, 1.0, 5.0)
        self.assertGreater(sc.T, tp.T)
        self.assertLess(sc.T, tp.T * 1.5)      # 但不会慢太多

    def test_peak_velocity_reduced_when_short(self):
        long_ = pf.SCurveProfile(2.0, 0.5, 1.0, 5.0)
        short = pf.SCurveProfile(0.05, 0.5, 1.0, 5.0)
        self.assertAlmostEqual(long_.vpeak, 0.5, places=6)
        self.assertLess(short.vpeak, 0.2)
        self.assertAlmostEqual(short.ss[-1], 0.05, places=6)

    def test_monotonic_position(self):
        sc = pf.SCurveProfile(1.0, 0.5, 1.0, 5.0)
        self.assertTrue(all(sc.ss[i + 1] >= sc.ss[i] - 1e-12
                            for i in range(len(sc.ss) - 1)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
