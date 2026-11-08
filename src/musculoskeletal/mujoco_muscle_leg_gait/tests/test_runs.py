"""实验层测试：L3 解析互验 + L1/L5/L6 集成冒烟。"""

import unittest

import numpy as np

from mujoco_muscle_leg_gait import runs


class TestL3Stiffness(unittest.TestCase):
    def test_numeric_matches_analytic_on_ascending_limb(self):
        res = runs.run_l3()
        asc = [r for r in res["sweep"] if r["limb"] == "ascending"]
        for r in asc:
            self.assertAlmostEqual(r["stiffness_num"], r["stiffness_ana"], places=6)

    def test_stiffness_grows_with_coactivation_on_ascending_limb(self):
        asc = [r for r in runs.run_l3()["sweep"] if r["limb"] == "ascending"]
        ks = [r["stiffness_num"] for r in asc]
        self.assertTrue(all(b > a for a, b in zip(ks, ks[1:])))


class TestL1SingleMuscle(unittest.TestCase):
    def test_muscle_pulls_joint_up(self):
        res = runs.run_l1()
        self.assertGreater(res["peak_angle"], 0.5)
        self.assertGreater(res["peak_force"], 10.0)


class TestWeightBearing(unittest.TestCase):
    def test_single_leg_stands(self):
        res = runs.run_l5()
        self.assertGreater(res["pelvis_z"], -0.10)      # 没有塌下去
        self.assertGreater(res["final_act"], 0.05)      # 确实在出力

    def test_stepping_lifts_foot(self):
        res = runs.run_l6()
        self.assertGreater(res["foot_clearance"], 0.05)
        self.assertGreater(res["pelvis_z"], -0.10)


if __name__ == "__main__":
    unittest.main()
