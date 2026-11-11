"""实验层测试：夹持力单调、抬升阈值、松开掉落。"""

import unittest

from mujoco_muscle_gripper import extra, runs


class TestGripForce(unittest.TestCase):
    def test_force_monotonic_in_activation(self):
        res = runs.run_g1()
        forces = [lv["grip_force_n"] for lv in res["levels"]]
        self.assertEqual(forces, sorted(forces))
        self.assertGreater(forces[-1], forces[0] * 3)


class TestLift(unittest.TestCase):
    def test_high_activation_grips(self):
        res = runs.run_g2()
        strong = [lv for lv in res["levels"] if lv["activation"] >= 0.4]
        self.assertTrue(strong)
        for lv in strong:
            self.assertGreater(lv["lift_mm"], 50.0)

    def test_low_activation_slips(self):
        res = runs.run_g2()
        weak = [lv for lv in res["levels"] if lv["activation"] <= 0.1]
        self.assertTrue(weak)
        for lv in weak:
            self.assertLess(lv["lift_mm"], 5.0)


class TestFriction(unittest.TestCase):
    def test_low_friction_slips(self):
        res = runs.run_g3()
        low = [lv for lv in res["levels"] if lv["friction"] <= 0.3]
        for lv in low:
            self.assertLess(lv["lift_mm"], 20.0)


class TestRelease(unittest.TestCase):
    def test_drops_after_release(self):
        res = runs.run_g4()
        self.assertGreater(res["lift_mm"], 50.0)
        self.assertGreater(res["drop_after_release_s"], 0.0)
        self.assertLess(res["drop_after_release_s"], 1.0)


class TestForceLoop(unittest.TestCase):
    def test_tracks_target_force(self):
        res = extra.run_g5()
        for lv in res["levels"]:
            self.assertAlmostEqual(lv["steady_n"], lv["target_n"], delta=0.2)
            self.assertGreater(lv["rise_time_s"], 0.0)
            self.assertLess(lv["rise_time_s"], 1.5)
            self.assertEqual(lv["force_source"], "mj_contactForce_bilateral_sum")
            self.assertGreater(lv["contact_points"], 0)
            self.assertLess(lv["final_activation"], 0.99)

    def test_eight_newton_target_remains_unsaturated(self):
        result = extra.run_g5(targets=(8.0,))["levels"][0]
        self.assertAlmostEqual(result["steady_n"], 8.0, delta=0.2)
        self.assertLess(result["final_activation"], 0.99)
        self.assertGreater(result["contact_points"], 0)


class TestForcePI(unittest.TestCase):
    def test_positive_error_does_not_wind_up_at_upper_saturation(self):
        activation, integral = extra._force_pi_step(
            target_force=10.0,
            measured_force=0.0,
            integral=0.9,
        )
        self.assertEqual(activation, 1.0)
        self.assertEqual(integral, 0.9)

    def test_negative_error_can_unwind_from_upper_saturation(self):
        activation, integral = extra._force_pi_step(
            target_force=0.0,
            measured_force=0.1,
            integral=0.9,
        )
        self.assertEqual(activation, 1.0)
        self.assertLess(integral, 0.9)

    def test_negative_error_does_not_wind_up_at_lower_saturation(self):
        activation, integral = extra._force_pi_step(
            target_force=0.0,
            measured_force=10.0,
            integral=-0.8,
        )
        self.assertEqual(activation, 0.0)
        self.assertEqual(integral, -0.8)

    def test_positive_error_can_recover_from_lower_saturation(self):
        activation, integral = extra._force_pi_step(
            target_force=10.0,
            measured_force=0.0,
            integral=-0.8,
        )
        self.assertEqual(activation, 0.0)
        self.assertGreater(integral, -0.8)


class TestSlipReflex(unittest.TestCase):
    def test_reflex_recovers_grasp(self):
        res = extra.run_g6()
        self.assertLess(res["no_reflex"]["lift_mm"], 5.0)
        self.assertGreater(res["with_reflex"]["lift_mm"], 50.0)
        self.assertGreater(res["with_reflex"]["reflex_steps"], 0)


class TestLoadSweep(unittest.TestCase):
    def test_all_masses_grip(self):
        res = extra.run_g7()
        for lv in res["levels"]:
            self.assertTrue(lv["gripped"])
            self.assertIsNotNone(lv["min_activation"])


class TestArmGrasp(unittest.TestCase):
    def test_arm_lifts_object(self):
        res = extra.run_g8()
        self.assertGreater(res["lift_mm"], 50.0)


if __name__ == "__main__":
    unittest.main()
