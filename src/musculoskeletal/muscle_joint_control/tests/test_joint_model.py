# -*- coding: utf-8 -*-
"""单自由度肌肉关节的单元测试。"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from joint_model import (  # noqa: E402
    JointParams,
    MuscleJoint,
    evaluate,
    simulate_step_response,
)


# ===========================================================================
class TestJointParams(unittest.TestCase):
    def test_defaults_are_sane(self):
        p = JointParams()
        self.assertGreater(p.inertia, 0)
        self.assertGreaterEqual(p.damping, 0)
        self.assertGreater(p.moment_arm, 0)
        self.assertGreater(p.f_max, 0)

    def test_reject_non_positive_inertia(self):
        with self.assertRaises(ValueError):
            JointParams(inertia=0.0)
        with self.assertRaises(ValueError):
            JointParams(inertia=-1.0)

    def test_reject_negative_damping(self):
        with self.assertRaises(ValueError):
            JointParams(damping=-0.1)

    def test_allow_zero_damping(self):
        self.assertEqual(JointParams(damping=0.0).damping, 0.0)

    def test_reject_non_positive_moment_arm(self):
        with self.assertRaises(ValueError):
            JointParams(moment_arm=0.0)

    def test_reject_non_positive_lengths(self):
        with self.assertRaises(ValueError):
            JointParams(rest_length=0.0)
        with self.assertRaises(ValueError):
            JointParams(l_opt=-0.1)

    def test_muscle_params_carries_over(self):
        p = JointParams(f_max=77.0, l_opt=0.08, v_max=0.9,
                        tau_act=0.012, tau_deact=0.05)
        mp = p.muscle_params()
        self.assertAlmostEqual(mp.f_max, 77.0)
        self.assertAlmostEqual(mp.l_opt, 0.08)
        self.assertAlmostEqual(mp.v_max, 0.9)
        self.assertAlmostEqual(mp.tau_act, 0.012)
        self.assertAlmostEqual(mp.tau_deact, 0.05)


# ===========================================================================
class TestMuscleKinematics(unittest.TestCase):
    """肌肉长度/速度与关节角的关系。"""

    def setUp(self):
        self.p = JointParams(rest_length=0.10, l_opt=0.10, moment_arm=0.03)
        self.j = MuscleJoint(self.p)

    def test_zero_angle_both_at_rest_length(self):
        lf, vf, le, ve = self.j.muscle_state()
        self.assertAlmostEqual(lf, 1.0, places=9)
        self.assertAlmostEqual(le, 1.0, places=9)
        self.assertAlmostEqual(vf, 0.0, places=9)
        self.assertAlmostEqual(ve, 0.0, places=9)

    def test_positive_angle_shortens_flexor(self):
        self.j.theta = 0.2
        lf, _vf, le, _ve = self.j.muscle_state()
        self.assertLess(lf, 1.0)      # 屈肌缩短
        self.assertGreater(le, 1.0)   # 伸肌被拉长

    def test_negative_angle_shortens_extensor(self):
        self.j.theta = -0.2
        lf, _vf, le, _ve = self.j.muscle_state()
        self.assertGreater(lf, 1.0)
        self.assertLess(le, 1.0)

    def test_lengths_sum_to_twice_rest(self):
        """小角度线性化下两条肌长度之和恒定。"""
        for th in (-0.3, -0.1, 0.0, 0.1, 0.3):
            self.j.theta = th
            lf, _vf, le, _ve = self.j.muscle_state()
            self.assertAlmostEqual(lf + le, 2.0, places=9)

    def test_positive_angular_velocity_shortens_flexor(self):
        self.j.theta_dot = 1.0
        _lf, vf, _le, ve = self.j.muscle_state()
        self.assertGreater(vf, 0.0)   # 缩短速度为正
        self.assertLess(ve, 0.0)      # 伸肌被拉长（离心）

    def test_velocity_signs_opposite(self):
        self.j.theta_dot = 0.7
        _lf, vf, _le, ve = self.j.muscle_state()
        self.assertAlmostEqual(vf, -ve, places=12)


# ===========================================================================
class TestJointDynamics(unittest.TestCase):
    def test_reject_non_positive_dt(self):
        j = MuscleJoint()
        with self.assertRaises(ValueError):
            j.step(0.5, 0.5, 0.0)
        with self.assertRaises(ValueError):
            j.step(0.5, 0.5, -0.01)

    def test_equal_activation_produces_no_motion(self):
        """两侧同样激活时力相互抵消，关节不动（无初始速度时）。"""
        j = MuscleJoint()
        for _ in range(200):
            j.step(0.5, 0.5, 0.001)
        self.assertAlmostEqual(j.theta, 0.0, places=6)
        self.assertAlmostEqual(j.theta_dot, 0.0, places=6)

    def test_flexor_activation_moves_positive(self):
        j = MuscleJoint()
        for _ in range(300):
            j.step(1.0, 0.0, 0.001)
        self.assertGreater(j.theta, 0.0)

    def test_extensor_activation_moves_negative(self):
        j = MuscleJoint()
        for _ in range(300):
            j.step(0.0, 1.0, 0.001)
        self.assertLess(j.theta, 0.0)

    def test_damping_dissipates_motion(self):
        """无激活时，初速度最终被阻尼耗散掉。"""
        j = MuscleJoint(JointParams(damping=0.2), theta=0.0, theta_dot=1.0)
        for _ in range(4000):
            j.step(0.0, 0.0, 0.001)
        self.assertLess(abs(j.theta_dot), 1e-3)

    def test_reset_clears_state(self):
        j = MuscleJoint(theta=0.5, theta_dot=0.3)
        j.record(0.0)
        j.reset()
        self.assertAlmostEqual(j.theta, 0.0)
        self.assertAlmostEqual(j.theta_dot, 0.0)
        self.assertEqual(len(j.history["t"]), 0)

    def test_record_appends_one_sample(self):
        j = MuscleJoint()
        for i in range(5):
            j.step(0.4, 0.1, 0.001)
            j.record(i * 0.001)
        self.assertEqual(len(j.history["t"]), 5)
        self.assertEqual(len(j.history["theta"]), 5)
        self.assertEqual(len(j.history["a_flex"]), 5)

    def test_activation_stays_in_range_during_run(self):
        j = MuscleJoint()
        for _ in range(1000):
            j.step(1.0, 1.0, 0.001)
            self.assertGreaterEqual(j.flexor.activation, 0.0)
            self.assertLessEqual(j.flexor.activation, 1.0)
            self.assertGreaterEqual(j.extensor.activation, 0.0)
            self.assertLessEqual(j.extensor.activation, 1.0)


# ===========================================================================
class TestStepResponse(unittest.TestCase):
    def test_rejects_bad_arguments(self):
        with self.assertRaises(ValueError):
            simulate_step_response(0.3, dt=0.0)
        with self.assertRaises(ValueError):
            simulate_step_response(0.3, duration=0.0)

    def test_converges_toward_target(self):
        res = simulate_step_response(0.3, duration=1.5, dt=0.001)
        m = res["metrics"]
        self.assertLess(abs(m["steady_state_error"]), 0.05)
        self.assertGreater(m["final_angle"], 0.2)

    def test_tracks_negative_target(self):
        res = simulate_step_response(-0.25, duration=1.5, dt=0.001)
        self.assertLess(res["metrics"]["final_angle"], -0.15)

    def test_rise_time_is_reported_and_positive(self):
        m = simulate_step_response(0.3, duration=1.5, dt=0.001)["metrics"]
        self.assertIsNotNone(m["rise_time"])
        self.assertGreater(m["rise_time"], 0.0)
        self.assertLess(m["rise_time"], 1.5)

    def test_zero_target_stays_put(self):
        res = simulate_step_response(0.0, duration=0.5, dt=0.001)
        self.assertAlmostEqual(res["metrics"]["final_angle"], 0.0, places=5)

    def test_larger_target_needs_more_activation(self):
        m_small = simulate_step_response(0.1, duration=1.2, dt=0.001)["metrics"]
        m_big = simulate_step_response(0.6, duration=1.2, dt=0.001)["metrics"]
        self.assertGreater(m_big["max_activation_flex"],
                           m_small["max_activation_flex"])

    def test_torque_is_saturated_by_muscle_capacity(self):
        """目标很大时实际力矩不应超过肌肉上限。"""
        p = JointParams()
        tau_max = p.f_max * p.moment_arm
        m = simulate_step_response(1.0, duration=1.5, dt=0.001,
                                   joint_params=p)["metrics"]
        # 允许一点数值余量（被动力、共激活会略增）
        self.assertLess(m["max_torque"], tau_max * 1.35)

    def test_history_length_matches_simulated_steps(self):
        dt, dur = 0.002, 0.4
        h = simulate_step_response(0.2, duration=dur, dt=dt)["history"]
        self.assertEqual(len(h["t"]), int(round(dur / dt)))
        self.assertAlmostEqual(h["t"][1] - h["t"][0], dt, places=12)

    def test_coactivation_increases_steady_state_error(self):
        """共激活让两侧肌肉同时出力，形成对抗，稳态误差随之增大。"""
        low = simulate_step_response(0.35, duration=1.5, dt=0.001,
                                     co_activation=0.0)["metrics"]
        high = simulate_step_response(0.35, duration=1.5, dt=0.001,
                                      co_activation=0.6)["metrics"]
        self.assertGreater(abs(high["steady_state_error"]),
                           abs(low["steady_state_error"]))

    def test_coactivation_slows_the_response(self):
        """对抗越强，达到目标所需时间越长。"""
        low = simulate_step_response(0.35, duration=1.5, dt=0.001,
                                     co_activation=0.0)["metrics"]
        high = simulate_step_response(0.35, duration=1.5, dt=0.001,
                                      co_activation=0.6)["metrics"]
        self.assertGreater(high["rise_time"], low["rise_time"])

    def test_higher_gain_reduces_steady_state_error(self):
        """在肌肉能力范围内提高增益能减小稳态误差。"""
        weak = simulate_step_response(0.3, duration=1.5, dt=0.001,
                                      kp=4.0)["metrics"]
        strong = simulate_step_response(0.3, duration=1.5, dt=0.001,
                                        kp=9.0)["metrics"]
        self.assertLess(abs(strong["steady_state_error"]),
                        abs(weak["steady_state_error"]))

    def test_excessive_gain_causes_oscillation(self):
        """增益过大时力矩指令超过肌肉上限，反而引起振荡。

        这是肌肉驱动与理想力矩源的差别：力矩源的输出能力无上限，
        而肌肉最多只能全力收缩，超出部分会被饱和掉。
        """
        gentle = simulate_step_response(0.3, duration=1.5, dt=0.001,
                                        kp=9.0)["metrics"]
        harsh = simulate_step_response(0.3, duration=1.5, dt=0.001,
                                       kp=40.0)["metrics"]
        self.assertGreater(harsh["overshoot_percent"],
                           gentle["overshoot_percent"])


# ===========================================================================
class TestEvaluate(unittest.TestCase):
    def test_raises_without_history(self):
        with self.assertRaises(ValueError):
            evaluate(MuscleJoint(), 0.3)

    def test_reports_all_keys(self):
        res = simulate_step_response(0.3, duration=0.8, dt=0.001)
        for key in ("final_angle", "steady_state_error", "rise_time",
                    "peak_angle", "overshoot_percent", "max_activation_flex",
                    "max_activation_ext", "max_torque", "steps"):
            self.assertIn(key, res["metrics"])

    def test_overshoot_is_non_negative(self):
        res = simulate_step_response(0.3, duration=1.0, dt=0.001)
        self.assertGreaterEqual(res["metrics"]["overshoot_percent"], 0.0)

    def test_steady_state_error_sign(self):
        """稳态值低于目标时误差应为负。"""
        res = simulate_step_response(0.3, duration=1.5, dt=0.001)
        self.assertLess(res["metrics"]["steady_state_error"], 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
