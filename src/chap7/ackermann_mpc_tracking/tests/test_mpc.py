"""MPC 轨迹跟踪单元测试。

覆盖：车辆模型几何正确性、误差投影符号、QP 求解的最优性与约束满足、
闭环收敛、以及与 Stanley 基线的定量对比。全部不依赖 ROS / numpy。
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))

import mpc as M
import simulate as S
import stanley_baseline as SB


class TestBicycle(unittest.TestCase):
    def test_straight_line(self):
        car = M.KinematicBicycle(wheelbase=2.5)
        x, y, psi = car.step((0.0, 0.0, 0.0), v=2.0, delta=0.0, dt=0.5)
        self.assertAlmostEqual(x, 1.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(psi, 0.0, places=9)

    def test_constant_steer_gives_circle(self):
        car = M.KinematicBicycle(wheelbase=2.5)
        delta = 0.3
        radius = 2.5 / math.tan(delta)      # 后轴的转弯半径
        state = (0.0, 0.0, 0.0)
        dt = 0.01
        for _ in range(2000):
            state = car.step(state, v=1.0, delta=delta, dt=dt)
            # 圆心在左侧 (0, radius)
            r = math.hypot(state[0] - 0.0, state[1] - radius)
            self.assertAlmostEqual(r, radius, places=6)

    def test_steer_is_clipped(self):
        car = M.KinematicBicycle(wheelbase=2.5, delta_max=0.4)
        a = car.step((0.0, 0.0, 0.0), 1.0, 0.4, 0.1)
        b = car.step((0.0, 0.0, 0.0), 1.0, 5.0, 0.1)   # 远超限幅
        self.assertAlmostEqual(a[2], b[2], places=12)

    def test_rk4_converges_second_order(self):
        """步长减半，误差应降到约 1/4（RK4 的实际阶数受模型平滑性限制）。"""
        car = M.KinematicBicycle(wheelbase=2.5)
        v, delta = 3.0, 0.4

        def integrate(dt):
            state = (0.0, 0.0, 0.0)
            for _ in range(int(round(1.0 / dt))):
                state = car.step(state, v, delta, dt)
            return state

        s1 = integrate(0.1)
        s2 = integrate(0.05)
        s4 = integrate(0.025)
        e1 = math.hypot(s1[0] - s4[0], s1[1] - s4[1])
        e2 = math.hypot(s2[0] - s4[0], s2[1] - s4[1])
        self.assertGreater(e1, e2)
        self.assertLess(e2 / e1, 0.45)      # 至少是二阶收敛

    def test_curvature_to_steer(self):
        car = M.KinematicBicycle(wheelbase=2.5)
        kappa = 0.2
        delta = car.delta_of_curvature(kappa)
        self.assertAlmostEqual(math.tan(delta) / 2.5, kappa, places=12)


class TestPathTracker(unittest.TestCase):
    def setUp(self):
        self.path = S.build_s_curve()

    def test_on_path_has_zero_error(self):
        tr = M.PathTracker(self.path, window=40.0)
        x, y, th, _k = self.path.pose_at(5.0)
        info = tr.update(x, y)
        self.assertAlmostEqual(info['e_y'], 0.0, places=6)
        self.assertAlmostEqual(info['s'], 5.0, delta=0.05)

    def test_left_offset_is_positive(self):
        # 路径沿 +x，左法向为 +y，因此 y 偏大时 e_y 应为正
        tr = M.PathTracker(self.path, window=40.0)
        x, y, th, _k = self.path.pose_at(5.0)
        info = tr.update(x, y + 0.7)
        self.assertAlmostEqual(info['e_y'], 0.7, delta=0.02)

    def test_right_offset_is_negative(self):
        tr = M.PathTracker(self.path, window=40.0)
        x, y, th, _k = self.path.pose_at(5.0)
        info = tr.update(x, y - 0.4)
        self.assertAlmostEqual(info['e_y'], -0.4, delta=0.02)

    def test_curvature_of_reference(self):
        info = M.PathTracker(self.path, window=40.0).update(*self.path.pose_at(8.0)[:2])
        self.assertAlmostEqual(abs(info['kappa']), 1.0 / 6.0, places=6)


class TestQP(unittest.TestCase):
    def setUp(self):
        self.mpc = M.LinearMPC(dt=0.05, horizon=12, wheelbase=2.5,
                               q=(10.0, 5.0), r=0.5, r_rate=40.0,
                               delta_max=0.6, v_ref=5.0, rate_max=1.6)

    def test_converges(self):
        d, info = self.mpc.solve([0.8, 0.0], [0.0] * 12, delta_prev=0.0)
        self.assertLess(info['diff'], 1e-9)
        self.assertEqual(len(d), 12)

    def test_beats_zero_control(self):
        d, _info = self.mpc.solve([0.8, 0.0], [0.0] * 12, delta_prev=0.0)
        j_opt = self.mpc.cost([0.8, 0.0], d, [0.0] * 12)
        j_zero = self.mpc.cost([0.8, 0.0], [0.0] * 12, [0.0] * 12)
        self.assertLess(j_opt, j_zero)

    def test_beats_random_perturbations(self):
        """在最优解附近随机扰动，代价都不应更低——说明解确实是最小点。"""
        d, _info = self.mpc.solve([0.5, 0.1], [0.1] * 12, delta_prev=0.05)
        j_opt = self.mpc.cost([0.5, 0.1], d, [0.1] * 12, 0.05)
        seed = 12345
        for _ in range(30):
            pert = []
            for i in range(12):
                seed = (1103515245 * seed + 12345) % (2 ** 31)
                noise = (seed / float(2 ** 31) - 0.5) * 0.4
                pert.append(max(-self.mpc.du_max,
                                min(self.mpc.du_max, d[i] + noise)))
            j = self.mpc.cost([0.5, 0.1], pert, [0.1] * 12, 0.05)
            self.assertLessEqual(j_opt, j + 1e-9)

    def test_rate_limit_respected(self):
        d, _info = self.mpc.solve([1.2, 0.0], [0.0] * 12, delta_prev=0.0)
        for step in d:
            self.assertLessEqual(abs(step), self.mpc.du_max + 1e-12)

    def test_amplitude_limit_respected(self):
        d, _info = self.mpc.solve([2.0, 0.5], [0.0] * 12, delta_prev=0.0)
        deltas = self.mpc.delta_sequence(d, 0.0)
        for delta in deltas:
            self.assertLessEqual(abs(delta), self.mpc.delta_max + 1e-12)

    def test_zero_error_zero_reference_keeps_wheel_straight(self):
        d, _info = self.mpc.solve([0.0, 0.0], [0.0] * 12, delta_prev=0.0)
        self.assertLess(max(abs(x) for x in d), 1e-6)

    def test_error_sign_drives_negative_steer(self):
        # 车在路径左侧（e_y > 0）必须往右打方向（delta < 0）
        d, _info = self.mpc.solve([0.8, 0.0], [0.0] * 12, delta_prev=0.0)
        deltas = self.mpc.delta_sequence(d, 0.0)
        self.assertLess(deltas[0], 0.0)


class TestClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = S.build_s_curve()
        cls.pose = (0.0, 0.8, 0.0)
        cls.tm, cls.mm = S.run_mpc(cls.path, cls.pose)
        cls.ts, cls.ms = S.run_stanley(cls.path, cls.pose)

    def test_mpc_converges_to_path(self):
        self.assertLess(self.mm['final_err'], 0.01)
        self.assertTrue(self.mm['completed'])

    def test_mpc_does_not_overshoot_initial_offset(self):
        # 初始偏差 0.8 m，最大误差不应超过它（不超调）
        self.assertLessEqual(self.mm['max_err'], 0.8 + 1e-6)

    def test_mpc_beats_stanley_on_rmse(self):
        self.assertLess(self.mm['rmse'], self.ms['rmse'] * 0.6)

    def test_mpc_keeps_steering_inside_limits(self):
        for p in self.tm:
            self.assertLessEqual(abs(p['delta']), S.DELTA_MAX + 1e-9)
        rates = [abs(self.tm[i]['delta'] - self.tm[i - 1]['delta']) / S.DT
                 for i in range(1, len(self.tm))]
        self.assertLessEqual(max(rates), 1.6 + 1e-6)

    def test_stanley_also_completes(self):
        self.assertTrue(self.ms['completed'])


class TestStanley(unittest.TestCase):
    def test_correction_direction(self):
        ctrl = SB.StanleyController(wheelbase=2.5, k=1.5, rate_max=100.0)
        # 偏左 -> 往右打（负）
        self.assertLess(ctrl.control(0.5, 0.0, 0.0, 2.0, 0.05), 0.0)
        ctrl.reset()
        # 偏右 -> 往左打（正）
        self.assertGreater(ctrl.control(-0.5, 0.0, 0.0, 2.0, 0.05), 0.0)

    def test_feedforward_on_curve(self):
        ctrl = SB.StanleyController(wheelbase=2.5, k=1.5, rate_max=100.0)
        delta = ctrl.control(0.0, 0.0, 0.2, 2.0, 0.05)
        self.assertAlmostEqual(delta, math.atan(2.5 * 0.2), places=6)

    def test_saturation(self):
        ctrl = SB.StanleyController(wheelbase=2.5, k=50.0, rate_max=100.0)
        self.assertLessEqual(ctrl.control(10.0, 0.0, 0.0, 2.0, 0.05), 0.6 + 1e-12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
