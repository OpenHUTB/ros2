# -*- coding: utf-8 -*-
"""闭环逻辑仿真单测（不需要 MuJoCo）。

用一个"理想速度跟踪"的一维运动学被控对象，把真实的 decision.decide() 与
controller.SpeedRamp 接成闭环，验证：
    - 巡航 -> 减速 -> 停车 的阈值链是否自洽；
    - 是否会停在障碍物之前（不依赖任何调参试错）；
    - 突发近距障碍与感知失效两条兜底路径是否生效。

它检验的是"决策+速度整形的设计"，不检验 MuJoCo 的接触与摩擦；
后者由 test_scene_and_sensors.py 在装有 MuJoCo 的环境里验证。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_avoidance.controller import RampParams, SpeedRamp  # noqa: E402
from mujoco_car_avoidance.decision import DecisionParams, State, decide  # noqa: E402

DT = 0.02          # 50 Hz，与 config.CONTROL_HZ 一致
T_MAX = 60.0


def simulate(initial_distance,
             params=DecisionParams(),
             ramp_params=RampParams(),
             sensor=None,
             dt=DT,
             t_max=T_MAX):
    """返回逐帧记录 [(t, d, v_cmd, state, x)]。

    sensor: 可选，函数 f(t, x) -> 距离或 None，用于模拟突发障碍/感知失效。
            默认为理想测距 d = initial_distance - x。
    """
    ramp = SpeedRamp(ramp_params)
    x = 0.0
    t = 0.0
    since_valid = float("inf")
    log = []

    while t < t_max:
        if sensor is None:
            distance = initial_distance - x
        else:
            distance = sensor(t, x)

        if distance is None:
            since_valid += dt
        else:
            since_valid = 0.0

        decision = decide(distance, since_valid, params)
        if decision.state in (State.FULL_BRAKE, State.STALE):
            ramp.reset(0.0)
            v_cmd = 0.0
        else:
            v_cmd = ramp.update(decision.target_speed, dt)

        log.append((t, distance, v_cmd, decision.state, x))
        x += v_cmd * dt
        t += dt

        # 足够接近停车目标后提前结束，避免长时间空跑
        if distance is not None and abs(v_cmd) < 1e-4 and t > 1.0:
            break

    return log


def first_state_time(log, state):
    for entry in log:
        if entry[3] is state:
            return entry[0]
    return None


class TestClosedLoopNominal(unittest.TestCase):
    """标称工况：7.5 m 外有静止障碍物。"""

    def setUp(self):
        self.params = DecisionParams()
        self.log = simulate(7.5, self.params)
        self.final_distance = self.log[-1][1]
        self.max_speed = max(entry[2] for entry in self.log)

    def test_never_collides(self):
        self.assertTrue(all(entry[1] > 0.0 for entry in self.log))

    def test_reaches_cruise_speed(self):
        self.assertAlmostEqual(self.max_speed, self.params.cruise_speed, places=2)

    def test_starts_cruising_first(self):
        self.assertEqual(self.log[0][3], State.CRUISE)

    def test_slowdown_and_stop_happen_in_order(self):
        t_slow = first_state_time(self.log, State.SLOW_DOWN)
        self.assertIsNotNone(t_slow, "应当出现减速阶段")
        # 减速之后速度必须单调下降（不允许二次加速）
        after_slow = [entry[2] for entry in self.log if entry[0] >= t_slow]
        for previous, current in zip(after_slow, after_slow[1:]):
            self.assertLessEqual(current, previous + 1e-9)

    def test_stops_near_target_distance(self):
        # 低速主动刹停使实际停车位置略大于 stop_target（不再蠕动着逼近目标），
        # 因此断言区间：不早于目标位置，且不超过目标位置 + 0.5 m。
        self.assertGreaterEqual(self.final_distance, self.params.stop_target_m - 1e-6)
        self.assertLessEqual(self.final_distance, self.params.stop_target_m + 0.5)

    def test_stops_within_reasonable_time(self):
        self.assertLess(self.log[-1][0], 12.0, "低速蠕动会让停车时间过长")

    def test_stops_before_emergency_threshold(self):
        self.assertGreater(self.final_distance, self.params.emergency_m)


class TestClosedLoopVariants(unittest.TestCase):
    """其它工况。"""

    def test_starts_inside_slow_zone(self):
        params = DecisionParams()
        log = simulate(3.0, params)
        self.assertGreater(log[-1][1], 0.0)
        self.assertGreaterEqual(log[-1][1], params.stop_target_m - 1e-6)
        self.assertLessEqual(log[-1][1], params.stop_target_m + 0.5)

    def test_already_closer_than_emergency_stays_still(self):
        params = DecisionParams()
        log = simulate(0.5, params)
        self.assertEqual(log[0][3], State.FULL_BRAKE)
        self.assertAlmostEqual(log[-1][1], 0.5, places=6)

    def test_stale_sensor_never_moves(self):
        params = DecisionParams()
        log = simulate(7.5, params, sensor=lambda t, x: None)
        self.assertAlmostEqual(log[-1][4], 0.0, places=9)   # 位置始终为 0
        self.assertEqual(log[-1][3], State.STALE)
        self.assertTrue(all(entry[2] == 0.0 for entry in log), "失效期间不应给出任何速度")

    def test_sudden_close_obstacle_triggers_emergency(self):
        params = DecisionParams()
        # 前 5 s 正常（无目标），随后在 0.6 m 处突然出现障碍物
        def sensor(t, x):
            return 7.5 - x if t < 5.0 else 0.6

        log = simulate(7.5, params, sensor=sensor)
        self.assertTrue(any(entry[3] is State.FULL_BRAKE for entry in log),
                        "突发近距障碍应触发急刹")
        self.assertGreater(log[-1][1], 0.0, "急刹后仍不应碰撞")

    def test_higher_cruise_speed_still_stops_safely(self):
        params = DecisionParams(cruise_speed=4.0)
        log = simulate(12.0, params)
        self.assertGreater(log[-1][1], 0.0)
        self.assertAlmostEqual(log[-1][1], params.stop_target_m, delta=0.6)


if __name__ == "__main__":
    # 直接运行时额外打印一组标称工况数据，便于写入文档的"性能评价"一节
    unittest.main(verbosity=2, exit=False)

    params = DecisionParams()
    log = simulate(7.5, params)
    speeds = [entry[2] for entry in log]
    print("\n[标称工况] 初始间距 {:.2f} m，控制频率 {:.0f} Hz".format(7.5, 1.0 / DT))
    print("  进入减速时刻      t = {:.2f} s (d = {:.2f} m)".format(
        first_state_time(log, State.SLOW_DOWN),
        next(entry[1] for entry in log if entry[3] is State.SLOW_DOWN)))
    print("  峰值速度          v = {:.3f} m/s".format(max(speeds)))
    print("  收敛停车间距      d = {:.3f} m".format(log[-1][1]))
    print("  停车耗时          t = {:.2f} s".format(log[-1][0]))
    print("  全程最小间距      d = {:.3f} m".format(min(entry[1] for entry in log)))
