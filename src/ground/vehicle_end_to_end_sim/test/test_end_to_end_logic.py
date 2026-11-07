"""vehicle_end_to_end_sim 单元测试 —— 不依赖 pygame，可无头运行。

覆盖四层纯逻辑：
    kinematics : 自行车运动学模型数值正确性
    perception : 前方视野锥测距与侧/后方障碍过滤
    decision   : 安全区 / 预警区 / 刹车区三级状态机与速度行为
    control    : 转向回正与限幅

运行：python3 test/test_end_to_end_logic.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vehicle_end_to_end_sim import config
from vehicle_end_to_end_sim.kinematics import Car
from vehicle_end_to_end_sim.perception import compute_nearest_front_distance
from vehicle_end_to_end_sim.decision import decide, DriveState
from vehicle_end_to_end_sim.control import update_steering

import unittest


class TestKinematics(unittest.TestCase):
    """3.1 自行车运动学模型。"""

    def test_velocity_zero_position_stays(self):
        car = Car(x=0.0, y=0.0, theta=0.0)
        car.v_pixel = 0.0
        car.update()
        self.assertAlmostEqual(car.x, 0.0)
        self.assertAlmostEqual(car.y, 0.0)
        self.assertAlmostEqual(car.theta, 0.0)

    def test_straight_motion_along_heading(self):
        car = Car(x=0.0, y=0.0, theta=0.0)
        car.v_pixel = 2.0
        car.steer = 0.0
        car.update()
        # theta=0 时沿 +x 前进：x += v*cos(0)=v，y 不变
        self.assertAlmostEqual(car.x, 2.0)
        self.assertAlmostEqual(car.y, 0.0)
        self.assertAlmostEqual(car.theta, 0.0)

    def test_heading_rotates_with_steer(self):
        car = Car(x=0.0, y=0.0, theta=0.0)
        car.v_pixel = 1.0
        car.steer = 1.0  # delta = pi/4，d_theta = (v/L)*tan(pi/4) = 1/7
        car.update()
        expected_dtheta = (car.v_pixel / config.WHEELBASE_PIXEL) * math.tan(
            car.steer * config.MAX_STEER_RAD)
        self.assertAlmostEqual(car.theta, expected_dtheta, places=10)


class TestPerception(unittest.TestCase):
    """3.2 前方视野锥感知。"""

    def test_nearest_in_front_cone(self):
        # 车头朝上（theta=-pi/2，指向 -y 方向），正前方障碍位于更小的 y
        car = Car(x=0.0, y=0.0, theta=-math.pi / 2)
        obstacles = [(0, -100), (0, -300)]
        dist = compute_nearest_front_distance(car, obstacles)
        self.assertAlmostEqual(dist, 100.0)

    def test_filters_side_and_rear(self):
        # 侧方（±90°）与后方障碍应被过滤 → 返回 inf
        car = Car(x=0.0, y=0.0, theta=-math.pi / 2)  # 朝上（-y）
        obstacles = [(100, 0), (0, 100), (-100, 0)]  # 右 / 后 / 左
        dist = compute_nearest_front_distance(car, obstacles)
        self.assertTrue(math.isinf(dist))

    def test_takes_only_nearest_in_cone(self):
        car = Car(x=0.0, y=0.0, theta=-math.pi / 2)
        obstacles = [(0, -50), (0, -20), (5, -200)]
        dist = compute_nearest_front_distance(car, obstacles)
        self.assertAlmostEqual(dist, 20.0)


class TestDecision(unittest.TestCase):
    """3.3 三级状态机（AEB 平滑减速）。"""

    def test_safe_zone_accelerates_toward_target(self):
        car = Car()
        car.v_pixel = 0.5
        target = 1.0
        state, brake = decide(car, target, min_dist=500.0)
        self.assertIs(state, DriveState.SAFE)
        self.assertFalse(brake)
        self.assertAlmostEqual(car.v_pixel, 0.5 + config.ACCEL_STEP)

    def test_safe_zone_holds_at_target(self):
        car = Car()
        car.v_pixel = 1.0
        target = 1.0
        state, _ = decide(car, target, min_dist=500.0)
        self.assertAlmostEqual(car.v_pixel, target)

    def test_warn_zone_keeps_speed(self):
        # 预警区（180 <= dist < 300）：速度保持不变，只提示，不刹车
        car = Car()
        car.v_pixel = 0.5
        target = 1.0
        state, brake = decide(car, target, min_dist=200.0)
        self.assertIs(state, DriveState.WARN)
        self.assertFalse(brake)
        self.assertAlmostEqual(car.v_pixel, 0.5)  # 机器不抢方向盘

    def test_brake_zone_decelerates(self):
        car = Car()
        car.v_pixel = 1.0
        state, brake = decide(car, 1.0, min_dist=100.0)
        self.assertIs(state, DriveState.BRAKE)
        self.assertTrue(brake)
        self.assertAlmostEqual(car.v_pixel, 1.0 - config.BRAKE_DECEL)

    def test_brake_never_goes_negative(self):
        car = Car()
        car.v_pixel = 0.02
        for _ in range(10):
            state, _ = decide(car, 1.0, min_dist=100.0)
        self.assertGreaterEqual(car.v_pixel, 0.0)


class TestControl(unittest.TestCase):
    """键盘转向 + 平滑回正 + 限幅。"""

    def test_steer_left_decreases(self):
        car = Car()
        car.steer = 0.0
        update_steering(True, False, car)
        self.assertAlmostEqual(car.steer, -config.STEER_SPEED)

    def test_steer_right_increases(self):
        car = Car()
        car.steer = 0.0
        update_steering(False, True, car)
        self.assertAlmostEqual(car.steer, config.STEER_SPEED)

    def test_steer_clamped_to_one(self):
        car = Car()
        car.steer = 0.99
        for _ in range(5):
            update_steering(False, True, car)
        self.assertLessEqual(car.steer, config.MAX_STEER)

    def test_auto_return_to_zero(self):
        car = Car()
        car.steer = 0.5
        for _ in range(100):
            update_steering(False, False, car)
        self.assertAlmostEqual(car.steer, 0.0)


class TestUnitConversion(unittest.TestCase):
    def test_kmh_to_pixel_roundtrip(self):
        kmh = 60.0
        px = config.kmh_to_pixel(kmh)
        back = config.pixel_to_kmh(px)
        self.assertAlmostEqual(back, kmh, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
