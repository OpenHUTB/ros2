"""纯 Python 单测，不依赖 ROS。"""
import math
import unittest
from ugv_slam_nav.nav_logic import clamp, relay, sector_speed, Limits


class TestClamp(unittest.TestCase):
    def test_in_range(self):
        self.assertEqual(clamp(0.3, -0.5, 0.5), 0.3)

    def test_high(self):
        self.assertEqual(clamp(1.0, -0.5, 0.5), 0.5)

    def test_low(self):
        self.assertEqual(clamp(-1.0, -0.5, 0.5), -0.5)


class TestRelay(unittest.TestCase):
    def test_nav2_alive(self):
        vx, yaw = relay(0.3, 0.5, 0.0, 0.0, True)
        self.assertEqual((vx, yaw), (0.3, 0.5))

    def test_nav2_dead(self):
        vx, yaw = relay(0.3, 0.5, 0.1, 0.2, False)
        self.assertEqual((vx, yaw), (0.1, 0.2))

    def test_clamp_nav2(self):
        vx, yaw = relay(1.0, 2.0, 0.0, 0.0, True)
        self.assertEqual((vx, yaw), (0.5, 1.0))

    def test_custom_limits(self):
        lim = Limits(max_vx=1.0, max_yaw=2.0)
        vx, yaw = relay(1.5, 3.0, 0.0, 0.0, True, lim)
        self.assertEqual((vx, yaw), (1.0, 2.0))


class TestSectorSpeed(unittest.TestCase):
    def test_empty_scan(self):
        self.assertEqual(sector_speed([], -math.pi/2, math.pi/180), 0.4)

    def test_obstacle_too_close(self):
        # 正前方 0.2m 障碍
        ranges = [0.2] * 10
        vx = sector_speed(ranges, -0.05, 0.01, fov_deg=10)
        self.assertEqual(vx, 0.0)

    def test_far_cruise(self):
        ranges = [5.0] * 20
        vx = sector_speed(ranges, -0.1, 0.01, fov_deg=20)
        self.assertAlmostEqual(vx, 0.4, places=3)

    def test_slow_zone(self):
        # 正前方 0.6m（在 safe=0.4 和 slow=0.8 之间）
        ranges = [0.6] * 10
        vx = sector_speed(ranges, -0.05, 0.01, fov_deg=10,
                          safe_dist=0.4, slow_dist=0.8, cruise=0.4)
        # (0.6-0.4)/(0.8-0.4) = 0.5, *0.4 = 0.2
        self.assertAlmostEqual(vx, 0.2, places=3)

    def test_nan_ignored(self):
        ranges = [float("nan"), float("inf"), 5.0]
        vx = sector_speed(ranges, 0.0, 0.01, fov_deg=5)
        self.assertAlmostEqual(vx, 0.4, places=3)

    def test_side_obstacle_ignored(self):
        # 障碍在侧面（角度超扇区）
        ranges = [0.2] * 5 + [5.0] * 5
        # angle_min=-0.5, 中间那几个在正前方
        vx = sector_speed(ranges, -0.5, 0.1, fov_deg=10)
        self.assertAlmostEqual(vx, 0.4, places=3)


if __name__ == "__main__":
    unittest.main()
