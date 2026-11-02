# -*- coding: utf-8 -*-
"""专家策略单测（不需要 MuJoCo）。

专家策略只负责数据采集阶段打标签：前方空旷时按航向偏差回正，前方有障碍时
朝更开放一侧转向，油门按最近障碍距离分级。这里验证这些分支的方向与档位。
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_nn_avoidance.config import Config  # noqa: E402
from mujoco_car_nn_avoidance.expert import expert_control  # noqa: E402


def full_rays(distance):
    """25 束射线全部取同一距离（米）。"""
    return np.full(25, distance, dtype=float)


class TestExpertCruise(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_open_road_keeps_cruise_throttle(self):
        steer, throttle = expert_control(full_rays(6.0), self.cfg, yaw=0.0)
        self.assertAlmostEqual(throttle, self.cfg.cruise_throttle, places=6)
        self.assertAlmostEqual(steer, 0.0, places=6)

    def test_open_road_corrects_heading(self):
        # 前方空旷时用航向偏差回正：yaw>0 应给负向 steer 拉回
        steer, throttle = expert_control(full_rays(6.0), self.cfg, yaw=0.2)
        self.assertLess(steer, 0.0)
        self.assertAlmostEqual(throttle, self.cfg.cruise_throttle, places=6)


class TestExpertAvoidance(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def _rays(self, center, left, right):
        rays = full_rays(6.0)
        # 参数名沿用历史：left 对应索引 0..9（车体右侧射线），right 对应 15..24（车体左侧射线）
        rays[10:15] = center       # 正前方窄带
        rays[:10] = left           # 车体右侧
        rays[15:] = right          # 车体左侧
        return rays

    def test_steers_toward_open_side(self):
        # 右侧开放、左侧近障碍 -> 朝右转（steer<0）
        rays_left_open = self._rays(center=2.0, left=5.5, right=1.0)
        steer_left, _ = expert_control(rays_left_open, self.cfg, yaw=0.0)
        self.assertLess(steer_left, 0.0)

        # 左侧开放、右侧近障碍 -> 朝左转（steer>0）
        rays_right_open = self._rays(center=2.0, left=1.0, right=5.5)
        steer_right, _ = expert_control(rays_right_open, self.cfg, yaw=0.0)
        self.assertGreater(steer_right, 0.0)

    def test_throttle_decreases_when_close(self):
        far = expert_control(full_rays(4.0), self.cfg, yaw=0.0)[1]
        mid = expert_control(full_rays(2.5), self.cfg, yaw=0.0)[1]
        near = expert_control(full_rays(0.8), self.cfg, yaw=0.0)[1]
        self.assertGreater(far, mid)
        self.assertGreater(mid, near)
        self.assertAlmostEqual(near, 0.20, places=6)

    def test_steer_is_saturated(self):
        rays = self._rays(center=0.8, left=5.5, right=0.5)
        steer, _ = expert_control(rays, self.cfg, yaw=0.0)
        self.assertLessEqual(abs(steer), 0.7 + 1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
