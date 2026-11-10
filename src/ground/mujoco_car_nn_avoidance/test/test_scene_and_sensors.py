# -*- coding: utf-8 -*-
"""场景与射线感知测试（需要 MuJoCo；未安装时自动跳过并说明原因）。

覆盖：
    1. 三个场景都能加载，执行器与几何体齐备；
    2. 射线不会打到自车（geom group 屏蔽 + bodyexclude 是否生效）；
    3. 给定轮速后车辆确实前进；
    4. 静置后等轮速直行不跑偏（打滑/偏航回归）。
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import mujoco  # noqa: F401

    MUJOCO_OK = True
    MUJOCO_HINT = ""
except Exception as exc:  # pragma: no cover
    MUJOCO_OK = False
    MUJOCO_HINT = "未安装 MuJoCo（{}），已跳过场景测试".format(exc)


@unittest.skipUnless(MUJOCO_OK, MUJOCO_HINT)
class TestSceneAndSensors(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from mujoco_car_nn_avoidance.config import Config
        from mujoco_car_nn_avoidance.car_env import CarEnv

        cls.cfg = Config()
        cls.envs = {}
        for scene in cls.cfg.scenes:
            env = CarEnv(cls.cfg, scene)
            env.reset()
            cls.envs[scene] = env

    def test_all_scenes_load_with_actuators(self):
        for scene, env in self.envs.items():
            self.assertGreater(env.model.ngeom, 5, scene)
            self.assertEqual(env.model.nu, 2, scene)

    def test_rays_do_not_hit_ego_body(self):
        from mujoco_car_nn_avoidance.sensors import ray_scan

        for scene, env in self.envs.items():
            env.reset()
            rays = ray_scan(env)
            # 打到自车轮子会返回极近距离；静止起步时任何一束都不应低于 0.2 m
            self.assertGreater(float(rays.min()), 0.2,
                               "{}: 射线疑似打到自车".format(scene))

    def test_forward_obstacle_is_detected(self):
        from mujoco_car_nn_avoidance.sensors import ray_scan

        # basic 场景正前方 3.2 m 处有障碍，中心束应命中而不是超量程
        env = self.envs["car_scene_basic.xml"]
        env.reset()
        rays = ray_scan(env)
        self.assertLess(float(rays[12]), self.cfg.lidar_range_m,
                        "正前方障碍未被中心束检测到")

    def test_wheel_command_moves_car_forward(self):
        env = self.envs["car_scene_basic.xml"]
        env.reset()
        start_x = env.pos()[0]
        env.set_wheel_speeds(20.0, 20.0)
        for _ in range(int(1.0 / self.cfg.timestep)):
            env.step(1)
        self.assertGreater(env.pos()[0] - start_x, 0.1, "给定轮速后车辆没有前进")

    def test_straight_driving_stays_lane(self):
        env = self.envs["car_scene_basic.xml"]
        env.reset()
        # 先静置建立轮胎接触，再等轮速直行 3 s
        env.set_wheel_speeds(0.0, 0.0)
        env.step(int(self.cfg.settle_s / self.cfg.timestep))
        env.set_wheel_speeds(20.0, 20.0)
        for _ in range(int(3.0 / self.cfg.timestep)):
            env.step(1)
        pos = env.pos()
        yaw = env.yaw()
        self.assertLess(abs(pos[1]), 0.25,
                        "直行横向漂移过大：y={:.3f} m".format(pos[1]))
        self.assertLess(abs(yaw), math.radians(5.0),
                        "直行偏航过大：{:.2f}°".format(math.degrees(yaw)))


if __name__ == "__main__":
    if not MUJOCO_OK:
        print(MUJOCO_HINT)
    unittest.main(verbosity=2)
