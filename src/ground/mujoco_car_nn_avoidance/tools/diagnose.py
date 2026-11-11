#!/usr/bin/env python3
"""诊断：真实跑一个场景的 NN 闭环，打印轨迹与关键量，定位失败原因。"""
import math
import sys
from pathlib import Path

import numpy as np

MOD = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(MOD))

from mujoco_car_nn_avoidance.config import Config
from mujoco_car_nn_avoidance.car_env import CarEnv
from mujoco_car_nn_avoidance.sensors import ray_scan, normalize_rays
from mujoco_car_nn_avoidance.model import MLP

RESULTS = MOD / "results"


def diagnose(scene, seconds=15.0, every=25):
    cfg = Config()
    net = MLP.load(RESULTS / "mlp_weights.npz")
    env = CarEnv(cfg, scene)
    env.reset()
    n = int(seconds * cfg.control_hz)
    print(f"== {scene} ==")
    for step in range(n):
        rays = ray_scan(env)
        yaw = env.yaw()
        state = np.concatenate([
            normalize_rays(rays, cfg.lidar_range_m),
            [float(np.clip(yaw / math.pi, -1.0, 1.0))],
        ]).astype(np.float32)
        steer, th = net.predict(state)
        wl, wr = env.desired_wheel_vel(steer, th)
        env.step_vel(wl, wr)
        if step % every == 0:
            pos = env.pos()
            print(f"t={step/cfg.control_hz:5.1f}s  x={pos[0]:6.2f} y={pos[1]:6.2f} "
                  f"yaw={yaw:6.2f} steer={float(steer):5.2f} th={float(th):.2f} "
                  f"dmin={rays.min():4.2f} dcen={rays[12]:4.2f}")
    pos = env.pos()
    print(f"最终 x={pos[0]:.2f}（goal={cfg.goal_x}）")


if __name__ == "__main__":
    scene = sys.argv[1] if len(sys.argv) > 1 else "car_scene_basic.xml"
    diagnose(scene)
