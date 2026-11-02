#!/usr/bin/env python3
"""专家闭环诊断：查看专家策略下车的真实轨迹，核对标签覆盖的行驶状态。"""
import sys
from pathlib import Path

MOD = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(MOD))

from mujoco_car_nn_avoidance.config import Config
from mujoco_car_nn_avoidance.car_env import CarEnv
from mujoco_car_nn_avoidance.sensors import ray_scan
from mujoco_car_nn_avoidance.expert import expert_control


def main(scene="car_scene_basic.xml", seconds=15.0, every=50):
    cfg = Config()
    env = CarEnv(cfg, scene)
    env.reset()
    n = int(seconds * cfg.control_hz)
    print(f"== expert 闭环：{scene} ==")
    for step in range(n):
        rays = ray_scan(env)
        steer, th = expert_control(rays, cfg, env.yaw())
        wl, wr = env.desired_wheel_vel(steer, th)
        env.step_vel(wl, wr)
        if step % every == 0:
            p = env.pos()
            print(f"t={step/cfg.control_hz:5.1f}s x={p[0]:6.2f} y={p[1]:6.2f} "
                  f"yaw={env.yaw():6.2f} steer={steer:5.2f} th={th:.2f} "
                  f"dmin={rays.min():4.2f} dcen={rays[12]:4.2f}")
    p = env.pos()
    print(f"最终 x={p[0]:.2f} y={p[1]:.2f} (goal_x={cfg.goal_x})")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "car_scene_basic.xml")
