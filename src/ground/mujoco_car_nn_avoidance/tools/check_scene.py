#!/usr/bin/env python3
"""物理冒烟测试：确认场景能加载、小车能直行不翻、射线感知正常。

运行（无需图形界面）：
    python tools/check_scene.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mujoco_car_nn_avoidance.config import Config
from mujoco_car_nn_avoidance.car_env import CarEnv
from mujoco_car_nn_avoidance.sensors import ray_scan


def main():
    cfg = Config()
    ok = True
    for scene in cfg.scenes:
        env = CarEnv(cfg, scene)
        env.reset()
        # 直行：左右轮同速（速度执行器，ctrl=目标轮角速度）
        w = 20.0
        n = int(2.0 / cfg.dt)          # 2 秒仿真 = control_hz 次控制
        min_z = 1e9
        max_speed = 0.0
        for _ in range(n):
            env.step_vel(w, w)
            pos = env.pos()
            min_z = min(min_z, pos[2])
            max_speed = max(max_speed, abs(env.forward_speed()))
        pos_end = env.pos()
        yaw_end = env.yaw()
        rays = ray_scan(env)
        print(f"[{scene}]")
        print(f"  直行2s后 pos=({pos_end[0]:.3f},{pos_end[1]:.3f},{pos_end[2]:.3f}) yaw={yaw_end:.3f} rad")
        print(f"  最小z={min_z:.4f} (车身是否贴地/翻车检查)  max前向速度={max_speed:.3f} m/s")
        print(f"  射线最近/最远 = {rays.min():.3f} / {rays.max():.3f} m, 第0/12/24束={rays[0]:.2f}/{rays[12]:.2f}/{rays[24]:.2f}")
        # 判定
        if min_z < 0.02 or min_z > 0.30:
            print("  !! 车身高度异常（可能翻车/飞起）")
            ok = False
        if pos_end[0] < 0.3:
            print("  !! 2秒没前进（驱动失效）")
            ok = False
        if rays.min() > 5.9:
            print("  !! 射线全无命中（感知失效）")
            ok = False
    print("\n冒烟测试：" + ("通过" if ok else "存在问题"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
