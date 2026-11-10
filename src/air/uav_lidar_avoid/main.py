#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenHUTB 无人机激光雷达避障（作业二原始方案，已修正接口错误）。

对应文档：docs/air/uav_lidar_avoid.md

本文件是作业二的修正版本，保留"全向最小距离 + 固定阈值"的原始策略，
只修正会直接抛异常的三处接口错误，便于与作业三（分扇区+梯形规划）做对照：

  1. drone.set_velocity()      -> set_target_velocity()
  2. p.distance                -> 从 raw_data 解包 float32(x,y,z,intensity)
  3. 固定阈值 4m 会被机身自反射永久触发
     -> 把雷达挂到 z=2.0m（实测可消除自反射），并保留最小有效距离兜底

用法：
    python3 main.py
"""
import math
import struct
import os
import sys

try:
    import carla
except ImportError:  # pragma: no cover
    root = os.environ.get("OPENHUTB_ROOT", r"C:\hutb_windows_v2.10.0")
    dist = os.path.join(root, "PythonAPI", "carla", "dist")
    if os.path.isdir(dist):
        sys.path.insert(0, dist)
    import carla

# ---------- 参数 ----------
SAFE_DIST = 4.0      # 停止阈值 m（原始方案取值）
FORWARD_V = 2.0      # 前进速度 m/s
LIDAR_RANGE = "50.0"
LIDAR_MOUNT_Z = 2.0  # 实测：0.5m 时被自反射占据，2.0m 后消失
MIN_VALID = 3.0      # 最小有效距离兜底 m


def parse_points(msg):
    """从 LidarMeasurement.raw_data 解包点云。"""
    raw = msg.raw_data
    n = len(raw) // 16
    vals = struct.unpack("<" + "ffff" * n, raw)
    return [(vals[i], vals[i + 1], vals[i + 2]) for i in range(0, n * 4, 4)]


def min_distance(points):
    """全向最小有效水平距离（原始方案策略）。"""
    best = float("inf")
    for x, y, z in points:
        if abs(z) > 3.0:
            continue
        d = math.hypot(x, y)
        if d < MIN_VALID:
            continue
        best = min(best, d)
    return best


def main():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    print("[OK] 已连接 OpenHUTB:", world.get_map().name)

    drone = None
    for actor in world.get_actors():
        if "airsim.drone" in actor.type_id:
            drone = actor
            break
    if drone is None:
        print("[FAIL] 场景内没有无人机 Actor")
        return
    print(f"[OK] 无人机 id={drone.id}")

    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    bp.set_attribute("range", LIDAR_RANGE)
    bp.set_attribute("channels", "32")
    bp.set_attribute("points_per_second", "200000")
    # 默认 dropoff_general_rate=0.45 会丢掉 45% 的点
    bp.set_attribute("dropoff_general_rate", "0.0")

    lidar = world.spawn_actor(
        bp, carla.Transform(carla.Location(z=LIDAR_MOUNT_Z)), attach_to=drone
    )
    print(f"[OK] 雷达已挂载 id={lidar.id} 高度={LIDAR_MOUNT_Z}m")

    state = {"d": float("inf")}

    def on_scan(msg):
        state["d"] = min_distance(parse_points(msg))

    lidar.listen(on_scan)
    import time
    time.sleep(1.0)

    try:
        while True:
            d = state["d"]
            if d < SAFE_DIST:
                print(f"[停止] 前方障碍 {d:.2f}m")
                # 注意：不是 set_velocity
                drone.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            else:
                print(f"[前进] 前方安全 {d:.2f}m")
                fwd = drone.get_transform().get_forward_vector()
                drone.set_target_velocity(
                    carla.Vector3D(fwd.x * FORWARD_V, fwd.y * FORWARD_V, 0.0)
                )
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[STOP] 用户中断")
    finally:
        drone.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        lidar.stop()
        lidar.destroy()


if __name__ == "__main__":
    main()
