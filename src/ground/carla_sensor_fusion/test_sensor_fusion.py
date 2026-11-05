#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_sensor_fusion.py —— 最小离线测试（评审点：外参标定 / 时间同步 /
EKF 更新 / CARLA 传感数据回调）

无需 CARLA 与 ROS，纯 numpy / 标准库即可运行：
    python3 test_sensor_fusion.py
    python3 main.py test
"""
import math

import numpy as np

from sensor_fusion_core import (EKF2D, SyncBuffer, estimate_extrinsic_2d,
                                rot2, wrap_angle)
from fusion_perception import (lidar_points_to_observation,
                               radar_points_to_observation,
                               transform_camera_bearing)
from carla_sensor_fusion import SensorHub


# ---------------- 1. 外参标定 ----------------
def test_extrinsic_calibration():
    rng = np.random.default_rng(0)
    th_true = 0.2
    t_true = np.array([1.2, -0.5])
    R = rot2(th_true)
    pts_cam = rng.normal(0, 1.0, (24, 2))
    pts_radar = (R @ pts_cam.T).T + t_true + rng.normal(0, 0.02, (24, 2))
    th_est, tx, ty, res = estimate_extrinsic_2d(pts_cam, pts_radar)
    err_ang = abs(wrap_angle(math.radians(th_est) - th_true))
    assert err_ang < 0.03, f"标定角度偏差过大: {math.degrees(err_ang):.2f}°"
    assert abs(tx - t_true[0]) < 0.1, f"tx 偏差 {abs(tx-t_true[0]):.3f}"
    assert abs(ty - t_true[1]) < 0.1, f"ty 偏差 {abs(ty-t_true[1]):.3f}"
    print("[OK] 外参标定：恢复 Δθ/t，残差 "
          f"{res:.3f} m（角度偏差 {math.degrees(err_ang):.3f}°）")


# ---------------- 2. 时间同步 ----------------
def test_sync_buffer():
    b = SyncBuffer(tol=0.05)
    b.push("radar", 0.00, np.array([10.0, 0.1]))
    b.push("radar", 0.10, np.array([11.0, 0.2]))
    got = b.get("radar", 0.105)          # 最近 = 0.10
    assert got is not None and got[0] == 0.10, "应取最近时间戳 0.10"
    assert b.get("radar", 0.30) is None, "超过容差应判失同步"
    assert b.age("camera", 0.5) == float("inf"), "无观测 age 应为 inf"
    print("[OK] 时间同步：最近时间戳对齐、超时判失同步")


# ---------------- 3. EKF 更新收敛 ----------------
def test_ekf_update():
    px, py, r, th = 5.0, 3.0, math.hypot(5, 3), math.atan2(3, 5)
    ekf = EKF2D(px + 2, py + 2)          # 初值偏离真值
    for _ in range(5):
        ekf.predict(0.05)
        ekf.update_radar(r, th)
        ekf.update_camera(th)
    err = float(np.linalg.norm(ekf.state()[:2] - np.array([px, py])))
    assert err < 2.0, f"EKF 未收敛: {err:.3f} m"
    print(f"[OK] EKF 更新：雷达+相机观测驱动收敛，末帧误差 {err:.3f} m")


# ---------------- 4. 传感器回调（真实数据入队缓冲） ----------------
def test_sensor_callback_queue():
    hub = SensorHub()

    class FakeSensor:
        def __init__(self):
            self.cb = None
        def listen(self, cb):
            self.cb = cb

    cam, lid, rad, imu = (FakeSensor(), FakeSensor(),
                          FakeSensor(), FakeSensor())
    hub.attach(cam, lid, rad, imu)

    class Img:                       # 语义分割帧 800x450
        frame, timestamp = 1, 0.1
        raw_data = np.zeros(800 * 450 * 4, dtype=np.uint8).tobytes()
    class Lid:                       # 点云 4 点
        frame, timestamp = 1, 0.1
        raw_data = np.zeros(4 * 4, dtype=np.float32).tobytes()
    class Rad:                       # 毫米波雷达 4 点
        frame, timestamp = 1, 0.1
        raw_data = np.zeros(4 * 4, dtype=np.float32).tobytes()
    class Imu:
        frame, timestamp = 1, 0.1
        class measurement:
            compass = 0.5

    cam.cb(Img()); lid.cb(Lid()); rad.cb(Rad()); imu.cb(Imu())

    assert hub.counts["camera"] == 1 and hub.camera_q.qsize() == 1, "相机回调未入队"
    assert hub.counts["lidar"] == 1 and hub.lidar_q.qsize() == 1, "LiDAR 回调未入队"
    assert hub.counts["radar"] == 1 and hub.radar_q.qsize() == 1, "Radar 回调未入队"
    assert hub.counts["imu"] == 1 and hub.imu_q.qsize() == 1, "IMU 回调未入队"
    latest = hub.latest(hub.radar_q)  # 取走并清空
    assert latest is not None and hub.radar_q.qsize() == 0
    print("[OK] CARLA 传感数据回调：camera/lidar/radar/imu 均入队缓冲且可读取")


def run():
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("全部最小测试通过 ✓")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
