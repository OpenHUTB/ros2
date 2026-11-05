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


# ---------------- 5. 空 / None 观测不崩溃 ----------------
def test_none_observations():
    from fusion_perception import (lidar_points_to_observation,
                                   mask_to_bearing,
                                   radar_points_to_observation)
    assert mask_to_bearing(None, 90.0) == (None, 0)
    assert lidar_points_to_observation(None) is None
    assert lidar_points_to_observation(np.zeros((0, 4))) is None
    assert radar_points_to_observation(None) is None
    assert radar_points_to_observation(np.zeros((0, 4))) is None
    print("[OK] 空/None 观测不崩溃（相机/LiDAR/Radar 全部返回 None）")


# ---------------- 6. SyncBuffer.get 解包与 None 过滤 ----------------
def test_sync_get_obs_and_none_filter():
    b = SyncBuffer(tol=0.05)
    b.push("radar", 0.1, (3.0, 0.2))
    got = b.get("radar", 0.1)
    assert got is not None
    ts, (r, th) = got                    # 正确解包 (ts, (r, θ))
    assert abs(r - 3.0) < 1e-9 and abs(th - 0.2) < 1e-9
    b2 = SyncBuffer(tol=0.05)            # 只 push 有效观测时，空缓冲返回 None
    assert b2.get("radar", 0.0) is None
    print("[OK] SyncBuffer.get 解包 (ts,(r,θ))，空缓冲返回 None")


# ---------------- 7. 相机观测经外参转换进 EKF ----------------
def test_camera_extrinsic_into_ekf():
    ekf = EKF2D(5.0, 3.0)
    th_rad = transform_camera_bearing(0.2, 5.0, (0.0, 0.0, 0.0))
    ekf.update_camera(th_rad)
    assert np.all(np.isfinite(ekf.state()))
    th_rad2 = transform_camera_bearing(0.2, 5.0, (10.0, 0.5, -0.3))
    ekf.update_camera(th_rad2)
    assert np.all(np.isfinite(ekf.state()))
    print("[OK] 相机观测经外参转换后进 EKF（单位/非零外参均正常）")


# ---------------- 8. Fake CARLA 融合周期（捕获 None 解包 TypeError） ----------------
def test_fake_fusion_cycle():
    """模拟主循环 3 个融合周期：有效观测入队、None 不入队，再 get→解包→EKF。
    评审指出旧代码把 None push 进缓冲后解包会 TypeError，本测试覆盖该路径。"""
    b = SyncBuffer(tol=0.15)
    ekf = EKF2D(6.0, 1.0)
    for k in range(3):
        t = k * 0.05
        rad_obs = None if k == 0 else (6.0 + 0.1 * k, 0.15)
        lid_obs = (6.0 + 0.1 * k, 0.14)
        cam_th = 0.16
        if rad_obs is not None:
            b.push("radar", t, rad_obs)
        if lid_obs is not None:
            b.push("lidar", t, lid_obs)
        if cam_th is not None:
            b.push("camera", t, cam_th)
        ekf.predict(0.05)
        rad = b.get("radar", t)
        lid = b.get("lidar", t)
        cam = b.get("camera", t)
        if rad is not None and rad[1] is not None:
            _, (r_ref, th_body) = rad
            ekf.update_radar(r_ref, th_body)
        if lid is not None and lid[1] is not None:
            _, (r_lidar, th_lidar) = lid
            ekf.update_radar(r_lidar, th_lidar)
        if cam is not None and cam[1] is not None:
            _, cam_bearing = cam
            th_rad = transform_camera_bearing(cam_bearing, 6.0,
                                              (0.0, 0.0, 0.0))
            ekf.update_camera(th_rad)
    assert np.all(np.isfinite(ekf.state()))
    print("[OK] Fake CARLA 3 个融合周期：None/有效观测混合解包无 TypeError")


# ---------------- 9. 多车场景：ROI 关联锁定目标 ----------------
def test_multi_target_association():
    from fusion_perception import (bearing_to_pixel, image_to_bearing,
                                   mask_to_bearing)
    mask = np.zeros((200, 400), dtype=np.uint8)
    mask[80:120, 100:140] = 10            # 目标（列≈120）
    mask[80:120, 280:320] = 10            # 干扰车（列≈300）
    th_pred, _ = mask_to_bearing(
        mask, 90.0, pred_bearing=image_to_bearing(120, 400, 90.0))
    assert th_pred is not None
    u_t = bearing_to_pixel(th_pred, 400, 90.0)
    assert abs(u_t - 120) < 10, f"有 pred 应锁定目标列 120，实际 {u_t:.0f}"
    print(f"[OK] 多车场景 ROI 关联：有 pred 锁定目标列≈{u_t:.0f}（非两车质心）")


# ---------------- 10. LiDAR/Radar 杂波下按 pred 选目标 ----------------
def test_lidar_radar_association():
    from fusion_perception import (lidar_points_to_observation,
                                   radar_points_to_observation)
    pts = np.array([[3.0, 0.0, 0.0, 0.0],      # 近处杂波簇（3 点，间距>voxel）
                    [3.5, 0.0, 0.0, 0.0],
                    [3.0, 0.5, 0.0, 0.0],
                    [12.0, 0.0, 0.0, 0.0],     # 目标簇
                    [12.5, 0.0, 0.0, 0.0],
                    [12.0, 0.5, 0.0, 0.0]])
    near = lidar_points_to_observation(pts, min_range=1.5, pred=None)
    tgt = lidar_points_to_observation(pts, min_range=1.5, pred=(12.0,))
    assert near is not None and abs(near[0] - 3.0) < 1.0, "无 pred 应退回最近点"
    assert tgt is not None and abs(tgt[0] - 12.0) < 1.5, "有 pred 应锁定目标"
    rpts = np.array([[2.0, 0.0, 0.0, 0.0],     # 近处杂波
                     [10.0, 0.05, 0.0, 0.0],   # 目标
                     [10.1, 0.06, 0.0, 0.0]])
    rob = radar_points_to_observation(rpts, pred=(10.0, 0.05))
    assert rob is not None and abs(rob[0] - 10.0) < 1.0
    print("[OK] LiDAR/Radar 杂波下按 pred 锁定目标（非最近反射点）")


def run():
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("全部最小测试通过 ✓")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
