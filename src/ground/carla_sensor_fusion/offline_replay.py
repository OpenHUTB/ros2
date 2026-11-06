# -*- coding: utf-8 -*-
"""offline_replay.py —— 无 CARLA 环境下的端到端离线回放

用合成的 相机 / LiDAR / Radar / IMU 数据流，驱动与 carla_sensor_fusion **完全相同**的
感知（fusion_perception）→ 时间同步（SyncBuffer，按传感器真实时间戳）→ 外参标定
（estimate_extrinsic_2d）→ EKF 融合 链路，输出与 CARLA 版同格式的日志与 CSV。

用途：在无 GPU / CARLA / ROS 的沙箱或 CI 中验证整条链路不崩溃、可产出指标；
**不代表真实 CARLA 物理传感器结果**（真实结果需本机启动 CARLA 跑 `main.py carla`）。
"""
import argparse
import csv
import math

import numpy as np

from sensor_fusion_core import (EKF2D, SyncBuffer, estimate_extrinsic_2d,
                                wrap_angle)
from fusion_perception import (CARLA_TAG_VEHICLE, bearing_to_pixel,
                               imu_compass_to_yaw, lidar_points_to_observation,
                               mask_to_bearing, points_to_extrinsic_pairs,
                               radar_points_to_observation,
                               transform_camera_bearing)


def figure_eight(t, base_x=10.0, amp_x=2.0, amp_y=1.0, omega=0.5):
    """目标在 ego 前方约 10 m 处做小幅 8 字（保持相机/雷达视场内可跟踪）。"""
    return np.array([base_x + amp_x * math.sin(omega * t),
                     amp_y * math.sin(2.0 * omega * t)])


def ego_yaw(t):
    return math.radians(6.0 * math.sin(0.3 * t))


def body_polar(truth, yaw):
    c, s = math.cos(-yaw), math.sin(-yaw)
    bx = c * truth[0] - s * truth[1]
    by = s * truth[0] + c * truth[1]
    return bx, by, float(math.hypot(bx, by)), float(math.atan2(by, bx))


def make_mask(width, height, th_cam, fov_deg, rng, distractor=False):
    """合成语义分割图：目标车辆 + 一辆不同方位的干扰车（评审曾指出多车干扰）。"""
    mask = np.zeros((height, width), dtype=np.uint8)
    u = int(round(bearing_to_pixel(th_cam, width, fov_deg) + rng.normal(0, 1.0)))
    if 0 <= u < width:
        mask[height // 4:3 * height // 4, max(0, u - 15):min(width, u + 15)] = CARLA_TAG_VEHICLE
    if distractor:
        u2 = int(round(bearing_to_pixel(-0.6 * th_cam, width, fov_deg)))
        if 0 <= u2 < width:
            mask[height // 4:3 * height // 4,
                 max(0, u2 - 12):min(width, u2 + 12)] = CARLA_TAG_VEHICLE
    return mask


def make_lidar(r, th_lidar, rng):
    """合成 LiDAR 点云：目标簇 + 近处道路杂波簇（同距不同方位）。"""
    pts = []
    for _ in range(4):
        rr = r + rng.normal(0, 0.05)
        tt = th_lidar + rng.normal(0, 0.01)
        pts.append([rr * math.cos(tt), rr * math.sin(tt), rng.uniform(-0.2, 0.8), 0.0])
    for _ in range(4):
        rr = 3.0 + rng.normal(0, 0.05)
        tt = -th_lidar + rng.normal(0, 0.01)
        pts.append([rr * math.cos(tt), rr * math.sin(tt), 0.0, 0.0])
    return np.array(pts)


def make_radar(r, th_radar, rng):
    """合成毫米波雷达检测点：目标 + 近处道路反射。"""
    pts = []
    for _ in range(3):
        pts.append([r + rng.normal(0, 0.08), th_radar + rng.normal(0, 0.01), 0.0, 0.0])
    for _ in range(2):
        pts.append([2.5 + rng.normal(0, 0.05), -th_radar + rng.normal(0, 0.02), 0.0, 0.0])
    return np.array(pts)


def run(args):
    dt = args.dt
    ticks = int(round(args.seconds / dt))
    rng = np.random.default_rng(args.seed)
    cam_every = max(1, int(round((1.0 / 30.0) / dt)))
    lid_every = max(1, int(round(0.1 / dt)))
    rad_every = max(1, int(round(0.05 / dt)))
    fusion_every = max(1, int(round(args.fusion_period / dt)))

    counts = {"camera": 0, "lidar": 0, "radar": 0, "imu": 0}
    buffer = SyncBuffer(tol=0.15)
    last = {"cam": (None, -1e9), "lid": (None, -1e9),
            "rad": (None, -1e9), "yaw": (None, -1e9)}
    pairs_cam, pairs_lid = [], []
    ext = None
    ekf = None
    n_fuse = 0
    err_sum = err_sq = 0.0
    metrics = []

    for k in range(ticks):
        t = k * dt
        truth = figure_eight(t)
        yaw = ego_yaw(t)
        _, _, br, bth = body_polar(truth, yaw)

        pred_r = pred_th = None
        if ekf is not None:
            dx, dy = ekf.state()[0], ekf.state()[1]
            c, s = math.cos(-yaw), math.sin(-yaw)
            px, py = c * dx - s * dy, s * dx + c * dy
            pred_r = float(math.hypot(px, py))
            pred_th = float(math.atan2(py, px))

        # ---- 相机 30 Hz
        if k % cam_every == 0:
            counts["camera"] += 1
            mask = make_mask(args.image_width, args.image_height, bth,
                             args.camera_fov, rng)
            th_cam, _ = mask_to_bearing(mask, args.camera_fov, pred_bearing=pred_th)
            last["cam"] = (th_cam, t)
            if th_cam is not None:
                buffer.push("camera", t, th_cam)

        # ---- LiDAR 10 Hz（方位与相机相反）
        if k % lid_every == 0:
            counts["lidar"] += 1
            pts = make_lidar(br, -bth, rng)
            obs = lidar_points_to_observation(
                pts, max_range=args.lidar_range,
                pred=(pred_r, pred_th) if pred_r is not None else None)
            last["lid"] = (obs, t)
            if obs is not None:
                buffer.push("lidar", t, obs)

        # ---- Radar 20 Hz（方位右为正，与相机一致）
        if k % rad_every == 0:
            counts["radar"] += 1
            rpts = make_radar(br, bth, rng)
            rob = radar_points_to_observation(
                rpts, max_range=args.radar_range,
                pred=(pred_r, pred_th) if pred_r is not None else None)
            last["rad"] = (rob, t)
            if rob is not None:
                buffer.push("radar", t, rob)

        # ---- IMU 100 Hz
        if k % max(1, int(round(0.01 / dt))) == 0:
            counts["imu"] += 1
            yaw_obs = imu_compass_to_yaw(math.pi / 2.0 - yaw)
            last["yaw"] = (yaw_obs, t)
            buffer.push("imu", t, yaw_obs)

        # ---- 在线外参标定（相机方位 + 雷达观测配对）
        if (ext is None and last["cam"][0] is not None and
                last["rad"][0] is not None and last["cam"][1] == t and
                last["rad"][1] == t):
            pairs_cam.append(last["cam"][0])
            pairs_lid.append(last["rad"][0])
        if ext is None and len(pairs_cam) >= args.calib_samples:
            pc, pl = points_to_extrinsic_pairs(pairs_cam, pairs_lid)
            ext = estimate_extrinsic_2d(pc, pl)
            print(f"[i] 在线外参标定完成（{len(pairs_cam)} 组）："
                  f"Δθ={ext[0]:+.2f}° t=({ext[1]:+.2f},{ext[2]:+.2f}) m "
                  f"res={ext[3]:.3f} m")

        # ---- 首个观测初始化 EKF
        init_obs = last["rad"][0] if last["rad"][0] is not None else last["lid"][0]
        if ekf is None and init_obs is not None and last["yaw"][0] is not None:
            r0, th0 = init_obs
            th_w = wrap_angle(th0 + last["yaw"][0])
            ekf = EKF2D(r0 * math.cos(th_w), r0 * math.sin(th_w),
                        sig_theta_radar=0.10, sig_theta_cam=0.04)
            print(f"[i] EKF 初始化：r={r0:.2f} m θ={math.degrees(th0):+.1f}°")

        # ---- 融合周期
        if ekf is not None and ext is not None and k % fusion_every == 0:
            ekf.predict(args.fusion_period)
            yaw_est = last["yaw"][0] if last["yaw"][0] is not None else 0.0
            rad = buffer.get("radar", t)
            lid = buffer.get("lidar", t)
            cam = buffer.get("camera", t)
            r_ref = None
            if rad is not None and rad[1] is not None:
                _, (r_ref, th_body) = rad
                ekf.update_radar(r_ref, wrap_angle(th_body + yaw_est))
            if lid is not None and lid[1] is not None:
                _, (r_lidar, th_lidar) = lid
                if r_ref is None:
                    r_ref = r_lidar
                ekf.update_radar(r_lidar, wrap_angle(th_lidar + yaw_est))
            if cam is not None and cam[1] is not None:
                _, cam_bearing = cam
                if r_ref is None:
                    r_ref = max(float(np.linalg.norm(ekf.state()[:2])), 0.5)
                th_rad = transform_camera_bearing(cam_bearing, r_ref, ext)
                ekf.update_camera(wrap_angle(th_rad + yaw_est))

            est = ekf.state()
            err = float(np.linalg.norm(est[:2] - truth))
            err_sum += err
            err_sq += err * err
            n_fuse += 1
            metrics.append({"t": round(t, 3), "err_m": round(err, 4),
                            "est_x": round(float(est[0]), 3),
                            "est_y": round(float(est[1]), 3),
                            "truth_x": round(float(truth[0]), 3),
                            "truth_y": round(float(truth[1]), 3)})

    rmse = math.sqrt(err_sq / n_fuse) if n_fuse else float("nan")
    print(f"[OK] 离线回放完成：{ticks} ticks（{args.seconds:.1f} s）")
    print(f"     传感器帧数 camera={counts['camera']} lidar={counts['lidar']} "
          f"radar={counts['radar']} imu={counts['imu']}")
    print(f"     融合次数={n_fuse}  估计 RMSE={rmse:.3f} m")
    if args.csv and metrics:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(metrics[0].keys()))
            w.writeheader()
            w.writerows(metrics)
        print(f"[OK] 指标已写入 {args.csv}（{len(metrics)} 行）")
    elif args.csv:
        print(f"[i] 本次无融合样本（n_fuse=0），跳过 CSV 写入")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="无 CARLA 的离线端到端回放")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--fusion-period", type=float, default=0.05)
    ap.add_argument("--camera-fov", type=float, default=90.0)
    ap.add_argument("--image-width", type=int, default=800)
    ap.add_argument("--image-height", type=int, default=450)
    ap.add_argument("--lidar-range", type=float, default=50.0)
    ap.add_argument("--radar-range", type=float, default=40.0)
    ap.add_argument("--calib-samples", type=int, default=12)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--csv", default="offline_replay_metrics.csv")
    args = ap.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
