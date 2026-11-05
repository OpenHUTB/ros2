#!/usr/bin/env python3
"""
sim_2d_fusion.py —— 轻量 2D 多传感器融合仿真（matplotlib 渲染，可跑出效果动图）
==================================================================================
与 CARLA 版共用同一套融合核心（sensor_fusion_core）：
  场景：载体(ego)在原点缓慢横摆，跟踪一个沿 8 字轨迹运动的移动目标。
  传感器：
    - 雷达   ~10Hz ：测距 r + 方位（体坐标）
    - 相机   ~30Hz ：测方位（体坐标，噪声更大）
    - IMU    ~100Hz：估计载体姿态角，把各传感器体坐标观测转到世界系
  融合：EKF(20Hz) 世界系 4 态，匀速预测 + 雷达/相机观测更新。
  动图同时显示「雷达-only 跟踪」与「雷达+相机融合跟踪」，
  直观对比融合对跟踪精度的提升（RMSE 更低）。

运行：
    python3 sim_2d_fusion.py                  # 生成 fusion_demo.gif
    python3 sim_2d_fusion.py --show           # 弹出窗口实时查看
依赖：numpy, matplotlib, Pillow
"""
import argparse
import csv
import math
import os

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation, PillowWriter

from sensor_fusion_core import (EKF2D, SyncBuffer, estimate_extrinsic_2d,
                                rot2, wrap_angle)


def target_truth(t):
    """8 字(Lissajous)轨迹：x=A sin(wt), y=B sin(2wt)。"""
    A, B, w = 18.0, 9.0, 0.4
    return np.array([A * np.sin(w * t), B * np.sin(2.0 * w * t)])


# 各传感器采样周期（s）
PERIOD = {"radar": 0.1, "camera": 1.0 / 30.0, "imu": 0.01}
DT_SIM = 0.005      # 仿真推进步长
DT_FUSION = 0.05    # 融合周期 (20Hz)


def advance_sensors(clock, truth_f, RNG, sig_r, sig_th_r, sig_th_c, sig_imu,
                    t, buffer):
    """按各传感器『下一采样时刻』在 t 附近产生观测写入同步缓冲。"""
    phi = 0.06 * np.sin(0.25 * t)                 # ego 姿态真值
    pos_w = truth_f(t)
    p_body = rot2(-phi) @ pos_w                   # 世界→体坐标
    r = np.linalg.norm(p_body)
    th_body = np.arctan2(p_body[1], p_body[0])

    while clock["radar"] <= t:
        buffer.push("radar", clock["radar"],
                    np.array([r + RNG.normal(0, sig_r),
                              wrap_angle(th_body + RNG.normal(0, sig_th_r))]))
        clock["radar"] += PERIOD["radar"]
    while clock["camera"] <= t:
        buffer.push("camera", clock["camera"],
                    wrap_angle(th_body + RNG.normal(0, sig_th_c)))
        clock["camera"] += PERIOD["camera"]
    while clock["imu"] <= t:
        buffer.push("imu", clock["imu"],
                    wrap_angle(phi + RNG.normal(0, sig_imu)))
        clock["imu"] += PERIOD["imu"]


def run_fusion_step(buffer, ekf_fused, ekf_radar, sig_imu, truth_f, t):
    """一个融合周期：预测 + IMU 姿态补偿 + 取雷达/相机最近观测更新。
    返回 (fused_state, radar_state, obs, ages)。"""
    imu = buffer.get("imu", t)
    phi_est = imu[1] if imu is not None else 0.06 * np.sin(0.25 * t)

    for ekf in (ekf_fused, ekf_radar):
        ekf.predict(DT_FUSION)

    obs_radar = []
    cam_th = None
    rad = buffer.get("radar", t)
    if rad is not None:
        r, th_b = rad[1][0], rad[1][1]
        th_w = wrap_angle(th_b + phi_est)         # IMU 姿态补偿→世界系
        obs_radar.append((r * np.cos(th_w), r * np.sin(th_w)))
        ekf_fused.update_radar(r, th_w)
        ekf_radar.update_radar(r, th_w)
    cam = buffer.get("camera", t)
    if cam is not None:
        th_w = wrap_angle(cam[1] + phi_est)
        cam_th = th_w
        ekf_fused.update_camera(th_w)             # 仅融合滤波器用相机

    ages = {k: buffer.age(k, t) for k in ("radar", "camera", "imu")}
    return ekf_fused.state(), ekf_radar.state(), (obs_radar, cam_th), ages


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=240)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out", default="fusion_demo.gif",
                    help="动图输出路径（本地运行默认 fusion_demo.gif；文档展示动图 "
                         "docs/img/ground/carla_sensor_fusion_demo.gif 已入库，如需更新可 --out 指定）")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--csv", default="fusion_metrics.csv")
    ap.add_argument("--warmup-frames", type=int, default=0,
                    help="统计 RMSE 时跳过前 N 帧（默认 0=全程统计；可设如 40 只评收敛稳定段）")
    args = ap.parse_args(argv)

    # 后端按需选择：--show 用可交互后端（TkAgg），否则用 Agg（无显示环境也能稳定出图）
    matplotlib.use("TkAgg" if args.show else "Agg")

    RNG = np.random.default_rng(7)
    sig_r, sig_th_r = 0.5, 0.10    # 雷达：测距准、方位较粗
    sig_th_c = 0.04                # 相机：方位较准（30Hz 高频补充）
    sig_imu = 0.012                # IMU 姿态估计噪声

    # ---------------- 外参标定演示（相机↔雷达） ----------------
    th_cal = 0.18 + RNG.normal(0, 0.02)
    t_cal = np.array([1.1, -0.3]) + RNG.normal(0, 0.05, 2)
    pts_cam = RNG.normal(0, 1.0, (14, 2))
    pts_radar = (rot2(th_cal) @ pts_cam.T).T + t_cal + RNG.normal(0, 0.06, (14, 2))
    th_est, tx_est, ty_est, res_cal = estimate_extrinsic_2d(pts_cam, pts_radar)

    # ---------------- 滤波器 ----------------
    p0 = target_truth(0.0)
    ekf_fused = EKF2D(p0[0] + RNG.normal(0, 1), p0[1] + RNG.normal(0, 1),
                      sig_theta_radar=sig_th_r, sig_theta_cam=sig_th_c)
    ekf_radar = EKF2D(p0[0] + RNG.normal(0, 1), p0[1] + RNG.normal(0, 1),
                      sig_theta_radar=sig_th_r, sig_theta_cam=sig_th_c)
    buffer = SyncBuffer(tol=0.06)
    clock = {"radar": 0.0, "camera": 0.0, "imu": 0.0}

    # ---------------- 预跑：推进观测并让滤波收敛 ----------------
    t = 0.0
    t_end_pre = 3.0
    while t < t_end_pre:
        t += DT_SIM
        advance_sensors(clock, target_truth, RNG, sig_r, sig_th_r,
                        sig_th_c, sig_imu, t, buffer)
        if abs(t - round(t / DT_FUSION) * DT_FUSION) < 1e-9:
            run_fusion_step(buffer, ekf_fused, ekf_radar, sig_imu,
                            target_truth, t)

    # ---------------- 绘图 ----------------
    fig, ax = plt.subplots(figsize=(12, 6.5), dpi=100)
    truth_traj = np.array([target_truth(t) for t in np.linspace(0, args.frames /
                           args.fps, 500)])
    ax.plot(truth_traj[:, 0], truth_traj[:, 1], "--", color="gray", lw=1.0,
            alpha=0.8, label="target truth trajectory")
    ax.plot(0, 0, "s", color="black", ms=10, label="ego (origin)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Multi-sensor fusion tracking (radar + camera + IMU)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    est_line, = ax.plot([], [], "-o", color="#1f77b4", lw=2, ms=4,
                        label="EKF fused (radar+camera)")
    radar_pts, = ax.plot([], [], "x", color="red", ms=6, label="radar obs")
    cam_ray, = ax.plot([], [], "-", color="green", lw=1.0, alpha=0.7,
                       label="camera bearing")
    info = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top",
                   fontsize=9.5, fontfamily="monospace",
                   bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9))

    est_hist = []
    error_patch = [None]
    # RMSE 累计（融合 vs 仅雷达），结束统一打印并写 CSV
    err_sq_f, err_sq_r, n_cnt, n_frames = [0.0], [0.0], [0], [0]

    def update(frame):
        nonlocal t
        # 推进一个融合周期内的观测生成
        t_end = t + DT_FUSION
        while t < t_end:
            t += DT_SIM
            advance_sensors(clock, target_truth, RNG, sig_r, sig_th_r,
                            sig_th_c, sig_imu, t, buffer)
        st_f, st_r, (obs_radar, cam_th), ages = run_fusion_step(
            buffer, ekf_fused, ekf_radar, sig_imu, target_truth, t)

        est_hist.append(st_f[:2])
        eh = np.asarray(est_hist)
        est_line.set_data(eh[:, 0], eh[:, 1])

        if obs_radar:
            radar_pts.set_data([o[0] for o in obs_radar],
                               [o[1] for o in obs_radar])
        else:
            radar_pts.set_data([], [])
        if cam_th is not None:
            lx, ly = 40 * np.cos(cam_th), 40 * np.sin(cam_th)
            cam_ray.set_data([0, lx], [0, ly])
        else:
            cam_ray.set_data([], [])

        if error_patch[0] is not None:
            error_patch[0].remove()
        P = ekf_fused.covariance()
        vals, vecs = np.linalg.eigh(P[:2, :2])
        ang = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
        error_patch[0] = Ellipse((st_f[0], st_f[1]),
                                 6 * np.sqrt(max(vals[0], 1e-6)),
                                 6 * np.sqrt(max(vals[1], 1e-6)),
                                 angle=ang, fc="blue", alpha=0.10, ec="none")
        ax.add_patch(error_patch[0])

        truth = target_truth(t)
        err_f = np.linalg.norm(st_f[:2] - truth)
        err_r = np.linalg.norm(st_r[:2] - truth)
        if n_frames[0] >= args.warmup_frames:      # 跳过滤波器收敛段
            err_sq_f[0] += err_f * err_f
            err_sq_r[0] += err_r * err_r
            n_cnt[0] += 1
        n_frames[0] += 1
        info.set_text(
            f"t = {t:.1f} s\n"
            f"err  fused = {err_f:.2f} m   radar-only = {err_r:.2f} m\n"
            f"pos  fused = ({st_f[0]:.1f},{st_f[1]:.1f})   truth = "
            f"({truth[0]:.1f},{truth[1]:.1f})\n"
            f"sync  radar {ages['radar']*1000:.0f}ms  camera "
            f"{ages['camera']*1000:.0f}ms  imu {ages['imu']*1000:.0f}ms\n"
            f"calib  Δθ = {th_est:.2f}° (真 {np.degrees(th_cal):.2f}°)  "
            f"t = ({tx_est:.2f},{ty_est:.2f}) m  res = {res_cal:.3f} m")
        return (est_line, radar_pts, cam_ray, info)

    anim = FuncAnimation(fig, update, frames=args.frames, interval=1000 / args.fps,
                         blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps), dpi=100)
    print(f"[OK] 动图已生成: {os.path.abspath(args.out)}  "
          f"({args.frames} frames @ {args.fps}fps, "
          f"{os.path.getsize(args.out)//1024} KB)")
    if args.show:
        plt.show()

    # ---- 结束汇总：把 RMSE 指标打印出来并写入 CSV（评审点：指标不应只在文档里） ----
    if n_cnt[0]:
        rmse_f = math.sqrt(err_sq_f[0] / n_cnt[0])
        rmse_r = math.sqrt(err_sq_r[0] / n_cnt[0])
        improve = (1.0 - rmse_f / rmse_r) * 100.0 if rmse_r > 0 else 0.0
        print(f"[OK] 2D 实测指标: 融合 RMSE={rmse_f:.3f} m  仅雷达 RMSE={rmse_r:.3f} m  "
              f"提升≈{improve:.1f}%（{n_cnt[0]} 帧）")
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value"])
            w.writerow(["rmse_fused_m", f"{rmse_f:.3f}"])
            w.writerow(["rmse_radar_only_m", f"{rmse_r:.3f}"])
            w.writerow(["improvement_pct", f"{improve:.1f}"])
            w.writerow(["samples", n_cnt[0]])
        print(f"[i] 指标已写入 {os.path.abspath(args.csv)}")


if __name__ == "__main__":
    main()
