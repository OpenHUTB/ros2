#!/usr/bin/env python3
"""
sim_2d_relocalization.py —— 轻量 2D 粒子滤波重定位仿真（matplotlib 渲染）
==================================================================================
与 Gazebo/TurtleBot3 版共用同一套重定位核心（reloc_core.ParticleFilter，AMCL 风格）：
  - 机器人只知道环境地图、不知道自己初始在哪（未知位姿）；
  - 地图上均匀撒 N 个粒子，随里程计运动推进，用激光扫描与地图的匹配更新权重，
    低方差重采样后粒子收敛到真实位姿 = 重定位完成；
  - 同时维护一条『纯里程计航位推算』轨迹，展示无地图修正时误差累积漂移。

动图展示：粒子云从全图分散 → 逐步聚拢到机器人真实位置；AMCL 位置误差远小于
纯里程计漂移。

运行：
    python3 sim_2d_relocalization.py            # 生成 relocalization_demo.gif
    python3 sim_2d_relocalization.py --show     # 弹出窗口实时查看
依赖：numpy, matplotlib, Pillow
"""
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation, PillowWriter

from reloc_core import default_env, scan_sample, ParticleFilter


# ---------------- 真值路径（蛇形穿行内墙/孤岛区，扫描区分度高） ----------------
WAYPOINTS = np.array([
    [-4.5, -4.5], [4.5, -4.5], [2.5, -3.2], [2.5, 4.5], [-3.5, 4.5],
    [-3.5, 1.5], [3.0, 1.5], [3.0, -2.2], [-2.0, -2.2], [-2.0, 3.2],
    [4.5, 3.2], [4.5, 4.5],
])
SPEED = 0.55      # m/s
DT = 0.05         # 仿真步长


def true_pose_at(s):
    """沿 waypoints 按累计路程 s 取位姿 (x, y, θ)。"""
    seg_len = np.linalg.norm(np.diff(WAYPOINTS, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]
    s = s % total
    idx = int(np.searchsorted(cum, s)) - 1
    idx = max(idx, 0)
    t = (s - cum[idx]) / max(seg_len[idx], 1e-9)
    p = WAYPOINTS[idx] + t * (WAYPOINTS[idx + 1] - WAYPOINTS[idx])
    th = np.arctan2(WAYPOINTS[idx + 1][1] - WAYPOINTS[idx][1],
                    WAYPOINTS[idx + 1][0] - WAYPOINTS[idx][0])
    return np.array([p[0], p[1], th])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--npart", type=int, default=1000)
    ap.add_argument("--out", default="relocalization_demo.gif")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    m = default_env()
    rng = np.random.default_rng(11)
    angles = np.linspace(0, 2 * np.pi, 90, endpoint=False)
    max_range = 4.0

    pf = ParticleFilter(args.npart, rng)
    pf.init_uniform()

    # 里程计（航位推算）：真值位移 scale 1.02 + 航向偏置，起点与真值一致
    odom_pose = true_pose_at(0.0).copy()
    prev_th = odom_pose[2]
    odom_xy = []
    true_xy = []
    est_hist = []
    s = 0.0
    err_amcl = err_odom = 0.0
    # 末帧验收指标（供保存后以 key=value 打到 stdout，便于 CI / 复现者核对）
    metrics = {"state": "searching", "err_amcl": 0.0, "err_odom": 0.0,
               "spread": 0.0, "neff_pct": 0.0}

    # ---------------- 绘图 ----------------
    fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=100)
    ax.imshow(m.occ.T, origin="lower", extent=[-6, 6, -6, 6],
              cmap="Greys", alpha=0.55, interpolation="nearest")
    ax.plot([-6, 6, 6, -6, -6], [-6, -6, 6, 6, -6], "k-", lw=0.8, alpha=0.5)
    ax.set_xlim(-6.2, 6.2)
    ax.set_ylim(-6.2, 6.2)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Particle-filter localization (AMCL-style relocalization)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)

    true_line, = ax.plot([], [], "-", color="black", lw=1.6, label="true path")
    odom_line, = ax.plot([], [], "-", color="orange", lw=1.4, alpha=0.9,
                         label="odometry dead-reckoning (drifts)")
    est_line, = ax.plot([], [], "-", color="red", lw=1.8, label="AMCL estimate")
    pcloud, = ax.plot([], [], ".", color="#1f77b4", ms=3, alpha=0.35,
                      label="particles")
    est_marker, = ax.plot([], [], "x", color="red", ms=12, mew=2)
    true_marker, = ax.plot([], [], "o", color="black", ms=7)
    info = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top",
                   fontsize=9.5, fontfamily="monospace",
                   bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9))
    ell_patch = [None]

    def update(frame):
        nonlocal s, odom_pose, prev_th, err_amcl, err_odom, metrics
        # 一帧 = 若干仿真步
        for _ in range(1):
            s += SPEED * DT
            truth = true_pose_at(s)
            prev = true_xy[-1] if true_xy else truth[:2]
            # 真值位移与朝向增量
            move = truth[:2] - prev
            dtheta = truth[2] - prev_th
            dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
            prev_th = truth[2]

            # 里程计（航位推算，含尺度误差与航向漂移）
            d_move = float(np.linalg.norm(move))
            odom_pose[2] += dtheta + 0.010                # 航向每步偏置漂移
            odom_pose[2] = (odom_pose[2] + np.pi) % (2 * np.pi) - np.pi
            # 沿自身漂移后的航向走 d*scale（无地图修正，误差累积）
            odom_pose[:2] += (d_move * 1.04
                              * np.array([np.cos(odom_pose[2]),
                                          np.sin(odom_pose[2])]))

            # 粒子滤波：运动预测 + 观测更新 + 重采样
            pf.predict(move, dtheta)
            scan = scan_sample(m, truth, angles, max_range, rng, noise=0.02)
            pf.update(m, scan, angles, max_range=max_range)
            pf.resample()

            true_xy.append(truth[:2])
            odom_xy.append(odom_pose[:2].copy())
            est = pf.estimate()
            est_hist.append(est[:2])
            err_amcl = np.linalg.norm(est[:2] - truth[:2])
            err_odom = np.linalg.norm(odom_pose[:2] - truth[:2])

        tx = np.asarray(true_xy)
        ox = np.asarray(odom_xy)
        ex = np.asarray(est_hist)
        true_line.set_data(tx[:, 0], tx[:, 1])
        odom_line.set_data(ox[:, 0], ox[:, 1])
        est_line.set_data(ex[:, 0], ex[:, 1])
        pcloud.set_data(pf.particles[:, 0], pf.particles[:, 1])
        est_marker.set_data([est[0]], [est[1]])
        true_marker.set_data([truth[0]], [truth[1]])

        # 粒子云散布（收敛度）与位置不确定度
        spread = float(np.sqrt(np.var(pf.particles[:, 0])
                               + np.var(pf.particles[:, 1])))
        if ell_patch[0] is not None:
            ell_patch[0].remove()
        ell_patch[0] = Ellipse((est[0], est[1]), 2 * np.sqrt(np.var(pf.particles[:, 0]) + 0.05),
                               2 * np.sqrt(np.var(pf.particles[:, 1]) + 0.05),
                               fc="red", alpha=0.12, ec="none")
        ax.add_patch(ell_patch[0])

        state = "relocalized" if spread < 0.35 else "searching"
        info.set_text(
            f"t = {s / SPEED:.1f} s   state = {state}\n"
            f"err  AMCL = {err_amcl:.2f} m   odometry = {err_odom:.2f} m\n"
            f"pos  AMCL = ({est[0]:.1f},{est[1]:.1f})   truth = "
            f"({truth[0]:.1f},{truth[1]:.1f})\n"
            f"particle spread = {spread:.2f} m   Neff = "
            f"{pf.neff() / args.npart * 100:.0f}%")
        metrics.update(state=state, err_amcl=err_amcl, err_odom=err_odom,
                       spread=spread, neff_pct=pf.neff() / args.npart * 100)
        return (true_line, odom_line, est_line, pcloud, est_marker,
                true_marker, info)

    anim = FuncAnimation(fig, update, frames=args.frames, interval=1000 / args.fps,
                         blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps), dpi=100)
    # stdout 精简：第一行为 GIF 绝对路径（可直接 `$(...) | head -1` 取用），
    # 随后为末帧 key=value 验收指标（state / AMCL·里程计误差 / 粒子 spread / Neff）
    print(os.path.abspath(args.out))
    print(f"state={metrics['state']}")
    print(f"err_amcl={metrics['err_amcl']:.2f} m")
    print(f"err_odom={metrics['err_odom']:.2f} m")
    print(f"particle_spread={metrics['spread']:.2f} m")
    print(f"neff={metrics['neff_pct']:.0f}%")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
