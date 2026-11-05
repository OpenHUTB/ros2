#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_trajectory.py -- 由 record_trajectory.py 的 CSV 生成作业交付图表

产出（默认写入 docs/assets/）
------------------------------
    trajectory_3d.png      三维轨迹效果图（规划轨迹 + 实飞轨迹 + 航点）
    speed_profile.png      速度曲线（随时间）
    trajectory_topdown.png 俯视图（x-y 平面，便于观察避障绕行）

用法
----
    python3 plot_trajectory.py --csv ~/allocnet_trajectory.csv \\
                               --out-dir docs/assets
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")                      # 无显示环境（VM/headless）必须
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D     # noqa: F401  注册 3d 投影


SOURCE_STYLE = {
    "planned": dict(color="#1f77b4", lw=2.2, ls="-",
                    label="AllocNet planned trajectory"),
    "flight":  dict(color="#d62728", lw=1.6, ls="-",
                    label="Executed flight"),
    "teleop":  dict(color="#2ca02c", lw=1.4, ls="--",
                    label="Keyboard cursor path"),
}


def read_csv(path):
    """读回 CSV，返回按 source 分组的 {source: {field: [values]}}。"""
    if not os.path.isfile(path):
        sys.exit("找不到 CSV: %s" % path)

    data = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            src = row["source"]
            bucket = data.setdefault(
                src, {"time": [], "x": [], "y": [], "z": [], "v": []})
            try:
                for k in ("time", "x", "y", "z", "v"):
                    bucket[k].append(float(row[k]))
            except (ValueError, KeyError):
                continue

    if not data:
        sys.exit("CSV 为空，请先运行仿真并记录数据: %s" % path)
    return data


def pick(data, *names):
    """按优先级返回第一个存在的 source，便于 flight/planned 回退。"""
    for n in names:
        if n in data and len(data[n]["x"]) > 1:
            return data[n]
    return None


def split_runs(times, gap=1.0):
    """按时间间隔切分连续段，避免多段轨迹被连成一条斜线。

    注意：`planned` 点位来自一次性发布的整条轨迹 Marker，它们的 time
    全都相同（同一次发布）。这种情况不能按时间切分，否则会被切成
    一个个孤立点。此时直接整段返回。
    """
    if len(times) > 1 and times[-1] - times[0] < 1e-6:
        return [list(range(len(times)))]
    runs, cur = [], [0]
    for i in range(1, len(times)):
        if times[i] - times[i - 1] > gap:
            runs.append(cur)
            cur = []
        cur.append(i)
    if cur:
        runs.append(cur)
    return runs


def load_obstacles(path):
    """读取离线仿真导出的障碍物清单 (可选)。

    文件格式: cx, cy, radius 每行一个。没有该文件时返回空列表，
    图上就不画障碍物（真实 ROS 运行时不依赖它，因为地图来自点云）。
    """
    if not path or not os.path.isfile(path):
        return []
    obs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 3:
                try:
                    obs.append(tuple(float(v) for v in parts[:3]))
                except ValueError:
                    continue
    return obs


def load_cloud(path, z_slice=None):
    """读取地图点云 CSV (x,y,z)。z_slice=(lo,hi) 时只保留该高度带。

    真实 ROS 运行导出的障碍是点云（dump_cloud.py），不是规则圆柱，
    因此优先用点云作图，障碍圆只在离线演示时才会用到。
    """
    if not path or not os.path.isfile(path):
        return None
    xs, ys = [], []
    with open(path) as f:
        first = True
        for line in f:
            line = line.strip()
            if not line:
                continue
            if first:
                first = False
                if line.startswith("x"):
                    continue
            p = line.split(",")
            if len(p) < 3:
                continue
            try:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
            except ValueError:
                continue
            if z_slice and not (z_slice[0] <= z <= z_slice[1]):
                continue
            xs.append(x)
            ys.append(y)
    return (xs, ys) if xs else None


def draw_cloud_2d(ax, cloud, label=True):
    """俯视图里用散点画障碍点云。"""
    if not cloud:
        return
    xs, ys = cloud
    ax.scatter(xs, ys, s=1.0, c="#555555", alpha=0.35, marker="s",
               zorder=1, label="Obstacles (map point cloud)" if label else None)


def draw_obstacles_2d(ax, obstacles):
    """俯视图里的障碍圆（离线演示用）。"""
    for i, (cx, cy, r) in enumerate(obstacles):
        ax.add_patch(plt.Circle((cx, cy), r, color="#444444",
                                alpha=0.35, zorder=1,
                                label="Obstacle (cylinder)" if i == 0 else None))
        ax.add_patch(plt.Circle((cx, cy), r, fill=False,
                                color="#222222", lw=1.0, zorder=2))


def clear_topdown_axes(ax, obstacles, data, cloud=None, track_only=False):
    """按内容自适应俯视图范围。

    track_only=True 时只用轨迹本身的包络（外加少量留白），
    避免整张地图的点云把轨迹压成一小团。
    """
    xs = [0.0]
    ys = [0.0]
    for d in data.values():
        xs.extend(d["x"])
        ys.extend(d["y"])
    for (cx, cy, r) in obstacles:
        xs.extend([cx - r, cx + r])
        ys.extend([cy - r, cy + r])

    if track_only and (len(xs) > 1):
        pad = 2.5
    else:
        if cloud:
            xs.extend(cloud[0])
            ys.extend(cloud[1])
        pad = 1.5

    if not xs:
        return
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)


# ----------------------------------------------------------------------
def draw_obstacles_3d(ax, obstacles, z0=0.0, z1=3.0):
    """在 3D 图里把圆柱障碍画成半透明柱面线框。"""
    import numpy as np
    theta = np.linspace(0, 2 * np.pi, 24)
    zz = np.array([z0, z1])
    tt, zgrid = np.meshgrid(theta, zz)
    for (cx, cy, r) in obstacles:
        xx = cx + r * np.cos(tt)
        yy = cy + r * np.sin(tt)
        ax.plot_surface(xx, yy, zgrid, color="#888888", alpha=0.22,
                        linewidth=0, shade=False)


def plot_3d(data, out_path, obstacles=None, cloud=None):
    obstacles = obstacles or []
    fig = plt.figure(figsize=(11, 8), dpi=150)
    ax = fig.add_subplot(111, projection="3d")

    if obstacles:
        draw_obstacles_3d(ax, obstacles)
    # 注意：3D 视图里不画点云。地图有数十万点，全画出来是一团黑，
    # 反而看不清轨迹；障碍物仅在俯视图（plot_topdown）中呈现。

    for src, style in SOURCE_STYLE.items():
        d = data.get(src)
        if not d or len(d["x"]) < 2:
            continue
        for run in split_runs(d["time"]):
            ax.plot([d["x"][i] for i in run],
                    [d["y"][i] for i in run],
                    [d["z"][i] for i in run], **style)

    # 起终点标注
    for src, color, tag in (("start", "#8c564b", "START"),
                            ("goal", "#e377c2", "GOAL")):
        d = data.get(src)
        if not d or not d["x"]:
            continue
        ax.scatter(d["x"][0], d["y"][0], d["z"][0],
                   c=color, s=90, marker="o", edgecolors="k",
                   label="%s (%.1f, %.1f, %.1f)"
                         % (tag, d["x"][0], d["y"][0], d["z"][0]),
                   depthshade=False)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("AllocNet + Keyboard Teleoperation — 3D Trajectory")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    try:
        ax.set_box_aspect((1, 1, 0.45))
    except AttributeError:
        pass
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print("[plot] 写出 %s" % out_path)


def plot_topdown(data, out_path, obstacles=None, cloud=None):
    obstacles = obstacles or []
    fig, ax = plt.subplots(figsize=(8, 8), dpi=150)

    if obstacles:
        draw_obstacles_2d(ax, obstacles)
    elif cloud:
        draw_cloud_2d(ax, cloud)

    for src, style in SOURCE_STYLE.items():
        d = data.get(src)
        if not d or len(d["x"]) < 2:
            continue
        first = True
        for run in split_runs(d["time"]):
            # 注意：必须显式传 ls，否则实线会覆盖 SOURCE_STYLE 里的虚线约定
            ax.plot([d["x"][i] for i in run],
                    [d["y"][i] for i in run],
                    color=style["color"], lw=style["lw"], ls=style["ls"],
                    label=style["label"] if first else None, zorder=3)
            first = False

    for src, color, tag in (("start", "#8c564b", "START"),
                            ("goal", "#e377c2", "GOAL")):
        d = data.get(src)
        if not d or not d["x"]:
            continue
        ax.scatter(d["x"][0], d["y"][0], c=color, s=110,
                   marker="o", edgecolors="k", zorder=5,
                   label="%s (%.1f, %.1f)" % (tag, d["x"][0], d["y"][0]))

    # 有点云时不要把整图范围撑开，聚焦在轨迹附近
    clear_topdown_axes(ax, obstacles, data, cloud=cloud,
                       track_only=bool(cloud and not obstacles))
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Top-down View (X–Y) — Obstacle Avoidance")
    ax.legend(loc="upper center", fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print("[plot] 写出 %s" % out_path)


def plot_speed(data, out_path):
    """速度曲线。优先用实飞段的速度，缺失时用规划轨迹的时间序列估算。"""
    flight = pick(data, "flight", "planned", "teleop")
    if not flight:
        print("[plot] 无足够数据绘制速度曲线，跳过")
        return

    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)

    plotted = False
    for src in ("flight", "planned", "teleop"):
        d = data.get(src)
        if not d or len(d["time"]) < 2:
            continue
        # 只保留速度非零的样本，避免把静止游标也画成一条贴地线
        ts = [t for t, v in zip(d["time"], d["v"]) if v > 1e-6]
        vs = [v for v in d["v"] if v > 1e-6]
        if len(ts) < 2:
            continue
        style = SOURCE_STYLE.get(src, {})
        ax.plot(ts, vs, color=style.get("color", "C0"),
                lw=1.8, ls=style.get("ls", "-"),
                label=style.get("label", src))
        plotted = True

    if not plotted:
        print("[plot] 速度样本全为 0（可能 /visualizer/speed 未发布），跳过")
        plt.close(fig)
        return

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Speed Profile — AllocNet Trajectory Execution")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print("[plot] 写出 %s" % out_path)


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True, help="record_trajectory.py 输出的 CSV")
    ap.add_argument("--out-dir", default="docs/assets", help="图片输出目录")
    ap.add_argument("--obstacles", default=None,
                    help="障碍物清单 (cx,cy,r 每行一个)，仅离线演示时需要；"
                         "ROS 运行时地图来自点云，可不传")
    ap.add_argument("--cloud", default=None,
                    help="真实地图点云 CSV (x,y,z)，由 dump_cloud.py 从 "
                         "/structure_map/global_gridmap 导出；用于在俯视图上"
                         "叠加真实障碍")
    ap.add_argument("--cloud-z", default=None,
                    help="点云高度切片 lo,hi（米），如 1.2,1.8；只画无人机"
                         "飞行高度附近的障碍，图更清晰")
    args = ap.parse_args()

    data = read_csv(args.csv)
    print("[plot] 读到 source: %s" %
          ", ".join("%s(%d)" % (k, len(v["x"])) for k, v in data.items()))

    obstacles = load_obstacles(args.obstacles)
    if obstacles:
        print("[plot] 载入 %d 个障碍物" % len(obstacles))

    cloud = None
    if args.cloud:
        zs = None
        if args.cloud_z:
            lo, hi = [float(v) for v in args.cloud_z.split(",")]
            zs = (lo, hi)
        cloud = load_cloud(args.cloud, zs)
        if cloud:
            print("[plot] 载入点云 %d 点%s"
                  % (len(cloud[0]), "（切片 %s）" % (zs,) if zs else ""))
        else:
            print("[plot] 警告：点云为空或读取失败")

    os.makedirs(args.out_dir, exist_ok=True)
    plot_3d(data, os.path.join(args.out_dir, "trajectory_3d.png"), obstacles,
            cloud=cloud)
    plot_topdown(data, os.path.join(args.out_dir, "trajectory_topdown.png"),
                 obstacles, cloud=cloud)
    plot_speed(data, os.path.join(args.out_dir, "speed_profile.png"))


if __name__ == "__main__":
    main()
