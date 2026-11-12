# -*- coding: utf-8 -*-
"""巡线小车仿真（纯 NumPy + Matplotlib，不依赖 ROS / Gazebo / CARLA）。

黑色引导线构成一条带内凹弯的闭合赛道。小车车头一排虚拟光传感器读取
压线情况；控制器在路径上沿前进方向取预瞄目标点（pure pursuit 思想），
按朝向误差 PD 控制转向角速度沿中线巡线；支持脱线判定与无窗口评测。

用法：
    python main.py --demo --outdir demo_out
    python main.py --headless --duration 90 --seed 42 --outdir demo_out
"""
import argparse
import json
import math
import os
import random

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ---------- 赛道拐点（闭合环线，单位：米） ----------
WAYPOINTS = [
    (15.0, 15.0), (85.0, 15.0), (85.0, 65.0), (60.0, 65.0),
    (60.0, 40.0), (35.0, 40.0), (35.0, 65.0), (15.0, 65.0),
]
LINE_HALF_WIDTH = 0.5
SAMPLE_STEP = 0.5

# ---------- 小车与控制参数 ----------
SPEED = 4.0
KP = 1.8
KD = 0.25
LOOKAHEAD = 8            # 沿路径向前看的采样点数（约 4 m）
CAR_LEN, CAR_WID = 3.4, 2.0
SENSOR_OFFSETS = (-0.9, -0.45, 0.0, 0.45, 0.9)
OFFLINE_E = 1.5          # 到路径距离超过该值判定脱线
ARRIVE_LOOP = 1.0       # 跑完一圈（路径长度倍数）视为完成
DT = 1.0 / 30.0
DEFAULT_SEED = 42

CAR_COLOR = (0.20, 0.72, 0.45)


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def sample_path(waypoints, step=SAMPLE_STEP):
    """把闭合折线均匀采样成路径点数组。"""
    pts = []
    n = len(waypoints)
    for i in range(n):
        p0 = np.array(waypoints[i], dtype=float)
        p1 = np.array(waypoints[(i + 1) % n], dtype=float)
        seg = p1 - p0
        length = float(np.hypot(*seg))
        k = max(1, int(length / step))
        for j in range(k):
            pts.append(p0 + seg * j / k)
    return np.array(pts)


def nearest_index(path, x, y):
    d = np.hypot(path[:, 0] - x, path[:, 1] - y)
    i = int(np.argmin(d))
    return i, float(d[i])


class LineFollower:
    """巡线小车独轮运动学模型。"""

    def __init__(self, path, seed=DEFAULT_SEED):
        random.seed(seed)
        np.random.seed(seed)
        self.path = path
        self.x, self.y = WAYPOINTS[0]
        self.theta = 0.0
        self.v = SPEED
        self.omega = 0.0
        self.idx, self.dist = nearest_index(path, self.x, self.y)
        self.err_prev = 0.0
        self.e_prev = 0.0
        self.odom = 0.0
        self.trail = [(self.x, self.y)]
        self.offsets = []
        self.offline_steps = 0
        self.offline = False
        self.finished = False

    def _target(self):
        """沿路径前进方向取预瞄目标点。"""
        j = (self.idx + LOOKAHEAD) % len(self.path)
        return self.path[j]

    def sensor_readings(self):
        """五个传感器是否压线（True=压在线上）。"""
        readings = []
        for off in SENSOR_OFFSETS:
            sx = self.x - math.sin(self.theta) * off
            sy = self.y + math.cos(self.theta) * off
            _, dist = nearest_index(self.path, sx, sy)
            readings.append(dist <= LINE_HALF_WIDTH)
        return readings

    def step(self, dt=DT):
        target = self._target()
        f = np.array([math.cos(self.theta), math.sin(self.theta)])
        l = np.array([-math.sin(self.theta), math.cos(self.theta)])
        c = np.array([self.x, self.y])

        alpha = math.atan2(target[1] - self.y, target[0] - self.x)
        err = normalize_angle(alpha - self.theta)
        de = (err - self.err_prev) / dt
        self.omega = KP * err + KD * de
        self.err_prev = err
        self.e_prev = float((target - c) @ l)

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta = normalize_angle(self.theta + self.omega * dt)
        self.odom += self.v * dt

        self.idx, self.dist = nearest_index(self.path, self.x, self.y)
        self.offsets.append(self.e_prev)
        self.trail.append((self.x, self.y))

        if self.dist > OFFLINE_E:
            self.offline_steps += 1
        else:
            self.offline_steps = 0
        self.offline = self.offline_steps >= 15

        path_len = self._path_length()
        self.finished = self.odom >= ARRIVE_LOOP * path_len

    def _path_length(self):
        n = len(self.path)
        total = 0.0
        for i in range(n):
            total += float(np.hypot(*(self.path[(i + 1) % n] - self.path[i])))
        return total

    def metrics(self):
        arr = np.array(self.offsets) if self.offsets else np.array([0.0])
        return {
            "steps": len(self.offsets),
            "path_length": round(self.odom, 3),
            "mean_abs_offset": round(float(np.mean(np.abs(arr))), 3),
            "max_abs_offset": round(float(np.max(np.abs(arr))), 3),
            "offline": self.offline,
            "finished": self.finished,
        }


def draw(sc, ax, title):
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 80)
    ax.set_aspect("equal")
    ax.axis("off")

    closed = np.vstack([sc.path, sc.path[:1]])
    ax.plot(closed[:, 0], closed[:, 1], color="black", lw=2 * LINE_HALF_WIDTH,
            solid_capstyle="round", zorder=1)

    if len(sc.trail) > 1:
        tr = np.array(sc.trail)
        ax.plot(tr[:, 0], tr[:, 1], color=CAR_COLOR, lw=1.0, alpha=0.6, zorder=2)

    ax.add_patch(Rectangle((sc.x - CAR_LEN / 2, sc.y - CAR_WID / 2),
                           CAR_LEN, CAR_WID, facecolor=CAR_COLOR,
                           edgecolor="black", lw=1.0, zorder=4))
    fx = sc.x + math.cos(sc.theta) * CAR_LEN
    fy = sc.y + math.sin(sc.theta) * CAR_LEN
    ax.plot([sc.x, fx], [sc.y, fy], color="black", lw=1.3, zorder=5)

    readings = sc.sensor_readings()
    for off, on in zip(SENSOR_OFFSETS, readings):
        sx = sc.x - math.sin(sc.theta) * off
        sy = sc.y + math.cos(sc.theta) * off
        ax.plot(sx, sy, "o", color="green" if on else "red", ms=5, zorder=6)

    ax.set_title(title, fontsize=11)
    m = sc.metrics()
    info = (f"speed: {sc.v:.1f} m/s   offset: {sc.e_prev:.2f} m   "
            f"omega: {math.degrees(sc.omega):.1f} deg/s   "
            f"offline: {m['offline']}   t: {m['path_length']:.1f} m")
    ax.text(0.0, -0.08, info, transform=ax.transAxes, fontsize=9,
            color=(0.2, 0.2, 0.2))


def save_frame(sc, path, title):
    fig, ax = plt.subplots(figsize=(8.4, 6.8), dpi=110)
    draw(sc, ax, title)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def run_demo(outdir):
    os.makedirs(outdir, exist_ok=True)
    path = sample_path(WAYPOINTS)
    sc = LineFollower(path)
    saved = False
    steps = 0
    while not sc.finished and not sc.offline and steps < 3000:
        sc.step()
        steps += 1
        if not saved and 200 < steps < 260:
            save_frame(sc, os.path.join(outdir, "line_follower_car_track.png"),
                       "Line follower: tracking along the guide line")
            saved = True
    if not saved:
        save_frame(sc, os.path.join(outdir, "line_follower_car_track.png"),
                   "Line follower: tracking along the guide line")
    with open(os.path.join(outdir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(sc.metrics(), f, ensure_ascii=False, indent=2)
    return sc.metrics()


def run_headless(duration, seed, outdir):
    os.makedirs(outdir, exist_ok=True)
    path = sample_path(WAYPOINTS)
    sc = LineFollower(path, seed=seed)
    for _ in range(int(duration / DT)):
        sc.step()
        if sc.finished or sc.offline:
            break
    with open(os.path.join(outdir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(sc.metrics(), f, ensure_ascii=False, indent=2)
    return sc.metrics()


def main():
    parser = argparse.ArgumentParser(description="巡线小车仿真")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--outdir", default="demo_out")
    args = parser.parse_args()

    if args.demo:
        m = run_demo(args.outdir)
    else:
        m = run_headless(args.duration, args.seed, args.outdir)
    print(json.dumps(m, ensure_ascii=False))


if __name__ == "__main__":
    main()
