# -*- coding: utf-8 -*-
"""窄道会车让行仿真（纯 NumPy + Matplotlib，不依赖 ROS / Gazebo / CARLA）。

场景：一条双向共用的单车道窄路，路侧设有两处避让龛。两辆对向小车按
“谁先靠近自己一侧的避让龛，谁进龛等待”的规则会车：让行车驶入避让龛停稳，
对向车直行通过，交汇完成后让行车再驶出避让龛继续通行。

用法：
    python main.py --demo --outdir demo_out     # 跑场景并保存过程截图
    python main.py --headless --duration 40 --outdir demo_out
                                                # 无窗口评测，写 metrics.json
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

# ---------- 场景几何（单位：米） ----------
ROAD_X0, ROAD_X1 = 0.0, 120.0
ROAD_Y0, ROAD_Y1 = 40.0, 56.0
LANE_CY = (ROAD_Y0 + ROAD_Y1) / 2.0

CAR_LEN, CAR_WID = 5.0, 2.6
CAR_R = 2.5

# 避让龛：x 区间 + 停靠中心位置
NICHES = [
    {"x0": 28.0, "x1": 44.0, "cx": 36.0, "cy": 64.0},
    {"x0": 76.0, "x1": 92.0, "cx": 84.0, "cy": 64.0},
]

DT = 1.0 / 30.0
MAX_SPEED = 6.0
ARRIVE_R = 1.2
DEFAULT_SEED = 42

COLOR_A = (0.235, 0.627, 0.980)   # 蓝
COLOR_B = (0.902, 0.275, 0.275)   # 红


def normalize_angle(a):
    """把角度折到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Car:
    """二维独轮运动学小车。"""

    def __init__(self, x, y, theta, role):
        self.x = x
        self.y = y
        self.theta = theta
        self.role = role            # "A" 左 -> 右；"B" 右 -> 左
        self.v = 0.0
        self.state = "to_niche"
        self.trail = []
        self.wait_time = 0.0
        self.niche = None           # 分配给自己的避让龛

    def move_toward(self, tx, ty, dt):
        """朝目标点移动一步，返回是否已到达。"""
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist <= ARRIVE_R:
            self.v = 0.0
            return True
        step = min(MAX_SPEED * dt, dist)
        self.x += dx / dist * step
        self.y += dy / dist * step
        self.theta = math.atan2(dy, dx)
        self.v = step / dt
        return False

    def stop(self):
        self.v = 0.0


def decide_yield(car_a, car_b):
    """按到各自前方避让龛的距离决定让行车：距离更近者进龛等待。"""
    car_a.niche = NICHES[0]
    car_b.niche = NICHES[1]
    da = abs(car_a.niche["cx"] - car_a.x)
    db = abs(car_b.niche["cx"] - car_b.x)
    return car_a if da <= db else car_b


class Scenario:
    """会车场景状态机。"""

    def __init__(self, seed=DEFAULT_SEED):
        random.seed(seed)
        np.random.seed(seed)
        self.a = Car(8.0, LANE_CY, 0.0, "A")
        self.b = Car(112.0, LANE_CY, math.pi, "B")
        self.yield_car = decide_yield(self.a, self.b)
        self.pass_car = self.b if self.yield_car is self.a else self.a
        self.yield_car.state = "to_niche"
        self.pass_car.state = "cruise"
        self.collisions = 0
        self._blocked_last = False
        self.min_headway = float("inf")
        self.time = 0.0
        self.done = False

    def _check_collision(self):
        headway = math.hypot(self.a.x - self.b.x, self.a.y - self.b.y)
        self.min_headway = min(self.min_headway, headway)
        if headway < 2.0 * CAR_R:
            if not self._blocked_last:
                self.collisions += 1
            self._blocked_last = True
        else:
            self._blocked_last = False

    def _cruise(self, car, dt):
        if car.role == "A":
            car.move_toward(ROAD_X1, LANE_CY, dt)
        else:
            car.move_toward(ROAD_X0, LANE_CY, dt)

    def step(self, dt=DT):
        if self.done:
            return
        self.time += dt
        yc, pc = self.yield_car, self.pass_car
        n = yc.niche

        # 对向车直行通过
        if pc.state == "cruise":
            self._cruise(pc, dt)

        # 让行车状态机：to_niche -> enter -> waiting -> exit -> cruise
        if yc.state == "to_niche":
            if yc.role == "A":
                if yc.x >= n["x0"] - 2.0:
                    yc.state = "enter"
                else:
                    yc.move_toward(n["x0"], LANE_CY, dt)
            else:
                if yc.x <= n["x1"] + 2.0:
                    yc.state = "enter"
                else:
                    yc.move_toward(n["x1"], LANE_CY, dt)

        elif yc.state == "enter":
            if yc.move_toward(n["cx"], n["cy"], dt):
                yc.stop()
                yc.state = "waiting"

        elif yc.state == "waiting":
            yc.stop()
            yc.wait_time += dt
            cleared = (pc.x < n["x0"]) if pc.role == "B" else (pc.x > n["x1"])
            if cleared:
                yc.state = "exit"

        elif yc.state == "exit":
            if yc.move_toward(yc.x, LANE_CY, dt):
                yc.state = "cruise"

        elif yc.state == "cruise":
            self._cruise(yc, dt)

        self._check_collision()
        self.a.trail.append((self.a.x, self.a.y))
        self.b.trail.append((self.b.x, self.b.y))

        a_done = (self.a.x >= ROAD_X1 - ARRIVE_R) if self.a.role == "A" else (self.a.x <= ROAD_X0 + ARRIVE_R)
        b_done = (self.b.x >= ROAD_X1 - ARRIVE_R) if self.b.role == "A" else (self.b.x <= ROAD_X0 + ARRIVE_R)
        self.done = a_done and b_done

    def metrics(self):
        return {
            "yield_role": self.yield_car.role,
            "pass_role": self.pass_car.role,
            "collisions": self.collisions,
            "min_headway": round(self.min_headway, 3),
            "wait_time": round(self.yield_car.wait_time, 3),
            "total_time": round(self.time, 3),
            "passed": self.done and self.collisions == 0,
        }


# ---------- 渲染 ----------

def draw_scenario(scenario, ax, title):
    ax.set_xlim(-4, ROAD_X1 + 4)
    ax.set_ylim(28, 74)
    ax.set_aspect("equal")
    ax.axis("off")

    # 路面
    ax.add_patch(Rectangle((ROAD_X0, ROAD_Y0), ROAD_X1 - ROAD_X0,
                           ROAD_Y1 - ROAD_Y0, facecolor=(0.35, 0.35, 0.40),
                           edgecolor="none", zorder=1))
    # 避让龛
    for nn in NICHES:
        ax.add_patch(Rectangle((nn["x0"], ROAD_Y1), nn["x1"] - nn["x0"],
                               nn["cy"] - ROAD_Y1, facecolor=(0.35, 0.35, 0.40),
                               edgecolor="none", zorder=1))
    # 车道虚线
    for x in np.arange(ROAD_X0 + 4, ROAD_X1, 8):
        ax.plot([x, x + 3], [LANE_CY, LANE_CY], color=(0.9, 0.9, 0.9),
                lw=1.2, zorder=2)

    # 轨迹
    if len(scenario.a.trail) > 1:
        ta = np.array(scenario.a.trail)
        ax.plot(ta[:, 0], ta[:, 1], color=COLOR_A, lw=1.0, alpha=0.55, zorder=3)
    if len(scenario.b.trail) > 1:
        tb = np.array(scenario.b.trail)
        ax.plot(tb[:, 0], tb[:, 1], color=COLOR_B, lw=1.0, alpha=0.55, zorder=3)

    _draw_car(ax, scenario.a, COLOR_A)
    _draw_car(ax, scenario.b, COLOR_B)

    ax.text(scenario.a.x, scenario.a.y + 4.2, f"A [{scenario.a.state}]",
            color=COLOR_A, ha="center", fontsize=9, zorder=5)
    ax.text(scenario.b.x, scenario.b.y - 5.2, f"B [{scenario.b.state}]",
            color=COLOR_B, ha="center", fontsize=9, zorder=5)

    ax.set_title(title, fontsize=11)
    m = scenario.metrics()
    info = (f"yield: {m['yield_role']}   collisions: {m['collisions']}   "
            f"min headway: {m['min_headway']} m   wait: {m['wait_time']} s   "
            f"t: {m['total_time']} s")
    ax.text(0.0, -0.08, info, transform=ax.transAxes, fontsize=9,
            color=(0.2, 0.2, 0.2))


def _draw_car(ax, car, color):
    dx = math.cos(car.theta) * CAR_LEN / 2
    dy = math.sin(car.theta) * CAR_LEN / 2
    ax.add_patch(Rectangle((car.x - CAR_LEN / 2, car.y - CAR_WID / 2),
                           CAR_LEN, CAR_WID, facecolor=color,
                           edgecolor=(0, 0, 0), lw=1.0, zorder=4))
    ax.plot([car.x, car.x + dx * 1.6], [car.y, car.y + dy * 1.6],
           color="black", lw=1.4, zorder=5)


def save_frame(scenario, path, title):
    fig, ax = plt.subplots(figsize=(11, 4.6), dpi=110)
    draw_scenario(scenario, ax, title)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def run_demo(outdir):
    os.makedirs(outdir, exist_ok=True)
    sc = Scenario()
    frame_yield = False
    frame_passed = False
    steps = 0
    while not sc.done and steps < 3000:
        sc.step()
        steps += 1
        if (not frame_yield and sc.yield_car.state == "waiting"
                and 55 < sc.pass_car.x < 70):
            save_frame(sc, os.path.join(outdir, "narrow_giveway_passing_yield.png"),
                       "Narrow passing: yield car waiting in niche")
            frame_yield = True
        if (not frame_passed and sc.yield_car.state == "cruise"
                and sc.yield_car.wait_time > 0.0):
            save_frame(sc, os.path.join(outdir, "narrow_giveway_passing_passed.png"),
                       "Narrow passing: pass car cleared, yield car exits niche")
            frame_passed = True
    if not frame_yield:
        save_frame(sc, os.path.join(outdir, "narrow_giveway_passing_yield.png"),
                   "Narrow passing: yield car waiting in niche")
    if not frame_passed:
        save_frame(sc, os.path.join(outdir, "narrow_giveway_passing_passed.png"),
                   "Narrow passing: after passing")
    with open(os.path.join(outdir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(sc.metrics(), f, ensure_ascii=False, indent=2)
    return sc.metrics()


def run_headless(duration, seed, outdir):
    os.makedirs(outdir, exist_ok=True)
    sc = Scenario(seed=seed)
    steps = int(duration / DT)
    for _ in range(steps):
        sc.step()
        if sc.done:
            break
    with open(os.path.join(outdir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(sc.metrics(), f, ensure_ascii=False, indent=2)
    return sc.metrics()


def main():
    parser = argparse.ArgumentParser(description="窄道会车让行仿真")
    parser.add_argument("--demo", action="store_true", help="跑场景并保存截图")
    parser.add_argument("--headless", action="store_true", help="无窗口评测")
    parser.add_argument("--duration", type=float, default=40.0, help="headless 时长(秒)")
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
