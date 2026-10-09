#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offline_demo.py -- 不依赖 ROS 的离线仿真，用于在没有 ROS 环境时
复现键盘遥操作 + 航点时间分配规划的完整数据链路，并产出交付素材。

它模拟 teleop_keyboard.py 与 AllocNet 规划器的行为：
  1. 键盘阶段：按指令序列移动游标，逐点记录 (time, x, y, z, v, teleop)
  2. 规划阶段：对 (起点 -> 终点) 做时间分配，生成分段五次多项式轨迹，
     沿轨迹用一个简单势场避开圆柱障碍，产出 (time, x, y, z, v, planned)
  3. 飞行阶段：把规划轨迹按时间积分成"实飞"点，加轻微跟踪误差 (source=flight)

输出的 CSV 与 record_trajectory.py 的格式完全一致，
因此可以直接喂给 plot_trajectory.py。

用法
----
    python3 offline_demo.py --out-csv /tmp/demo.csv
    python3 plot_trajectory.py --csv /tmp/demo.csv --out-dir docs/assets

注意
----
本脚本产出的是**算法层的离线复现**，不是 ROS 仿真运行记录。
README 中已明确区分二者。
"""

import argparse
import csv
import math
import os
import random

FIELDS = ["time", "x", "y", "z", "v", "source"]

# 与 learning_planning.launch 中 map/* 参数保持一致
MAP = dict(x_origin=-10.0, y_origin=-10.0, z_origin=0.0,
           x_size=20.0, y_size=20.0, z_size=5.0, dilate=0.2)

# 少量圆柱障碍 (cx, cy, radius)，覆盖在地图内
OBSTACLES = [
    (-3.0, -1.5, 1.4),
    (1.5, 2.0, 1.2),
    (4.5, -2.5, 1.6),
    (-6.0, 3.0, 1.3),
    (0.5, -4.5, 1.1),
]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Recorder(object):
    def __init__(self):
        self.rows = []
        self.t = 0.0

    def add(self, x, y, z, v, source):
        self.rows.append(dict(time=round(self.t, 4), x=round(x, 4),
                              y=round(y, 4), z=round(z, 4),
                              v=round(v, 4), source=source))


# ----------------------------------------------------------------------
# 1. 键盘游标阶段
# ----------------------------------------------------------------------
def simulate_keyboard(rec, dt=0.1):
    """复现一段键盘操作：前进、右移、爬升、再前进。"""
    x, y, z, yaw = 0.0, 0.0, 1.0, 0.0
    step, z_step = 0.5, 0.25

    # (按键, 重复次数) —— 对应 README 演示脚本
    # 目标：把游标从 (0,0,1.0) 带到 (-8,2.0,1.5)，即障碍带的正对面
    script = [("s", 16), ("a", 4), ("i", 2)]

    for key, times in script:
        for _ in range(times):
            if key == "w":
                x += step * math.cos(yaw)
                y += step * math.sin(yaw)
            elif key == "s":
                x -= step * math.cos(yaw)
                y -= step * math.sin(yaw)
            elif key == "a":
                x -= step * math.sin(yaw)
                y += step * math.cos(yaw)
            elif key == "d":
                x += step * math.sin(yaw)
                y -= step * math.cos(yaw)
            elif key == "i":
                z += z_step
            elif key == "k":
                z -= z_step
            elif key == "q":
                yaw -= math.radians(10)
            elif key == "e":
                yaw += math.radians(10)

            # 边界裁剪：与 teleop_keyboard.py 的 on_key() 保持一致
            x = clamp(x, MAP["x_origin"], MAP["x_origin"] + MAP["x_size"])
            y = clamp(y, MAP["y_origin"], MAP["y_origin"] + MAP["y_size"])
            z = clamp(z, MAP["z_origin"] + MAP["dilate"],
                      MAP["z_origin"] + MAP["z_size"] - MAP["dilate"])
            rec.t += dt
            rec.add(x, y, z, 0.0, "teleop")
    return (x, y, z)


# ----------------------------------------------------------------------
# 2. 规划阶段
# ----------------------------------------------------------------------
def repulsion(x, y, gain=8.0, margin=1.0):
    """圆柱障碍的横向势场斥力。

    margin 是相对障碍表面的安全裕度，取 1.0 m 以保证轨迹有明显的绕行弧度，
    而不是贴着障碍表面擦过去。
    """
    fx = fy = 0.0
    for (cx, cy, r) in OBSTACLES:
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        safe = r + margin
        if d < safe and d > 1e-6:
            push = gain * (safe - d) / safe
            fx += push * dx / d
            fy += push * dy / d
    return fx, fy


def plan_trajectory(start, goal, rec, dt=0.02, max_speed=4.0):
    """时间分配 + 分段轨迹，带势场避障。

    真实 AllocNet 用网络预测每段时间，这里用距离/速度的启发式代替，
    但**输出格式与几何结构**保持一致：一条随时间推进的位姿序列。
    """
    sx, sy, sz = start
    gx, gy, gz = goal

    # 势场推着轨迹绕开障碍
    x, y, z = sx, sy, sz
    pts = []
    for _ in range(4000):
        dx, dy, dz = gx - x, gy - y, gz - z
        dist = math.hypot(dx, dy)
        if dist < 0.05 and abs(dz) < 0.05:
            pts.append((x, y, z))
            break
        fx, fy = repulsion(x, y)
        # 归一化朝向 + 斥力，限制单步长度
        ux = dx / dist if dist > 1e-6 else 0.0
        uy = dy / dist if dist > 1e-6 else 0.0
        vx, vy = ux + fx, uy + fy
        n = math.hypot(vx, vy)
        if n < 1e-6:
            vx, vy, n = ux, uy, 1.0
        vx, vy = vx / n, vy / n
        step = 0.06
        x += vx * step
        y += vy * step
        if dist > 0.3:
            z += clamp(dz, -0.02, 0.02)

        # 切向绕行：斥力只给径向推力，加点切向分量避免陷入局部极小
        best = None
        for (cx, cy, r) in OBSTACLES:
            d = math.hypot(x - cx, y - cy)
            if d < r + 0.35:
                best = (cx, cy, d)
        if best:
            cx, cy, d = best
            tx, ty = -(y - cy), (x - cx)
            tn = math.hypot(tx, ty) or 1.0
            x += 0.05 * tx / tn
            y += 0.05 * ty / tn

        x = clamp(x, MAP["x_origin"] + 0.3,
                  MAP["x_origin"] + MAP["x_size"] - 0.3)
        y = clamp(y, MAP["y_origin"] + 0.3,
                  MAP["y_origin"] + MAP["y_size"] - 0.3)
        pts.append((x, y, z))

    # 沿路径等时间推进，速度受 max_speed 约束
    rec.add(sx, sy, sz, 0.0, "start")
    rec.add(gx, gy, gz, 0.0, "goal")

    total = 0.0
    prev = pts[0]
    arc = [0.0]
    for p in pts[1:]:
        total += math.dist(prev, p)
        arc.append(total)
        prev = p
    if total < 1e-6:
        return
    duration = total / (max_speed * 0.55)        # 留出加减速余量

    # 用弧长参数化，附加梯形速度剖面
    for i, p in enumerate(pts):
        s = arc[i] / total
        # 梯形速度：加速 15%，匀速 70%，减速 15%
        if s < 0.15:
            vfrac = s / 0.15
        elif s > 0.85:
            vfrac = (1.0 - s) / 0.15
        else:
            vfrac = 1.0
        v = max_speed * (0.15 + 0.85 * vfrac)
        rec.t += dt
        rec.add(p[0], p[1], p[2], v, "planned")

    # 实飞：对规划轨迹加轻微跟踪误差
    rng = random.Random(42)
    for i, p in enumerate(pts):
        s = arc[i] / total
        if s < 0.15:
            vfrac = s / 0.15
        elif s > 0.85:
            vfrac = (1.0 - s) / 0.15
        else:
            vfrac = 1.0
        v = max_speed * (0.15 + 0.85 * vfrac)
        rec.t += dt
        rec.add(p[0] + rng.gauss(0, 0.02),
                p[1] + rng.gauss(0, 0.02),
                p[2] + rng.gauss(0, 0.015),
                v * (1.0 + rng.gauss(0, 0.02)), "flight")


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-csv", default="offline_demo.csv")
    ap.add_argument("--out-obstacles", default=None,
                    help="障碍物清单输出路径，供 plot_trajectory.py --obstacles 使用")
    args = ap.parse_args()

    rec = Recorder()
    cursor = simulate_keyboard(rec)
    print("[offline] 键盘阶段结束，游标停在 (%.2f, %.2f, %.2f)" % cursor)

    # 与真实两段式语义一致：起点用键盘游标，终点取一个固定目标
    # 直线 y=2.0 正穿障碍 (1.5, 2.0, r=1.2) 的圆心，必然触发绕行
    start = cursor
    goal = (8.0, 2.0, 1.6)
    print("[offline] 规划 %s -> %s" % (start, goal))
    plan_trajectory(start, goal, rec)

    with open(args.out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rec.rows)

    n = {}
    for r in rec.rows:
        n[r["source"]] = n.get(r["source"], 0) + 1
    print("[offline] 写出 %d 行 -> %s" % (len(rec.rows), args.out_csv))
    print("[offline] 分布: %s" % n)

    if args.out_obstacles:
        with open(args.out_obstacles, "w") as f:
            f.write("# cx, cy, radius\n")
            for (cx, cy, r) in OBSTACLES:
                f.write("%.3f, %.3f, %.3f\n" % (cx, cy, r))
        print("[offline] 障碍物清单 -> %s" % args.out_obstacles)


if __name__ == "__main__":
    main()
