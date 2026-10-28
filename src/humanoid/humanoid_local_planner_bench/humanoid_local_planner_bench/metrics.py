# -*- coding: utf-8 -*-
"""一次实验的指标统计。"""

import math
import statistics


class RunRecorder:
    def __init__(self):
        self.samples = []

    def add(self, t, pose, v, w, clearance, collided):
        self.samples.append({"t": t, "x": pose[0], "y": pose[1], "yaw": pose[2],
                             "v": v, "w": w, "clearance": clearance,
                             "collided": bool(collided)})

    def summary(self, goal, goal_tol=0.35):
        s = self.samples
        if not s:
            # 一帧都没跑（如 time_limit 过小）：返回完整键，避免调用方 KeyError
            return {"reached": False, "time": 0.0, "path_len": 0.0,
                    "straight_len": 0.0, "min_clearance": 0.0, "collision_steps": 0,
                    "mean_w": 0.0, "max_w": 0.0,
                    "final_dist": float("inf"), "end_xy": None, "samples": 0}
        x0, y0 = s[0]["x"], s[0]["y"]
        path = math.hypot(x0 - goal[0], y0 - goal[1])
        for a, b in zip(s, s[1:]):
            path += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        end = s[-1]
        dist = math.hypot(end["x"] - goal[0], end["y"] - goal[1])
        ws = [abs(p["w"]) for p in s]
        return {
            "reached": dist <= goal_tol,
            "time": end["t"],
            "path_len": path,
            "straight_len": math.hypot(goal[0] - x0, goal[1] - y0),
            "min_clearance": min(p["clearance"] for p in s),
            "collision_steps": sum(1 for p in s if p["collided"]),
            "mean_w": statistics.fmean(ws) if ws else 0.0,
            "max_w": max(ws) if ws else 0.0,
            "final_dist": dist,
            "end_xy": (end["x"], end["y"]),
        }

    def write_csv(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        cols = ["t", "x", "y", "yaw", "v", "w", "clearance", "collided"]
        with open(path, "w", encoding="utf-8") as f:
            f.write(",".join(cols) + "\n")
            for s in self.samples:
                f.write(",".join(str(int(s[c])) if c == "collided" else "{:.5f}".format(s[c])
                                 for c in cols) + "\n")


# 场景里障碍物的平面位置（与 corridor_scene.xml 保持一致），仅用于出图
SCENE_WALLS = [
    ("wall_south_a", (1.5, -1.60), (2.50, 0.06)),
    ("wall_south_b", (6.5, -1.60), (2.50, 0.06)),
    ("wall_north_a", (1.5, 1.60), (2.50, 0.06)),
    ("wall_north_b", (6.5, 1.60), (2.50, 0.06)),
    # 门柱内缘 y=±0.45 → 净宽 0.90 m（必须与 corridor_scene.xml 一致）
    ("door_jamb_s", (4.0, -1.025), (0.10, 0.575)),
    ("door_jamb_n", (4.0, 1.025), (0.10, 0.575)),
]
SCENE_CIRCLES = [((2.6, 0.00), 0.22), ((6.0, 0.35), 0.22)]


def plot_compare(named_records, goal, path, footprint_r=0.28):
    """三联图：轨迹 / 最小间距时程 / 速度时程。缺 matplotlib 时返回 False。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
    except Exception:                                        # noqa: BLE001
        return False

    colors = {"dwa": "tab:blue", "apf": "tab:green"}
    fig = plt.figure(figsize=(14, 9.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.55, 1.0], hspace=0.28, wspace=0.20)
    ax = fig.add_subplot(gs[0, :])
    ax_c = fig.add_subplot(gs[1, 0])
    ax_v = fig.add_subplot(gs[1, 1])

    # ---------- (a) 轨迹 ----------
    for name, (cx, cy), (hx, hy) in SCENE_WALLS:
        ax.add_patch(Rectangle((cx - hx, cy - hy), 2 * hx, 2 * hy,
                               facecolor="0.62", edgecolor="0.45", lw=0.8))
    for (cx, cy), r in SCENE_CIRCLES:
        ax.add_patch(Circle((cx, cy), r, facecolor="#cc4c47", edgecolor="none"))

    for name, rec in named_records:
        xs = [q["x"] for q in rec.samples]
        ys = [q["y"] for q in rec.samples]
        c = colors.get(name, "k")
        ax.plot(xs, ys, color=c, lw=2.2, label=name, zorder=3)
        # 每 60 个样本画一个足迹圆，体现"人形宽度"而不是质点
        for i in range(0, len(xs), 60):
            ax.add_patch(Circle((xs[i], ys[i]), footprint_r, facecolor="none",
                                edgecolor=c, lw=0.7, alpha=0.45, zorder=2))

    ax.plot([goal[0]], [goal[1]], "k*", ms=20, label="goal", zorder=4)
    ax.set_aspect("equal")
    ax.set_xlim(-1.0, 9.4)
    ax.set_ylim(-2.2, 2.2)
    ax.set_xlabel("x (m)", fontsize=11)
    ax.set_ylabel("y (m)", fontsize=11)
    ax.set_title("(a) trajectories in 8 m corridor with 0.90 m door  "
                 "(dashed circles = footprint r = 0.28 m)", fontsize=12)
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=10)

    # ---------- (b)(c) 时程 ----------
    for name, rec in named_records:
        c = colors.get(name, "k")
        t = [q["t"] for q in rec.samples]
        ax_c.plot(t, [q["clearance"] for q in rec.samples], color=c, lw=1.6, label=name)
        ax_v.plot(t, [q["v"] for q in rec.samples], color=c, lw=1.6, label=name)

    ax_c.axhline(0.0, color="k", lw=0.8, ls="--")
    ax_c.set_xlabel("t (s)", fontsize=11)
    ax_c.set_ylabel("clearance (m)", fontsize=11)
    ax_c.set_title("(b) distance from footprint to nearest obstacle", fontsize=11)
    ax_c.grid(alpha=0.25)
    ax_c.legend(fontsize=9)

    ax_v.set_xlabel("t (s)", fontsize=11)
    ax_v.set_ylabel("v (m/s)", fontsize=11)
    ax_v.set_title("(c) commanded linear speed", fontsize=11)
    ax_v.grid(alpha=0.25)

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return True
