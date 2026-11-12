"""生成 RRT 路径规划教学配图。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# simple_plot 由三个模块共用一份：源码树里在 src/motion_planning/common/，
# colcon 安装后各包会把它拷进自己的 lib/<pkg>/，两个位置都试一下。
_HERE = os.path.dirname(os.path.abspath(__file__))
_COMMON = os.path.normpath(os.path.join(_HERE, os.pardir, os.pardir, "common"))
for _dir in (_HERE, _COMMON):
    if os.path.isfile(os.path.join(_dir, "simple_plot.py")):
        sys.path.insert(0, _dir)
        break

import simple_plot as sp
import rrt as rrtmod


def _draw_rect(fig, rect, color):
    x0, y0, x1, y1 = rect
    fig.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color=color)


def _draw_cross(fig, p, color, r=2.0):
    fig.plot([p[0] - r, p[0] + r], [p[1], p[1]], color=color)
    fig.plot([p[0], p[0]], [p[1] - r, p[1] + r], color=color)


def run(outdir):
    os.makedirs(outdir, exist_ok=True)

    planner = rrtmod.RRT(rrtmod.START, rrtmod.GOAL, rrtmod.OBSTACLES, seed=3)
    path = planner.plan()
    assert path is not None, "RRT 未找到路径"
    smoothed = rrtmod.smooth(path, rrtmod.OBSTACLES, iterations=300, seed=5)

    # 图 1：树 + 障碍 + 平滑后路径
    fig = sp.Figure(width=720, height=720)
    for a, b in planner.edges:                      # 树
        fig.plot([a[0], b[0]], [a[1], b[1]], color=(215, 215, 215))
    for rect in rrtmod.OBSTACLES:                   # 障碍
        _draw_rect(fig, rect, sp.Figure.BLUE)
    fig.plot([p[0] for p in smoothed], [p[1] for p in smoothed],
             color=sp.Figure.RED, label="PATH")
    _draw_cross(fig, rrtmod.START, sp.Figure.GREEN, 3.0)
    _draw_cross(fig, rrtmod.GOAL, sp.Figure.PURPLE, 3.0)
    fig.set_xlim(0, 100)
    fig.set_ylim(0, 100)
    fig.xlabel = "x (m)"
    fig.ylabel = "y (m)"
    fig.title = "RRT: TREE, OBSTACLES, SMOOTHED PATH"
    fig.save(os.path.join(outdir, "rrt_map.png"))

    # 图 2：多次独立运行——原始路径 vs 平滑后
    runs = []
    for seed in (3, 11, 19, 27, 35):
        pl = rrtmod.RRT(rrtmod.START, rrtmod.GOAL, rrtmod.OBSTACLES, seed=seed)
        p = pl.plan()
        if p is None:
            continue
        s = rrtmod.smooth(p, rrtmod.OBSTACLES, seed=seed + 1)
        runs.append((seed, rrtmod.path_length(p), rrtmod.path_length(s)))
    straight = rrtmod.dist(rrtmod.START, rrtmod.GOAL)

    xs = [i + 1 for i, _ in enumerate(runs)]
    fig2 = sp.Figure(width=760, height=440)
    fig2.plot(xs, [r[1] for r in runs], color=sp.Figure.BLUE, label="RAW")
    fig2.plot(xs, [r[2] for r in runs], color=sp.Figure.RED, label="SMOOTH")
    fig2.plot([xs[0], xs[-1]], [straight, straight], color=sp.Figure.GRAY,
              label="STRAIGHT")
    fig2.xlabel = "run"
    fig2.ylabel = "length (m)"
    fig2.title = "PATH LENGTH: RAW VS SMOOTHED (5 RUNS)"
    fig2.save(os.path.join(outdir, "rrt_lengths.png"))

    return {
        "found": True,
        "nodes": len(planner.nodes),
        "edges": len(planner.edges),
        "raw_len": rrtmod.path_length(path),
        "smooth_len": rrtmod.path_length(smoothed),
        "straight": straight,
        "runs": runs,
    }
