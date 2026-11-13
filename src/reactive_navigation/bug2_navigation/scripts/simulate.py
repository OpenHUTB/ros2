"""生成 Bug2 反应式导航教学配图。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# simple_plot 由三个模块共用一份：源码树里在 src/reactive_navigation/common/，
# colcon 安装后各包会把它拷进自己的 lib/<pkg>/，两个位置都试一下。
_HERE = os.path.dirname(os.path.abspath(__file__))
_COMMON = os.path.normpath(os.path.join(_HERE, os.pardir, os.pardir, "common"))
for _dir in (_HERE, _COMMON):
    if os.path.isfile(os.path.join(_dir, "simple_plot.py")):
        sys.path.insert(0, _dir)
        break

import simple_plot as sp
import bug2 as bug2mod


def _draw_rect(fig, rect, color):
    x, y, w, h = rect
    fig.plot([x, x + w, x + w, x, x], [y, y, y + h, y + h, y], color=color)


def _draw_cross(fig, p, color, r=1.5):
    fig.plot([p[0] - r, p[0] + r], [p[1], p[1]], color=color)
    fig.plot([p[0], p[0]], [p[1] - r, p[1] + r], color=color)


def run(outdir):
    os.makedirs(outdir, exist_ok=True)

    # 图 1：标准场景 —— m-line 直行 → 撞墙沿墙 → 更近交点离开
    res = bug2mod.navigate(bug2mod.START, bug2mod.GOAL, bug2mod.OBSTACLES)
    assert res["success"], "标准场景 Bug2 应成功"
    fig = sp.Figure(width=720, height=720)
    for rect in bug2mod.OBSTACLES:
        _draw_rect(fig, rect, sp.Figure.BLUE)
    fig.plot([bug2mod.START[0], bug2mod.GOAL[0]],
             [bug2mod.START[1], bug2mod.GOAL[1]],
             color=sp.Figure.GRAY, label="M-LINE")
    p = res["path"]
    fig.plot([q[0] for q in p], [q[1] for q in p], color=sp.Figure.RED, label="PATH")
    for (mode, pt) in res["events"]:
        if mode == "hit":
            _draw_cross(fig, pt, sp.Figure.GREEN, 1.2)
        elif mode == "leave":
            _draw_cross(fig, pt, sp.Figure.PURPLE, 1.2)
    _draw_cross(fig, bug2mod.START, sp.Figure.GREEN, 2.0)
    _draw_cross(fig, bug2mod.GOAL, sp.Figure.PURPLE, 2.0)
    fig.set_xlim(0, 50)
    fig.set_ylim(0, 50)
    fig.xlabel = "x (m)"
    fig.ylabel = "y (m)"
    fig.title = "BUG2: M-LINE, WALL FOLLOWING, LEAVE POINTS"
    fig.save(os.path.join(outdir, "bug2_map.png"))

    # 图 2：窄门腔 —— 门宽 2.0 m < 机器人安全间隙需求 2.4 m，
    # 沿墙撞到相邻墙体 → 确定性宣告不可达（A*/RRT 给不了的答案）
    walls = bug2mod.navigate(bug2mod.WALL_START, bug2mod.WALL_GOAL,
                             bug2mod.WALLS)
    assert not walls["success"], "窄门腔应被宣告不可达"
    fig2 = sp.Figure(width=720, height=720)
    for rect in bug2mod.WALLS:
        _draw_rect(fig2, rect, sp.Figure.BLUE)
    # m-line 先画、PATH 后画（评审 #1400：旧版 PATH 在下、灰色 m-line
    # 在上，把红色路径整段盖住，图上只剩一条灰线，易被误读为"穿墙到达"）
    fig2.plot([bug2mod.WALL_START[0], bug2mod.WALL_GOAL[0]],
              [bug2mod.WALL_START[1], bug2mod.WALL_GOAL[1]],
              color=sp.Figure.GRAY, label="M-LINE")
    pw = walls["path"]
    fig2.plot([q[0] for q in pw], [q[1] for q in pw], color=sp.Figure.RED,
              label="PATH")
    blocked = [pt for (mode, pt) in walls["events"] if mode == "blocked"]
    for pt in blocked:
        _draw_cross(fig2, pt, sp.Figure.PURPLE, 1.5)
    _draw_cross(fig2, bug2mod.WALL_START, sp.Figure.GREEN, 2.0)
    _draw_cross(fig2, bug2mod.WALL_GOAL, sp.Figure.PURPLE, 2.0)
    fig2.set_xlim(10, 50)
    fig2.set_ylim(0, 50)
    fig2.xlabel = "x (m)"
    fig2.ylabel = "y (m)"
    fig2.title = "GATE TOO NARROW: DETERMINISTIC UNREACHABLE"
    fig2.save(os.path.join(outdir, "bug2_walls.png"))

    # 图 3：安全间隙 r_clear 扫描 —— 间隙越大，膨胀边界越远、沿墙弧
    # 越长，总长在 85→87 m 间抬升（局部回落来自离开交点的离散跳变，
    # 并非严格单调）。只画 LENGTH 一个量（评审 #1400：DETOURS 恒 0，
    # 与 85~87 m 的长度共轴会被压成贴底直线）。
    rows = []
    for rc in (0.8, 1.0, 1.2, 1.4, 1.6):
        r = bug2mod.navigate(bug2mod.START, bug2mod.GOAL, bug2mod.OBSTACLES,
                             r_clear=rc)
        rows.append((rc, r["length"], r["hits"], r["success"]))
    assert all(r[3] for r in rows)
    xs = [r[0] for r in rows]
    fig3 = sp.Figure(width=760, height=440)
    fig3.plot(xs, [r[1] for r in rows], color=sp.Figure.BLUE, label="LENGTH")
    fig3.set_xlim(0.7, 1.7)
    fig3.set_ylim(82.0, 90.0)
    fig3.xlabel = "r_clear (m)  (safe clearance to wall)"
    fig3.ylabel = "path length (m)"
    fig3.title = "SWEEP: PATH GROWS WITH CLEARANCE r_clear"
    fig3.save(os.path.join(outdir, "bug2_sweep.png"))

    m_line = ((bug2mod.START[0] - bug2mod.GOAL[0]) ** 2
              + (bug2mod.START[1] - bug2mod.GOAL[1]) ** 2) ** 0.5

    return {
        "std_len": res["length"],
        "std_detours": res["detours"],
        "std_events": len(res["events"]),
        "std_hits": res["hits"],
        "walls_len": walls["length"],
        "walls_blocked": bool(blocked),
        "sweep_len_min": min(r[1] for r in rows),
        "sweep_len_max": max(r[1] for r in rows),
        "m_line": m_line,
    }
