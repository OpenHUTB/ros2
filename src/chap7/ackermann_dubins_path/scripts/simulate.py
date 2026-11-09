"""Dubins 路径的算例与统计。

把文档里用到的数字都集中在这里，main.py 负责画图与打印，tests 只测算法本身。
"""

import math

from dubins import dubins_path, all_paths, wrap_pi

R_DEFAULT = 1.0


def scenario_lines(radius=R_DEFAULT):
    """三种典型工况：同向、换向 90°、原地掉头。"""
    return [
        ("同向直行", (0.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
        ("换向 90°", (0.0, 0.0, 0.0), (10.0, 0.0, math.pi / 2)),
        ("掉头 180°", (0.0, 0.0, 0.0), (10.0, 0.0, math.pi)),
        ("斜向换向", (0.0, 0.0, 0.0), (6.0, 4.0, -math.pi / 2)),
    ]


def scenario_table(radius=R_DEFAULT):
    """每个工况的最优情形、长度、以及比次优解省多少。"""
    rows = []
    for name, start, goal in scenario_lines(radius):
        paths = all_paths(start, goal, radius)
        best = paths[0]
        second = paths[1].length if len(paths) > 1 else float('nan')
        straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
        rows.append({
            'name': name,
            'start': start,
            'goal': goal,
            'case': best.case,
            'length': best.length,
            'segments': best.segments,
            'second': second,
            'saving': (second - best.length) / second if len(paths) > 1 else 0.0,
            'straight': straight,
            'ratio': best.length / straight if straight > 1e-9 else 1.0,
            'n_valid': len(paths),
        })
    return rows


def case_breakdown(start, goal, radius=R_DEFAULT):
    """六个情形各自的长度（不可行的记为 None），用于展示「谁输谁赢」。"""
    out = []
    seen = {p.case: p for p in all_paths(start, goal, radius)}
    for case in ('LSL', 'LSR', 'RSL', 'RSR', 'RLR', 'LRL'):
        p = seen.get(case)
        out.append((case, p.length if p else None))
    best_len = min(v for _, v in out if v is not None)
    return [(c, l, (abs(l - best_len) < 1e-9) if l else False) for c, l in out]


def heading_sweep(radius=R_DEFAULT, start=(0.0, 0.0, 0.0), dist=10.0, n=181):
    """固定起终点位置，扫末端朝向，看最短长度与获胜情形如何变化。"""
    headings = []
    lengths = []
    cases = []
    for i in range(n):
        th = -math.pi + 2.0 * math.pi * i / (n - 1)
        goal = (dist, 0.0, th)
        p = dubins_path(start, goal, radius)
        headings.append(th)
        lengths.append(p.length)
        cases.append(p.case)
    return headings, lengths, cases


def radius_sweep(start=(0.0, 0.0, 0.0), goal=(10.0, 0.0, math.pi),
                 radii=(1.0, 2.0, 3.0)):
    """同一工况、不同最小转弯半径下的路径与长度。"""
    out = []
    for r in radii:
        p = dubins_path(start, goal, r)
        out.append((r, p))
    return out


def curvature_stats(path, step=0.02):
    """采样路径，统计实际曲率是否越界、总弧长分布。"""
    pts = path.sample(step=step)
    max_kappa = max(abs(pt[3]) for pt in pts)
    arc_len = sum(s[1] for s in path.segments if s[0] in ('L', 'R'))
    return {
        'max_kappa': max_kappa,
        'limit': 1.0 / path.radius,
        'arc_length': arc_len,
        'straight_length': sum(s[1] for s in path.segments if s[0] == 'S'),
        'n_samples': len(pts),
    }


def endpoint_precision(cases, radius=R_DEFAULT):
    """端点闭环：采样积分回到终点的最大位置 / 朝向误差。"""
    worst_pos = 0.0
    worst_ang = 0.0
    for start, goal in cases:
        p = dubins_path(start, goal, radius)
        dp, dth = p.endpoint_error(goal)
        worst_pos = max(worst_pos, dp)
        worst_ang = max(worst_ang, dth)
    return worst_pos, worst_ang
