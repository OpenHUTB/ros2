#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Boustrophedon coverage planner with obstacle detours and semicircle turns.

覆盖路径规划：旋转卡壳求凸多边形最小宽方向 → 沿该方向布往返扫描线
（间距 = 喷幅 × (1 - 重叠率)）→ 遇圆形禁区沿外扩边界圆弧绕行 → 行间
转入用半圆弧衔接。输出逐行航段与总航程/时长，供 checker / flyer 复用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np

from .geometry import (CircleObstacle, line_polygon_span, min_width_direction,
                       segment_point_distance)

TAG = "[planner]"


@dataclass(frozen=True)
class SprayConfig:
    """Mission-wide spray parameters (dataclass config, no class hierarchies)."""

    spray_width: float = 3.0     # 喷幅 m
    overlap: float = 0.2         # 重叠率
    speed: float = 4.0           # 作业速度 m/s
    alt: float = 2.5             # 作业高度 m
    obstacle_margin: float = 5.0  # 障碍外扩绕行距离 m
    arc_step_deg: float = 10.0   # 圆弧采样步长
    sigma_factor: float = 0.25   # 沉积横向标准差 = 喷幅 * sigma_factor

    @property
    def spacing(self) -> float:
        return self.spray_width * (1.0 - self.overlap)

    @property
    def sigma(self) -> float:
        return self.spray_width * self.sigma_factor


@dataclass
class ScanLine:
    """One boustrophedon sweep line (straight runs + detour/turn arcs)."""

    line_id: int
    straight: List[Tuple[np.ndarray, np.ndarray]] = field(default_factory=list)
    arcs: List[np.ndarray] = field(default_factory=list)   # 障碍绕行弧（随行喷洒）
    turn_arc: np.ndarray = None                            # 行间半圆转入弧
    points: np.ndarray = None
    length: float = 0.0
    strip_area: float = 0.0      # 本行名义覆盖条带面积 m^2


@dataclass
class SprayPath:
    """Flattened mission path for one field (possibly multi-subregion)."""

    lines: List[ScanLine]
    points: np.ndarray           # (N, 2) 依次航点
    seg_line: np.ndarray         # (N-1,) 每段所属行号
    total_length: float
    duration: float
    config: SprayConfig
    subregions: int = 1

    def straight_segments(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """All straight scan runs (deposition sources for the checker)."""
        return [seg for ln in self.lines for seg in ln.straight]

    def deposit_sources(self) -> List[np.ndarray]:
        """Sprayed pieces: straight runs + detour arcs (turns excluded)."""
        out: List[np.ndarray] = []
        for ln in self.lines:
            out.extend(ln.straight)
            out.extend(ln.arcs)
        return out


def _arc_points(center: np.ndarray, radius: float, a0: float, a1: float,
                step_deg: float) -> np.ndarray:
    """Sample an arc from angle a0 to a1 (a1 > a0, ccw) into points."""
    k = max(2, int(math.ceil(abs(a1 - a0) / math.radians(step_deg))))
    ang = np.linspace(a0, a1, k)
    return center + radius * np.stack([np.cos(ang), np.sin(ang)], axis=1)


def _detour_segment(a: np.ndarray, b: np.ndarray, obs: CircleObstacle,
                    margin: float, step_deg: float
                    ) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], List[np.ndarray]]:
    """Split one straight run around an expanded circular obstacle.

    返回（直行段列表, 弧段列表）：A→入弧点 直行，沿半径 r+margin 的圆
    走劣弧（恒距障碍中心 R，天然满足安全间隙），出弧点→B 直行。
    """
    R = obs.radius + margin
    ab = b - a
    L = float(np.linalg.norm(ab))
    if L < 1e-9:
        return [(a, b)], []
    if float(segment_point_distance(a, b, obs.center.reshape(1, 2))[0]) >= R - 1e-9:
        return [(a, b)], []
    u = ab / L
    nvec = np.array([-u[1], u[0]])
    rel = obs.center - a
    tf = float(rel @ u)
    h = float(rel @ nvec)
    if abs(h) < 1e-9:
        h = 0.0
    half = math.sqrt(max(R * R - h * h, 0.0))
    t0 = max(0.0, min(1.0, (tf - half) / L))
    t1 = max(0.0, min(1.0, (tf + half) / L))
    if t1 - t0 < 1e-6:
        return [(a, b)], []
    e_pt = a + t0 * L * u
    x_pt = a + t1 * L * u
    c = obs.center
    ang_e = math.atan2(e_pt[1] - c[1], e_pt[0] - c[0])
    ang_x = math.atan2(x_pt[1] - c[1], x_pt[0] - c[0])
    # 劣弧：从 ang_e 逆时针到 ang_x 与顺时针两条中取短的那条
    ccw = (ang_x - ang_e) % (2 * math.pi)
    cw = (ang_e - ang_x) % (2 * math.pi)
    if ccw <= cw:
        arc = _arc_points(c, R, ang_e, ang_e + ccw, step_deg)
    else:
        arc = _arc_points(c, R, ang_x, ang_x + cw, step_deg)[::-1]
    straights = []
    if np.linalg.norm(a - e_pt) > 1e-6:
        straights.append((a, e_pt))
    if np.linalg.norm(x_pt - b) > 1e-6:
        straights.append((x_pt, b))
    return straights, [arc]


def _turn_arc(p_end: np.ndarray, p_start: np.ndarray, out_dir: np.ndarray,
              step_deg: float) -> np.ndarray:
    """Semicircular U-turn between two adjacent line ends (radius = spacing/2)."""
    m = 0.5 * (p_end + p_start)
    r = float(np.linalg.norm(p_end - p_start)) / 2.0
    if r < 1e-9:
        return np.stack([p_end, p_start])
    u = (p_end - m) / r
    w = np.array([-u[1], u[0]])
    if w @ out_dir < 0:
        w = -w
    a0 = math.atan2(u[1], u[0])
    aw = math.atan2(w[1], w[0])
    sgn = 1.0 if ((aw - a0) % (2 * math.pi)) < math.pi else -1.0
    return _arc_points(m, r, a0, a0 + sgn * math.pi, step_deg)


def plan_convex(polygon: np.ndarray, obstacles: Sequence[CircleObstacle],
                cfg: SprayConfig, first_line_id: int = 0) -> List[ScanLine]:
    """Plan boustrophedon lines for one convex subregion."""
    if cfg.spacing <= 1e-9:
        raise ValueError("overlap too high: effective spacing is zero")
    poly = np.asarray(polygon, dtype=float)
    d, _width = min_width_direction(poly)
    nvec = np.array([-d[1], d[0]])
    proj = poly @ nvec
    w0, w1 = float(proj.min()), float(proj.max())
    extent = w1 - w0
    n_lines = max(1, int(round(extent / cfg.spacing)))
    s_eff = extent / n_lines
    center = poly.mean(axis=0)
    pc = float(center @ nvec)

    lines: List[ScanLine] = []
    for k in range(n_lines):
        c = w0 + extent * (k + 0.5) / n_lines
        base = center + (c - pc) * nvec
        span = line_polygon_span(base, d, poly)
        if span is None:
            continue
        t0, t1 = span
        pa, pb = base + t0 * d, base + t1 * d
        a = pa if k % 2 == 0 else pb
        b = pb if k % 2 == 0 else pa

        straight = [(a, b)]
        arcs: List[np.ndarray] = []
        # 障碍按沿线先后排序，逐个把当前末端直行段替换为 绕行弧+剩余直行
        ordered = sorted(obstacles,
                         key=lambda o: float((o.center - a) @ (b - a)))
        for obs in ordered:
            p0, p1 = straight[-1]
            segs, darc = _detour_segment(p0, p1, obs,
                                         cfg.obstacle_margin, cfg.arc_step_deg)
            straight[-1:] = segs
            arcs.extend(darc)
        straight = [(s0, s1) for s0, s1 in straight
                    if np.linalg.norm(s1 - s0) > 1e-6]

        # 直行段 + 弧段按沿线顺序拼成行内航点序列
        pts: List[np.ndarray] = [a]
        pieces = []
        for s0, s1 in straight:
            pieces.append((float((s0 - a) @ (b - a)), "s", (s0, s1)))
        for arc in arcs:
            pieces.append((float((arc[0] - a) @ (b - a)), "a", arc))
        pieces.sort(key=lambda p: p[0])
        for _, kind, obj in pieces:
            if kind == "s":
                pts.append(obj[1])
            else:
                pts.extend(obj[1:])
        pts_arr = np.asarray(pts, dtype=float)
        length = float(sum(np.linalg.norm(pts_arr[i + 1] - pts_arr[i])
                           for i in range(len(pts_arr) - 1)))
        ln = ScanLine(line_id=first_line_id + len(lines), straight=straight,
                      arcs=arcs, points=pts_arr, length=length,
                      strip_area=s_eff * float(np.linalg.norm(b - a)))
        lines.append(ln)

    # 行间半圆弧 U 形转弯：衔接第 k 行末端与第 k+1 行起点（同一侧）
    for k in range(len(lines) - 1):
        p_end = lines[k].points[-1]
        p_start = lines[k + 1].points[0]
        s0, s1 = lines[k].straight[-1]
        out = s1 - s0
        norm = float(np.linalg.norm(out))
        out = out / norm if norm > 1e-9 else d
        turn = _turn_arc(p_end, p_start, out, cfg.arc_step_deg)
        lines[k + 1].turn_arc = turn
        lines[k + 1].points = np.vstack([turn, lines[k + 1].points[1:]])
    return lines


def plan_field(subregions: Sequence[np.ndarray],
               obstacles: Sequence[CircleObstacle],
               cfg: SprayConfig) -> SprayPath:
    """Plan a whole field: each convex subregion swept, regions chained."""
    lines: List[ScanLine] = []
    next_id = 0
    for poly in subregions:
        sub = plan_convex(np.asarray(poly, dtype=float), obstacles, cfg, next_id)
        next_id += len(sub)
        lines.extend(sub)

    pts: List[np.ndarray] = []
    seg_line: List[int] = []
    for ln in lines:
        for p in ln.points:
            if pts and np.allclose(pts[-1], p, atol=1e-9):
                continue  # 转入弧首点与上一行末端重合，去重
            if pts:
                seg_line.append(ln.line_id)
            pts.append(p)
    all_pts = (np.asarray(pts, dtype=float) if pts
               else np.zeros((0, 2), dtype=float))
    seg_arr = np.asarray(seg_line, dtype=int)
    total = 0.0
    for i in range(len(all_pts) - 1):
        total += float(np.linalg.norm(all_pts[i + 1] - all_pts[i]))
    dur = total / cfg.speed if cfg.speed > 0 else 0.0
    return SprayPath(lines=lines, points=all_pts, seg_line=seg_arr,
                     total_length=total, duration=dur, config=cfg,
                     subregions=len(subregions))


def progress_state(path: SprayPath, position: Sequence[float]) -> dict:
    """Online progress for the ROS node: current line id and sprayed area %.

    节点的薄壳逻辑：取离当前位置最近的航点段得到行号，已完成行的条带
    面积累加即为已作业面积。纯 numpy，无 ROS 依赖。
    """
    p = np.asarray(position[:2], dtype=float)
    pts = path.points
    d2 = ((pts - p) ** 2).sum(axis=1)
    i = int(np.argmin(d2))
    i = min(i, len(path.seg_line) - 1)
    cur = int(path.seg_line[i])
    done = sum(ln.strip_area for ln in path.lines if ln.line_id < cur)
    total = sum(ln.strip_area for ln in path.lines)
    pct = 100.0 * done / total if total > 0 else 0.0
    return {"line_id": cur, "num_lines": len(path.lines),
            "area_percent": round(pct, 1),
            "position": [round(float(p[0]), 2), round(float(p[1]), 2)]}
