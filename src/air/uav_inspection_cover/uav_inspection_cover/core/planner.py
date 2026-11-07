#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Route planning: nearest-neighbor tower order, 2-opt refinement, detours.

规划分层：先最近邻构造塔访问序，再 2-opt 只在总航程下降时接受改进；
塔内观察点按到达角排序串接；遇不可飞段在可飞图上用 Dijkstra 最短路
替代直飞段。总时长 = 航程 / 巡航速度 + 每观察点悬停时间。
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .geometry import (ObservePoint, Params, Rect, Tower, Waypoint,
                       corridor_viewpoints, leg_flyable, point_seg_dist,
                       polyline_length, seg_rect_dist)


@dataclass(frozen=True)
class Corridor:
    name: str
    title: str
    towers: Tuple[Tower, ...]
    start: Tuple[float, float]
    zones: Tuple[Rect, ...] = ()


@dataclass
class Route:
    """一条完整巡检航线（含起点起飞点）。"""

    name: str
    waypoints: List[Waypoint]
    viewpoints: List[ObservePoint] = field(default_factory=list)
    order: List[int] = field(default_factory=list)
    hover_at: set = field(default_factory=set)   # 需悬停拍摄的航点下标

    @property
    def distance_m(self) -> float:
        return polyline_length(self.waypoints)

    def duration_s(self, params: Params) -> float:
        return (self.distance_m / params.cruise_speed
                + params.hover_time * len(self.viewpoints))


# ------------------------------------------------------------------ 场景
def _grid_towers(n_per_row: int, x0: float, dx: float, y_row: float
                 ) -> List[Tower]:
    """一排塔：塔高按 26/32/38 m 循环，天然形成"每塔 2~3 个观察角度"混布。"""
    towers: List[Tower] = []
    for i in range(n_per_row):
        towers.append(Tower("T%02d" % (len(towers) + 1),
                            x0 + i * dx, y_row, 26.0 + (i % 3) * 6.0))
    return towers


def _double_row(n_per_row: int, x0: float, dx: float) -> List[Tower]:
    """双排网格走廊（case_grid 前缀的由来）：NN 会来回横跳，2-opt 有得改。"""
    a = _grid_towers(n_per_row, x0, dx, 0.0)
    for j, t in enumerate(_grid_towers(n_per_row, x0 + 0.5 * dx, dx, 220.0)):
        a.append(Tower("T%02d" % (n_per_row + j + 1), t.x, t.y, t.height))
    return a


def build_cases() -> Dict[str, Corridor]:
    """三个走廊场景：短 / 长 / 带河流禁飞区（需绕行）。起点=运维站位置。"""
    short = Corridor(
        name="case_grid_short", title="6-tower short corridor",
        towers=tuple(_double_row(3, 300.0, 170.0)), start=(300.0, 220.0))

    long_c = Corridor(
        name="case_grid_long", title="12-tower long corridor",
        towers=tuple(_double_row(6, 300.0, 170.0)), start=(0.0, 220.0))

    river_towers = _double_row(5, 300.0, 170.0)
    # 河流禁飞区横在双排之间（y 80~140），只留东端 x>700 绕行口
    river = Corridor(
        name="case_grid_river", title="10-tower corridor with river no-fly zone",
        towers=tuple(river_towers), start=(-100.0, 0.0),
        zones=((280.0, 80.0, 700.0, 140.0),))
    return {"case_grid_short": short, "case_grid_long": long_c,
            "case_grid_river": river}


# ------------------------------------------------------------------ 距离矩阵
def tower_cost(corridor: Corridor, i: int, j: int) -> float:
    """塔间代价：塔心二维距离（2-opt 评估用，与观察点串接解耦）。"""
    a, b = corridor.towers[i], corridor.towers[j]
    return math.hypot(a.x - b.x, a.y - b.y)


def order_cost(corridor: Corridor, order: Sequence[int]) -> float:
    """起点 + 塔序的塔心总航程。"""
    pts = [corridor.start] + [(corridor.towers[i].x, corridor.towers[i].y)
                              for i in order]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts[:-1], pts[1:]))


def nearest_neighbor(corridor: Corridor) -> List[int]:
    """最近邻构造塔访问序：从起点出发每步取最近未访问塔。"""
    n = len(corridor.towers)
    cur = corridor.start
    left = set(range(n))
    order: List[int] = []
    while left:
        nxt = min(left, key=lambda i: math.hypot(
            corridor.towers[i].x - cur[0], corridor.towers[i].y - cur[1]))
        order.append(nxt)
        left.discard(nxt)
        cur = (corridor.towers[nxt].x, corridor.towers[nxt].y)
    return order


def two_opt(order: List[int], cost_fn) -> List[int]:
    """2-opt 改进：只有总航程严格下降才接受，循环到无改进为止。"""
    order = list(order)
    best = cost_fn(order)
    improved = True
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                c = cost_fn(cand)
                if c < best - 1e-6:
                    order, best, improved = cand, c, True
    return order


# ------------------------------------------------------------------ 串接
def sort_viewpoints(vps: List[ObservePoint], tower: Tower,
                    incoming: Waypoint) -> List[ObservePoint]:
    """塔内观察点按到达角排序，减少塔位附近的往返折返。"""
    a_in = math.atan2(tower.y - incoming[1], tower.x - incoming[0])

    def key(vp: ObservePoint) -> float:
        d = math.atan2(vp.y - tower.y, vp.x - tower.x) - a_in
        return math.atan2(math.sin(d), math.cos(d))

    return sorted(vps, key=key)


# ------------------------------------------------------------------ 绕行
def build_flyable_graph(corridor: Corridor, params: Params
                        ) -> Tuple[List[Waypoint], Dict[int, List[int]]]:
    """在走廊包围盒上铺方格网，保留离塔/离禁飞区都安全的双向边。"""
    xs = [t.x for t in corridor.towers] + [corridor.start[0]]
    ys = [t.y for t in corridor.towers] + [corridor.start[1]]
    for z in corridor.zones:
        xs += [z[0], z[2]]
        ys += [z[1], z[3]]
    pad = 3.0 * params.grid_step_m
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    step = params.grid_step_m

    nodes: List[Waypoint] = []
    index: Dict[Tuple[int, int], int] = {}
    ny = int(math.ceil((y1 - y0) / step))
    nx = int(math.ceil((x1 - x0) / step))
    for iy in range(ny + 1):
        for ix in range(nx + 1):
            p = (x0 + ix * step, y0 + iy * step, 0.0)
            if all(math.hypot(t.x - p[0], t.y - p[1]) >= params.clearance_m
                   for t in corridor.towers) and all(
                   _point_zone(p, z) > params.zone_margin_m
                   for z in corridor.zones):
                index[(ix, iy)] = len(nodes)
                nodes.append(p)

    adj: Dict[int, List[int]] = {i: [] for i in range(len(nodes))}
    for (ix, iy), i in index.items():
        for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
            j = index.get((ix + dx, iy + dy))
            if j is None:
                continue
            a, b = nodes[i], nodes[j]
            if (all(point_seg_dist(t.x, t.y, a[0], a[1], b[0], b[1])
                    >= params.clearance_m for t in corridor.towers)
                    and all(seg_rect_dist(a, b, z) >= params.zone_margin_m
                            for z in corridor.zones)):
                adj[i].append(j)
                adj[j].append(i)
    return nodes, adj


def _point_zone(p: Waypoint, z: Rect) -> float:
    dx = max(z[0] - p[0], 0.0, p[0] - z[2])
    dy = max(z[1] - p[1], 0.0, p[1] - z[3])
    return math.hypot(dx, dy)


def _nearest_node(nodes: List[Waypoint], adj: Dict[int, List[int]],
                  p: Waypoint) -> int:
    cand = [i for i in adj if adj[i]]
    return min(cand, key=lambda i: math.hypot(nodes[i][0] - p[0],
                                              nodes[i][1] - p[1]))


def _connect(nodes: List[Waypoint], adj: Dict[int, List[int]], p: Waypoint,
             corridor: Corridor, params: Params,
             k: int = 12) -> Optional[int]:
    """从 p 找一个"连线本身可飞"的图节点；找不到返回 None。"""
    cand = sorted((i for i in adj if adj[i]),
                  key=lambda i: math.hypot(nodes[i][0] - p[0],
                                           nodes[i][1] - p[1]))[:k]
    for i in cand:
        if leg_flyable(p, nodes[i], corridor.towers, corridor.zones, params):
            return i
    return None


def dijkstra_path(nodes: List[Waypoint], adj: Dict[int, List[int]],
                  s: int, g: int) -> List[int]:
    """Dijkstra 最短路（可飞图上直线段的替代路径）。"""
    dist = {s: 0.0}
    prev: Dict[int, int] = {}
    pq = [(0.0, s)]
    done = set()
    while pq:
        d, u = heapq.heappop(pq)
        if u in done:
            continue
        done.add(u)
        if u == g:
            break
        for v in adj[u]:
            w = math.hypot(nodes[v][0] - nodes[u][0], nodes[v][1] - nodes[u][1])
            if d + w < dist.get(v, math.inf) - 1e-9:
                dist[v] = d + w
                prev[v] = u
                heapq.heappush(pq, (d + w, v))
    if g not in done and g != s:
        return []
    path, cur = [g], g
    while cur != s:
        cur = prev[cur]
        path.append(cur)
    return path[::-1]


def _simplify_points(pts: List[Waypoint], corridor: Corridor,
                     params: Params) -> List[Waypoint]:
    """整段拼接点的视线简化：贪心连最远可直飞点，消除微段与锯齿。"""
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not leg_flyable(pts[i], pts[j], corridor.towers,
                                            corridor.zones, params):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def _replace_leg(a: Waypoint, b: Waypoint, corridor: Corridor,
                 params: Params, nodes: List[Waypoint],
                 adj: Dict[int, List[int]]) -> List[Waypoint]:
    """直飞段不可飞时，用可飞图 Dijkstra 路径替代（保持两端观察点不变）。"""
    s = _connect(nodes, adj, a, corridor, params)
    g = _connect(nodes, adj, b, corridor, params)
    if s is None or g is None:
        return [a, b]
    idx = dijkstra_path(nodes, adj, s, g)
    if len(idx) < 2:
        return [a, b]
    # 先按沿线里程给节点插值 z，再整段视线简化
    total = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    pts: List[Waypoint] = [a]
    done = 0.0
    for i in idx:                      # 含首尾连接节点，a→s / g→b 已由 _connect 保证可飞
        n = nodes[i]
        done += math.hypot(n[0] - pts[-1][0], n[1] - pts[-1][1])
        u = min(1.0, done / total)
        pts.append((n[0], n[1], a[2] + (b[2] - a[2]) * u))
    pts.append(b)
    return _simplify_points(pts, corridor, params)


# ------------------------------------------------------------------ 主入口
# 可飞图按 (corridor, params) 缓存：2-opt 反复评估航线时不重建
_GRAPH_CACHE: Dict[Tuple[Corridor, Params], tuple] = {}


def make_route(corridor: Corridor, params: Params,
               order: Optional[Sequence[int]] = None) -> Route:
    """生成完整航线：起点 → 逐塔观察点（按到达角串接）→ 不可飞段绕行。"""
    vps_by_tower: Dict[int, List[ObservePoint]] = {}
    for vp in corridor_viewpoints(corridor.towers, params):
        vps_by_tower.setdefault(vp.tower, []).append(vp)

    if order is None:
        order = nearest_neighbor(corridor)
    order = list(order)

    waypoints: List[Waypoint] = [(corridor.start[0], corridor.start[1],
                                  params.min_height_m)]
    viewpoints: List[ObservePoint] = []
    hover_at: set = set()
    nodes = adj = None
    for ti in order:
        tw = corridor.towers[ti]
        seq = sort_viewpoints(vps_by_tower[ti], tw, waypoints[-1])
        for vp in seq:
            a, b = waypoints[-1], vp.xyz()
            if not leg_flyable(a, b, corridor.towers, corridor.zones, params):
                if nodes is None:
                    key = (corridor, params)
                    if key not in _GRAPH_CACHE:
                        _GRAPH_CACHE[key] = build_flyable_graph(corridor,
                                                                params)
                    nodes, adj = _GRAPH_CACHE[key]
                # out[0] == a，跳过避免重复航点
                waypoints.extend(_replace_leg(a, b, corridor, params,
                                              nodes, adj)[1:])
            else:
                waypoints.append(b)
            hover_at.add(len(waypoints) - 1)
            viewpoints.append(vp)

    return Route(name=corridor.name, waypoints=waypoints,
                 viewpoints=viewpoints, order=order, hover_at=hover_at)


def route_cost(corridor: Corridor, params: Params) -> "callable":
    """2-opt 的代价函数：整条航线的实际航程（含串接与绕行）。"""
    def cost(order: Sequence[int]) -> float:
        return make_route(corridor, params, order).distance_m
    return cost


# ------------------------------------------------------------------ 在线进度
def progress_snapshot(waypoints: Sequence[Waypoint], pos: Sequence[float],
                      arrive_radius_m: float, hint: int = 0) -> dict:
    """当前目标观察点与进度百分比（ROS 节点薄的判定核心，纯函数）。"""
    i = hint
    n = len(waypoints)
    while i < n and math.dist(waypoints[i], pos[:3]) <= arrive_radius_m:
        i += 1
    target = waypoints[min(i, n - 1)]
    return dict(target_index=min(i, n - 1),
                target=[round(float(v), 2) for v in target],
                progress=round(100.0 * i / max(1, n), 1),
                done=i >= n)
