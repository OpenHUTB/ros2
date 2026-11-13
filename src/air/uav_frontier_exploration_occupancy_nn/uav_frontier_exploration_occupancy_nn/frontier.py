#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""前沿候选提取：把前沿格聚类成候选点并计算评分特征。"""

from __future__ import annotations

import math

import numpy as np

from .occupancy_map import OccupancyMap, FREE, UNKNOWN

MAX_DIST = 3.0


def cluster_frontiers(map_: OccupancyMap) -> list[list[tuple[int, int]]]:
    """八邻域连通聚类，返回前沿格簇列表。"""
    cells = set(map_.frontier_cells())
    clusters: list[list[tuple[int, int]]] = []
    while cells:
        seed = cells.pop()
        stack = [seed]
        cluster = [seed]
        while stack:
            c, r = stack.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    n = (c + dc, r + dr)
                    if n in cells:
                        cells.discard(n)
                        stack.append(n)
                        cluster.append(n)
        clusters.append(cluster)
    return clusters


def _cluster_features(map_: OccupancyMap, cluster: list[tuple[int, int]],
                     uav_x: float, uav_y: float, uav_yaw: float) -> dict:
    cs = [c for c, _ in cluster]
    rs = [r for _, r in cluster]
    cc, rr = int(round(np.mean(cs))), int(round(np.mean(rs)))
    wx, wy = map_.cell_to_world(cc, rr)

    dist = math.hypot(wx - uav_x, wy - uav_y)
    bearing = math.atan2(wy - uav_y, wx - uav_x) - uav_yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))

    unknown_nb = 0
    free_nb = 0
    total = 0
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            nc, nr = cc + dc, rr + dr
            if map_.in_bounds(nc, nr):
                total += 1
                if map_.grid[nr, nc] == UNKNOWN:
                    unknown_nb += 1
                elif map_.grid[nr, nc] == FREE:
                    free_nb += 1

    features = np.array([
        float(np.clip(dist / MAX_DIST, 0.0, 1.0)),
        float(np.clip(bearing / math.pi, -1.0, 1.0)),
        float(unknown_nb / 8.0),
        float(free_nb / total) if total else 0.0,
    ])
    return {"cell": (cc, rr), "x": wx, "y": wy, "features": features}


def extract_candidates(map_: OccupancyMap, uav_x: float, uav_y: float,
                       uav_yaw: float) -> list[dict]:
    """返回候选前沿点列表，每项含世界坐标与归一化特征。"""
    candidates: list[dict] = []
    for cluster in cluster_frontiers(map_):
        candidates.append(_cluster_features(map_, cluster, uav_x, uav_y, uav_yaw))
    return candidates
