#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图读写与栅格/世界坐标转换（纯 Python + NumPy，无 ROS 依赖）。"""

import math
from pathlib import Path

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


class GridMap(object):
    def __init__(self, grid, resolution, origin_x, origin_y):
        self.grid = grid                  # bool[row, col]，row=0 为 y 最小侧
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = grid.shape

    def world_to_cell(self, x, y, clamp=True):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if clamp:
            col = min(max(col, 0), self.width - 1)
            row = min(max(row, 0), self.height - 1)
        return row, col

    def cell_to_world(self, row, col):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def occupancy_values(self):
        """ROS OccupancyGrid 的 data：0 空闲、100 占用。"""
        return (self.grid.astype(np.int8) * 100).reshape(-1).tolist()


def _read_pgm(path):
    """读取 P2/P5 灰度 PGM，返回 (data[row=图片行, col], width, height)。"""
    raw = Path(path).read_bytes()
    tokens = []
    pos = 0
    n = len(raw)
    while len(tokens) < 4 and pos < n:
        while pos < n and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            while pos < n and raw[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < n and not raw[pos:pos + 1].isspace():
            pos += 1
        tokens.append(raw[start:pos])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    pos += 1  # 跳过 maxval 后的单个空白字符
    if magic == b'P5':
        data = np.frombuffer(raw[pos:pos + width * height], dtype=np.uint8)
    elif magic == b'P2':
        data = np.array([int(v) for v in raw[pos:].split()[:width * height]], dtype=np.uint16)
    else:
        raise ValueError("unsupported PGM magic: %r" % magic)
    return data.reshape((height, width)), width, height, maxval


def load_map(yaml_path):
    """读取 map YAML + PGM，返回 GridMap；图片上下翻转后与栅格坐标一致。"""
    yaml_path = Path(yaml_path)
    if yaml is not None:
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    else:  # pragma: no cover
        meta = {}
        for line in yaml_path.read_text(encoding="utf-8").splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                meta[key.strip()] = value.strip()
        meta["origin"] = [float(v) for v in meta["origin"].strip("[]").split(",")]
        meta["resolution"] = float(meta["resolution"])
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    image_path = yaml_path.parent / meta["image"]
    data, width, height, maxval = _read_pgm(image_path)
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))
    # 灰度值 -> 占用概率（与 map_server 的 negate=0 语义一致）
    occ = 1.0 - data.astype(np.float64) / float(maxval)
    grid = occ >= occupied_thresh
    grid[occ <= free_thresh] = False
    grid = grid[::-1]  # 图片第一行是地图最上方
    return GridMap(grid, resolution, float(origin[0]), float(origin[1]))


def grid_from_occupancy(data, width, height, occupied_threshold=50):
    """把 ROS OccupancyGrid 的 data（行优先、row=0 为 y 最小）转成 bool 栅格。"""
    values = np.asarray(data, dtype=np.int16)
    if values.size != width * height:
        raise ValueError("occupancy size mismatch")
    grid = values.reshape((height, width)) >= occupied_threshold
    grid[values.reshape((height, width)) < 0] = True   # 未知区域按占用处理（保守）
    return grid


def point_to_polyline_distance(point, points):
    """点到折线的最短距离（米），用于横向误差统计。"""
    px, py = point
    best = float("inf")
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg_len2 = dx * dx + dy * dy
        if seg_len2 <= 1e-12:
            dist = math.hypot(px - x0, py - y0)
        else:
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg_len2))
            dist = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        best = min(best, dist)
    return best
