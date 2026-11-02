# -*- coding: utf-8 -*-
"""占据栅格地图：加载 PGM/YAML、栅格射线投射、似然场。

三件事
------
1. **加载**：直接读 `maps/mcl_room.pgm` + `.yaml`，不依赖 `map_server`。
   这样整个模块不需要 `ros-noetic-map-server` 也能跑（虚拟机里可能没装）。
2. **射线投射**：给定位姿与一组角度，算出每束激光的期望距离。
   用于两个地方：单元测试里**合成激光数据**；反向验证地图与场景是否一致。
3. **似然场**：预先算出"每个格子到最近障碍物的距离"，观测模型只需查表，
   比逐束射线投射快两个数量级——这是 MCL 能在 100 Hz 下跑几百个粒子的关键。

栅格约定（与 ROS map_server 一致）
--------------------------------
* 图像第 0 行是地图**顶部**，即 y 最大处；世界坐标原点在地图左下角。
* 值 0 = 占据，254 = 空闲，205 = 未知。
* `yaml` 里的 `origin` 是地图左下角在世界系中的位姿。
"""

import math
import os

import numpy as np

OCCUPIED = 0
FREE = 254
UNKNOWN = 205


def load_pgm(path):
    """读 PGM（P5 二进制）→ 二维 uint8 数组。只处理本项目用到的格式。"""
    with open(path, "rb") as fh:
        blob = fh.read()

    # 解析头部：魔数、可能存在的注释行、宽高、灰度上限
    pos = 0
    tokens = []
    while len(tokens) < 4:
        # 跳过空白
        while pos < len(blob) and blob[pos:pos + 1].isspace():
            pos += 1
        # 跳过注释
        if blob[pos:pos + 1] == b"#":
            while pos < len(blob) and blob[pos:pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while pos < len(blob) and not blob[pos:pos + 1].isspace():
            pos += 1
        tokens.append(blob[start:pos])
    pos += 1  # 跳过灰度上限后的那一个换行

    magic, width, height, maxval = tokens
    if magic != b"P5":
        raise ValueError("只支持 P5 格式的 PGM，收到 %r" % magic)
    width, height, maxval = int(width), int(height), int(maxval)
    data = np.frombuffer(blob[pos:pos + width * height], dtype=np.uint8)
    if data.size != width * height:
        raise ValueError("PGM 数据长度与宽高不符")
    return data.reshape(height, width)


def load_yaml(path):
    """读 map 的 yaml（项目里只用 image/resolution/origin 三个字段）。"""
    info = {}
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            info[key.strip()] = value.strip()

    origin_text = info.get("origin", "[0, 0, 0]")
    numbers = origin_text.strip("[]").split(",")
    return dict(
        image=info.get("image", ""),
        resolution=float(info.get("resolution", 0.05)),
        origin=tuple(float(v) for v in numbers),
        negate=int(info.get("negate", 0)),
    )


class OccupancyMap(object):
    """占据栅格 + 似然场。

    参数
    ----
    grid : (H, W) uint8
        第 0 行对应地图顶部（y 最大）。
    resolution : float
        米/像素。
    origin : (x, y, yaw)
        地图左下角在世界系中的位姿。
    """

    def __init__(self, grid, resolution=0.05, origin=(0.0, 0.0, 0.0)):
        self.grid = np.asarray(grid, dtype=np.uint8)
        self.resolution = float(resolution)
        self.origin = tuple(float(v) for v in origin)
        self.height, self.width = self.grid.shape

        # 占据掩码：把"未知"也当作障碍，避免粒子跑到墙外
        self.occupied = (self.grid == OCCUPIED) | (self.grid == UNKNOWN)
        self._likelihood = None

    # ------------------------------------------------------------------ 加载
    @classmethod
    def from_files(cls, pgm_path, yaml_path=None):
        grid = load_pgm(pgm_path)
        if yaml_path is None:
            root, _ = os.path.splitext(pgm_path)
            yaml_path = root + ".yaml"
        info = load_yaml(yaml_path)
        return cls(grid, info["resolution"], info["origin"])

    # ------------------------------------------------------------ 坐标变换
    def world_to_grid(self, x, y):
        """世界坐标 → 浮点栅格坐标（列, 行）。行号相对图像顶部。"""
        ox, oy = self.origin[0], self.origin[1]
        col = (np.asarray(x, dtype=np.float64) - ox) / self.resolution
        row_from_bottom = (np.asarray(y, dtype=np.float64) - oy) / self.resolution
        row = (self.height - 1) - row_from_bottom
        return col, row

    def grid_to_world(self, col, row):
        ox, oy = self.origin[0], self.origin[1]
        x = ox + (np.asarray(col, dtype=np.float64) + 0.5) * self.resolution
        y = oy + ((self.height - 1) - np.asarray(row, dtype=np.float64) + 0.5) * self.resolution
        return x, y

    def is_occupied_grid(self, col, row):
        """按整数栅格索引查占据（越界视为占据，防止粒子飞出地图）。"""
        col = np.asarray(col, dtype=np.int32)
        row = np.asarray(row, dtype=np.int32)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        result = np.ones(col.shape, dtype=bool)
        result[inside] = self.occupied[row[inside], col[inside]]
        return result

    # -------------------------------------------------------------- 似然场
    def likelihood_field(self):
        """每格到最近占据格的距离（米）。缓存，首次调用时计算。

        用多源 BFS（八邻域）近似欧氏距离，误差不超过约 1 个像素（5 cm），
        对观测模型完全可以接受，而且只用 NumPy，不需要 scipy 的距离变换。
        """
        if self._likelihood is not None:
            return self._likelihood

        height, width = self.grid.shape
        INF = np.int32(1 << 20)
        dist_px = np.full((height, width), INF, dtype=np.int32)
        dist_px[self.occupied] = 0

        # 八邻域代价：直向 1、斜向 1（近似，足够做权重）
        offsets = [(-1, -1), (-1, 0), (-1, 1),
                   (0, -1), (0, 1),
                   (1, -1), (1, 0), (1, 1)]
        frontier = np.argwhere(self.occupied)
        level = 0
        while frontier.size:
            level += 1
            collected = []
            for dr, dc in offsets:
                rows = frontier[:, 0] + dr
                cols = frontier[:, 1] + dc
                inside = ((rows >= 0) & (rows < height) & (cols >= 0) & (cols < width))
                rows, cols = rows[inside], cols[inside]
                if rows.size == 0:
                    continue
                fresh = dist_px[rows, cols] == INF
                rows, cols = rows[fresh], cols[fresh]
                if rows.size == 0:
                    continue
                dist_px[rows, cols] = level
                collected.append(np.stack([rows, cols], axis=1))
            frontier = np.concatenate(collected, axis=0) if collected else np.empty((0, 2), dtype=np.int32)

        dist_px = np.minimum(dist_px, 1 << 19)
        self._likelihood = dist_px.astype(np.float64) * self.resolution
        return self._likelihood

    def likelihood_at_world(self, x, y):
        """世界坐标处的"到最近障碍物距离"（米），越界返回大值。"""
        field = self.likelihood_field()
        col, row = self.world_to_grid(x, y)
        col = np.round(col).astype(np.int32)
        row = np.round(row).astype(np.int32)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        out = np.full(np.shape(col), 1.0, dtype=np.float64)
        out[inside] = field[row[inside], col[inside]]
        return out

    # -------------------------------------------------------------- 射线投射
    def raycast(self, pose, angles, max_range=8.0):
        """从 pose 出发沿 angles（车体系）投射激光，返回每束的距离。

        用标准的 DDA 大步进（步长 0.5 像素），精度足够合成测试数据。
        """
        x0, y0, th0 = pose
        angles = np.asarray(angles, dtype=np.float64)
        world = th0 + angles
        cos_a, sin_a = np.cos(world), np.sin(world)

        step = self.resolution * 0.5
        n_steps = int(max_range / step) + 1
        distances = np.full(angles.shape, float(max_range), dtype=np.float64)

        hit = np.zeros(angles.shape, dtype=bool)
        for i in range(1, n_steps + 1):
            d = i * step
            px = x0 + cos_a * d
            py = y0 + sin_a * d
            col, row = self.world_to_grid(px, py)
            occ = self.is_occupied_grid(np.round(col), np.round(row))
            new_hit = occ & (~hit)
            if np.any(new_hit):
                distances[new_hit] = d
                hit |= new_hit
            if hit.all():
                break
        return distances

    # ------------------------------------------------------------------ 其它
    def free_ratio(self):
        return float(np.mean(self.grid == FREE))

    def __repr__(self):
        return ("OccupancyMap(%dx%d, %.3f m/px, origin=(%.2f, %.2f))"
                % (self.width, self.height, self.resolution,
                   self.origin[0], self.origin[1]))
