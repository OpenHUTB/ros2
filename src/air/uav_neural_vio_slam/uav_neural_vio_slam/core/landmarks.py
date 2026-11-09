#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光雷达 → 立柱路标观测（测距 + 方位）。

判据是"**细**"：一根立柱在方位向只占极少的几个波束，而墙面、地面、
大方柱都会在方位向连成一片。再加上跨俯仰层的距离一致性（同一根柱子被
±15° 两层打到时距离几乎相同，地面不是），就能把柱子从环境里挑出来。
这套判据不需要知道无人机的高度和姿态，因此在线估计器可以原样使用。

观测到的是**柱面**，地图里存的是**轴线**，两者差一个半径。基准世界里立柱
半径都在 0.08~0.12 m，按标称 0.10 m 补偿，残余 ±0.02 m，低于测距噪声。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["DetectorConfig", "Observation", "LandmarkDetector",
           "observations_to_array", "to_world_horizontal"]


@dataclass
class DetectorConfig:
    n_azimuth: int = 180
    elevations_deg: Sequence[float] = (-15.0, 0.0, 15.0)
    max_range: float = 16.0
    min_range: float = 0.8
    r_nominal: float = 0.10          # 立柱标称半径
    max_bins: int = 8                # 一根柱子最多占几个方位波束（"细"的判据）
    ring_tol: float = 0.35           # 跨俯仰层 / 跨方位的距离一致性容差
    min_rings: int = 2               # 至少几层同时命中
    max_obs: int = 12


@dataclass
class Observation:
    rng: float                       # 到轴线的**水平**距离（已补偿标称半径）
    bear: float                      # 机体系水平方位角（前为 0，左为正）
    rings: int = 1
    #: 命中点在机体系（雷达安装坐标系）里的三维坐标，已补偿标称半径。
    #: 无人机有俯仰/横滚时，射线方向被姿态"掰"过，机体系方位角最多能偏 7°
    #: （±15° 那两层最明显），5 m 处就是 0.6 m 误差 —— 只有拿这个三维点做一次
    #: 完整的姿态旋转，才能得到二维 SLAM 观测模型需要的水平方位。
    point: np.ndarray = None


class LandmarkDetector:
    """把一帧激光射线距离变成若干 (距离, 方位) 观测。"""

    def __init__(self, cfg: Optional[DetectorConfig] = None):
        self.cfg = cfg or DetectorConfig()
        self.elev = np.deg2rad(np.asarray(self.cfg.elevations_deg, float))
        self.n_el = len(self.elev)
        self.n_az = int(self.cfg.n_azimuth)
        az = -np.pi + 2 * np.pi * np.arange(self.n_az) / self.n_az
        self.az = az
        ce, se = np.cos(self.elev), np.sin(self.elev)
        # 机体系：x 前、y 左、z 上；方位角自 x 轴向 y 为正
        self.dirs = np.stack([
            np.outer(ce, np.cos(az)).ravel(),
            np.outer(ce, np.sin(az)).ravel(),
            np.outer(se, np.ones(self.n_az)).ravel(),
        ], axis=1)                                   # (n_el·n_az, 3)

    # ------------------------------------------------------------ 检测
    def detect(self, ranges: np.ndarray) -> List[Observation]:
        """``ranges`` 为 (n_el·n_az,) 的**三维**射线距离，0/超量程表示未命中。

        对外报出的 ``Observation.rng`` 是**水平距离**（乘过 cos 俯仰角），
        与二维 SLAM 的观测模型 ``r = hypot(mx-x, my-y)`` 一致。

        -15° 层的三维距离比水平距离大 sec(15°)−1 = 3.5%，8 m 处 0.28 m；
        实测路标 RMSE 0.66 m 全部来自这一项。
        """
        cfg = self.cfg
        r3 = np.asarray(ranges, float).reshape(self.n_el, self.n_az)
        ok = (r3 > cfg.min_range) & (r3 < cfg.max_range)
        # 水平投影距离：只有它才进得了二维观测模型
        r = r3 * np.cos(self.elev)[:, None]
        # 每个方位上先把有效距离按容差聚簇，只保留"至少 min_rings 层落在同一距离"
        # 的簇 —— 这一条才是"立柱"的判据。只检查"有没有命中"是不够的：
        # 地面回波在 -15° 层命中、远处的树在 0° 层命中，两者距离差好几米，
        # 混在一起会让测距中位数整个歪掉（实测平均误差近 1 m）。
        clus: List[List[Tuple[float, int]]] = []
        for a in range(self.n_az):
            clus.append(self._ring_clusters(r[ok[:, a], a]))

        used = np.zeros(self.n_az, dtype=bool)
        out: List[Observation] = []
        for a in range(self.n_az):
            if used[a] or not clus[a]:
                continue
            # 同一方位上可能有近处柱子 + 远处柱子，逐簇尝试
            for (r0, _n) in clus[a]:
                if used[a]:
                    break
                # 整组一起长：只要邻位在这个距离上还有一致的回波就并进来。
                # 不能逐格判"孤立"——近处的柱子会横跨好多个方位波束，
                # 逐格判会把中间那些格子全部丢掉，只剩两侧边缘，
                # 于是一根柱子被拆成两条观测（实测 1 m 处拆成 1.01/+9° 与 1.02/+1°）
                grp = self._grow(clus, a, r0, used)
                if grp is None or len(grp) > cfg.max_bins:
                    continue                    # 方位向连成一片 → 墙面，不是柱子
                if not self._isolated(clus, grp, r0, used):
                    continue
                for b in grp:
                    used[b] = True
                vals, rings, pts = [], 0, []
                for b in grp:
                    c = min(clus[b], key=lambda cc: abs(cc[0] - r0))
                    vals.append(c[0])
                    rings = max(rings, c[1])
                    for e in np.nonzero(ok[:, b])[0]:
                        if abs(r[e, b] - c[0] / np.cos(self.elev[e])) <= cfg.ring_tol:
                            pts.append(self.dirs[e * self.n_az + b] * r3[e, b])
                ang = float(np.angle(np.mean(np.exp(1j * self.az[grp]))))
                rr = float(np.median(vals)) + cfg.r_nominal
                pt = None
                if pts:
                    p = np.mean(np.asarray(pts), axis=0)
                    n = math.hypot(p[0], p[1])
                    if n > 1e-6:                     # 补上标称半径
                        p = p * (1.0 + cfg.r_nominal / n)
                    pt = p
                out.append(Observation(rng=rr, bear=ang, rings=rings, point=pt))
                if len(out) >= cfg.max_obs:
                    break
        out.sort(key=lambda o: o.rng)
        return out

    def _grow(self, clus, a: int, r0: float, used: np.ndarray):
        """从方位 a 向两侧生长，返回**连续**的方位集合（可能为空）。"""
        grp = [a]
        for step in (1, -1):
            b = (a + step) % self.n_az
            while b != a and b not in grp and not used[b] \
                    and len(grp) <= self.cfg.max_bins:
                if not any(abs(c[0] - r0) <= self.cfg.ring_tol for c in clus[b]):
                    break
                grp.append(b)
                b = (b + step) % self.n_az
        return sorted(grp)

    def _isolated(self, clus, grp, r0: float, used: np.ndarray) -> bool:
        """组的两侧至少一侧在 r0 附近**断掉**：柱子是孤立的，墙面是连续的。"""
        cfg = self.cfg
        a0, a1 = grp[0], grp[-1]
        far = 0
        for b in ((a0 - 1) % self.n_az, (a1 + 1) % self.n_az):
            if b in grp or used[b]:
                far += 1
                continue
            if not any(abs(c[0] - r0) <= cfg.ring_tol for c in clus[b]):
                far += 1
        return far >= 1

    def _ring_clusters(self, vals: np.ndarray) -> List[Tuple[float, int]]:
        """把同一方位上的有效距离按容差聚簇，返回 [(中位数, 簇大小), …]，按距离升序。

        只保留大小 ≥ ``min_rings`` 的簇。
        """
        if len(vals) == 0:
            return []
        v = np.sort(np.asarray(vals, float))
        groups = [[v[0]]]
        for x in v[1:]:
            if x - groups[-1][-1] <= self.cfg.ring_tol:
                groups[-1].append(x)
            else:
                groups.append([x])
        out = [(float(np.median(g)), len(g)) for g in groups
               if len(g) >= self.cfg.min_rings]
        out.sort()
        return out

    # ------------------------------------------------------------ 几何
    def points(self, ranges: np.ndarray) -> np.ndarray:
        """射线距离 → 机体系点云 (M,3)（只保留命中点）。"""
        r = np.asarray(ranges, float).ravel()
        m = (r > self.cfg.min_range) & (r < self.cfg.max_range)
        return self.dirs[m] * r[m, None]

    def ranges_from_points(self, pts_body: np.ndarray) -> np.ndarray:
        """机体系点云 → 整幅**三维**射线距离图（未命中为 0）。

        对外报出的仍是三维距离，与 ``detect`` 的入参约定一致；水平投影在
        ``detect`` 内部完成。

        激光点必然落在某条射线上，所以只用方位角/俯仰角查表即可精确还原，
        不需要最近邻搜索。这条路径让 ROS 节点可以直接吃 ``/lidar/points``，
        与仿真侧共用同一份检测器。
        """
        r = np.zeros(self.rays())
        p = np.asarray(pts_body, float).reshape(-1, 3)
        if len(p) == 0:
            return r
        d = np.linalg.norm(p, axis=1)
        m = d > 1e-6
        p, d = p[m], d[m]
        az = np.arctan2(p[:, 1], p[:, 0])
        el = np.arcsin(np.clip(p[:, 2] / d, -1.0, 1.0))
        ai = np.rint((az + np.pi) / (2 * np.pi) * self.n_az).astype(int) % self.n_az
        ei = np.argmin(np.abs(el[:, None] - self.elev[None, :]), axis=1)
        r[ei * self.n_az + ai] = d
        return r

    def rays(self) -> int:
        return self.n_el * self.n_az


def observations_to_array(obs: Sequence[Observation], max_obs: int = 12
                          ) -> Tuple[np.ndarray, np.ndarray, int]:
    n = min(len(obs), max_obs)
    r = np.zeros(max_obs)
    b = np.zeros(max_obs)
    for i in range(n):
        r[i] = obs[i].rng
        b[i] = obs[i].bear
    return r, b, n


def to_world_horizontal(obs: Sequence[Observation], R: np.ndarray, yaw: float,
                        max_range: float = 16.0) -> List[Observation]:
    """把机体观测转成"二维 SLAM 观测模型"要的那种 (水平距离, 全局方位 − 偏航)。

    ``R`` 为机体→世界的完整旋转矩阵（含俯仰与横滚）；只用偏航时，倾斜引起的
    方位偏差会留在观测里。
    """
    out: List[Observation] = []
    for o in obs:
        if o.point is None:
            out.append(o)
            continue
        p = np.asarray(R, float) @ np.asarray(o.point, float)
        r = float(math.hypot(p[0], p[1]))
        if r < 1e-3 or r > max_range:
            continue
        b = float(np.angle(np.exp(1j * (math.atan2(p[1], p[0]) - yaw))))
        out.append(Observation(rng=r, bear=b, rings=o.rings, point=o.point))
    return out
