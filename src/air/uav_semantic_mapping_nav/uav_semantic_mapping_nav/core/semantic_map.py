#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语义栅格地图：占据概率与类别分布共用一个 2.5D 栅格。

每格维护三样东西：

* ``logodds``  占据对数几率（射线穿过减、命中加）
* ``votes``    各类别的票数（只有命中点投票，票数最多的即该格语义）
* ``hits``     命中次数（用来区分"没看过"和"看过是空的"）

融合方式是**分开估计**：占据由几何射线决定，语义由分类结果投票决定。
把两者耦合在一起（例如"建筑类像素才标占据"）会让分割错误直接变成
地图错误，而分开之后语义错了只影响代价，不影响避障。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .labels import BLOCKED, N_CLASS, TRAVERSAL_COST

__all__ = ["SemMapConfig", "SemanticGrid", "CameraModel"]


@dataclass
class SemMapConfig:
    size: float = 60.0          # 地图边长（米），以原点为中心
    res: float = 0.20           # 分辨率
    l_occ: float = 0.85
    l_free: float = -0.40
    l_clamp: float = 4.0
    stride: int = 3             # 深度图抽样步长（每 stride 个像素取一个）
    max_range: float = 18.0
    min_height: float = 0.35    # 高于此高度视为障碍，低于视为地面
    ray_steps: int = 24         # 每条射线的空域采样点数


@dataclass
class CameraModel:
    """针孔相机内参。``fovy_deg`` 与底座 XML 里的 ``fovy`` 对应。"""
    width: int = 160
    height: int = 128
    fovy_deg: float = 60.0

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.fovy_deg) / 2.0)

    @property
    def fy(self) -> float:
        return (self.height / 2.0) / math.tan(math.radians(self.fovy_deg) / 2.0)

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0


class SemanticGrid:
    def __init__(self, cfg: Optional[SemMapConfig] = None,
                 n_class: int = N_CLASS):
        self.cfg = cfg or SemMapConfig()
        self.n_class = int(n_class)
        self.n = int(round(self.cfg.size / self.cfg.res))
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.votes = np.zeros((self.n_class, self.n, self.n), np.int32)
        self.hits = np.zeros((self.n, self.n), np.int32)
        self.free = np.zeros((self.n, self.n), np.int32)

    # ------------------------------------------------------------ 坐标
    def cell_of(self, xy: np.ndarray) -> np.ndarray:
        idx = np.floor((np.asarray(xy, float) + self.cfg.size / 2.0)
                       / self.cfg.res).astype(np.int64)
        return idx

    def _flat(self, idx: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ok = np.all((idx >= 0) & (idx < self.n), axis=-1)
        i = idx[ok]
        return i[:, 1] * self.n + i[:, 0], ok

    def centre(self, r: int, c: int) -> np.ndarray:
        return np.array([(c + 0.5) * self.cfg.res - self.cfg.size / 2.0,
                         (r + 0.5) * self.cfg.res - self.cfg.size / 2.0])

    # ------------------------------------------------------------ 更新
    def integrate(self, pose: np.ndarray, depth: np.ndarray, seg: Optional[np.ndarray],
                  cam: CameraModel) -> int:
        """把一帧 (深度, 语义) 融进地图。``pose`` 为 (x, y, yaw)。"""
        cfg = self.cfg
        h, w = depth.shape
        v, u = np.mgrid[0:h:cfg.stride, 0:w:cfg.stride]
        z = np.asarray(depth, float)[::cfg.stride, ::cfg.stride]
        u = u.astype(float)
        v = v.astype(float)
        ok = (z > cfg.min_height * 0.5) & (z < cfg.max_range) & np.isfinite(z)
        if not np.any(ok):
            return 0
        z, u, v = z[ok], u[ok], v[ok]

        # 针孔反投影：相机系 (x 右, y 下, z 前) → 机体系 (x 前, y 左, z 上)
        xb = z
        yb = -(u - cam.cx) / cam.fx * z
        zb = -(v - cam.cy) / cam.fy * z

        c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
        ox, oy = float(pose[0]), float(pose[1])
        wx = ox + c * xb - s * yb
        wy = oy + s * xb + c * yb
        wh = zb                                    # 相对机体的高度偏移

        idx = self.cell_of(np.column_stack([wx, wy]))
        flat, good = self._flat(idx)
        if not np.any(good):
            return 0
        wx, wy, wh = wx[good], wy[good], wh[good]

        # 命中：高于阈值算障碍，否则只是地面
        above = wh > cfg.min_height
        occ_flat = flat[above]
        if len(occ_flat):
            cnt = np.bincount(occ_flat, minlength=self.n * self.n)
            self.logodds += (cnt.reshape(self.n, self.n) * cfg.l_occ).astype(np.float32)
            self.hits += cnt.reshape(self.n, self.n).astype(np.int32)
        free_flat = flat[~above]
        if len(free_flat):
            cf = np.bincount(free_flat, minlength=self.n * self.n)
            self.free += cf.reshape(self.n, self.n).astype(np.int32)

        # 语义投票：只对命中点投，地面点同样投票
        if seg is not None:
            lab = np.asarray(seg)[::cfg.stride, ::cfg.stride][ok][good].astype(np.int64)
            lab = np.clip(lab, 0, self.n_class - 1)
            sel = flat * self.n_class + lab
            vc = np.bincount(sel, minlength=self.n * self.n * self.n_class)
            self.votes += vc.reshape(self.n, self.n, self.n_class).transpose(2, 0, 1) \
                .astype(np.int32)

        # 空域：从机体到命中点沿线标记"走过"
        self._carve(ox, oy, wx, wy)

        np.clip(self.logodds, -cfg.l_clamp, cfg.l_clamp, out=self.logodds)
        return int(len(flat))

    def _carve(self, ox: float, oy: float, wx: np.ndarray, wy: np.ndarray) -> None:
        """把机体到命中点之间标成空域。

        采样点**不含终点**：终点是命中格，把它的空域票也算进去会把刚加的
        占据票抵消掉（同一堵墙被几百条射线打中，实测 log-odds 直接被压成负值，
        概率图上整面墙消失）。
        """
        cfg = self.cfg
        n_s = max(cfg.ray_steps, 2)
        t = np.linspace(0.0, 1.0, n_s, endpoint=False)[None, :]
        px = ox + t * (wx[:, None] - ox)
        py = oy + t * (wy[:, None] - oy)
        idx = self.cell_of(np.stack([px.ravel(), py.ravel()], axis=1))
        flat, good = self._flat(idx)
        if not np.any(good):
            return
        cnt = np.bincount(flat, minlength=self.n * self.n)
        self.logodds += (cnt.reshape(self.n, self.n) * cfg.l_free).astype(np.float32)

    # ------------------------------------------------------------ 读出
    def probability(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.logodds))

    def observed(self, min_hits: int = 1) -> np.ndarray:
        return (self.hits + self.free) >= min_hits

    def semantic(self) -> np.ndarray:
        """每格票数最多的类别（未投票的格为 0=背景）。"""
        return np.argmax(self.votes, axis=0).astype(np.int32)

    def semantic_conf(self) -> np.ndarray:
        """票数占比，作为语义置信度。"""
        tot = self.votes.sum(axis=0).astype(np.float32)
        best = self.votes.max(axis=0).astype(np.float32)
        return np.where(tot > 0, best / np.maximum(tot, 1.0), 0.0)

    def cost_map(self, unknown_cost: float = 2.0) -> np.ndarray:
        """语义代价图。

        **只有几何占据给 ``inf``**；语义给出的高代价是有限的（见
        ``labels.TRAVERSAL_COST``）。语义置信度低时把代价往 ``unknown_cost``
        拉，避免"猜错的语义"把代价压得过低。
        """
        cls = self.semantic()
        base = TRAVERSAL_COST[cls].copy()
        conf = self.semantic_conf()
        obs = self.observed()
        # 未观测格给中性代价，不鼓励也不禁止
        base[~obs] = unknown_cost
        # 置信度低 → 向中性代价插值
        k = np.clip(conf, 0.0, 1.0)
        low = (obs & (k < 0.6))
        base[low] = 0.5 * base[low] + 0.5 * unknown_cost
        # 已知占据 → 不可通行
        base[self.probability() > 0.7] = np.inf
        return base

    def stats(self) -> Dict[str, float]:
        p = self.probability()
        return {"n": float(self.n), "cells": float(self.n * self.n),
                "occupied": float((p > 0.7).sum()),
                "free": float((p < 0.3).sum()),
                "observed": float(self.observed().sum()),
                "n_class_used": float(len(np.unique(self.semantic()[self.observed()])))
                if self.observed().any() else 0.0}
