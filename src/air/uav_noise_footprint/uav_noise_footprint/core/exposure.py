#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航线噪声暴露：沿航迹能量积分 → 敏感点 LAeq,T 与噪声足迹等值线。

评估流程：

1. **沿航迹采样**：把飞行剖面按 2 m 步长采样成位置序列（爬升/巡航/下降
   全程），每个采样点的驻留时间 Δt = ds / speed；
2. **逐点传播**：每个采样点对每个敏感点算 Lp（球面扩散 + 大气吸收）；
3. **能量合成**：等效连续 A 声级 LAeq,T = 10·log10( (1/T)·Σ 10^(Lp/10)·Δt )，
   多架飞机把能量直接相加；
4. **足迹栅格化**：在航迹包络外扩 150 m 的矩形上取 200×200 网格，逐格算
   同一 LAeq 场，再用 marching-squares 提取 45/55/65 dB 等值线段。

敏感点（学校/医院/居民区）带权重，供降噪优化加权目标使用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

from .acoustic import AcousticModel, DEFAULT
from .track import Flight

#: 敏感点类型 → LAeq 限值（dB(A)，昼间）
LIMITS = {"residential": 60.0, "school": 55.0, "hospital": 55.0}


@dataclass(frozen=True)
class SensitivePoint:
    """噪声敏感点：坐标（ENU 平面）+ 类型 + 权重。"""

    name: str
    x: float
    y: float
    kind: str = "residential"
    weight: float = 1.0

    @property
    def limit(self) -> float:
        return LIMITS.get(self.kind, 60.0)

    def to_dict(self) -> dict:
        return dict(name=self.name, x=self.x, y=self.y, kind=self.kind,
                    weight=self.weight)


# --------------------------------------------------------------------- 采样
def sample_track(flight: Flight, ds: float = 2.0) -> Tuple[np.ndarray, float]:
    """把剖面按 ``ds`` 米采样 → (positions (N,3), dt)。

    沿轨迹速度恒为 ``speed``，故驻留时间 Δt = ds / speed 对所有样本相同。
    """
    if ds <= 0.0:
        raise ValueError("采样步长必须为正")
    total = flight.path_length_3d()
    n = max(2, int(round(total / ds)))
    ts = np.linspace(flight.start_time, flight.end_time, n)
    pos = np.asarray([flight.position_at(float(t)) for t in ts])
    dt = (flight.end_time - flight.start_time) / (n - 1)
    return pos, dt


# --------------------------------------------------------------------- LAeq
def laeq_from_samples(pos: np.ndarray, dt: float, point: Tuple[float, float],
                      model: AcousticModel = DEFAULT) -> float:
    """一条采样轨迹对一个地面接收点 (x, y, 0) 的 LAeq（dB）。"""
    d = np.linalg.norm(pos - np.array([point[0], point[1], 0.0]), axis=1)
    d = np.maximum(d, 1e-3)
    lp = model.sound_power_level() - 20.0 * np.log10(d) - model.alpha * d
    energy = float(np.mean(10.0 ** (lp / 10.0)))
    return 10.0 * math.log10(energy)


def laeq_at_points(flights: Sequence[Flight],
                   points: Sequence[SensitivePoint],
                   ds: float = 2.0,
                   model: AcousticModel = DEFAULT,
                   repeat: int = 1,
                   t_obs: float | None = None) -> Dict[str, float]:
    """多机合成：每个敏感点的 LAeq,T。

    ``repeat`` 是观测期内同类航班的重复次数（交通密度），``t_obs`` 是观测
    期（s）：LAeq = 10·log10(repeat·Σ能量·Δt / t_obs)。缺省 repeat=1 且
    t_obs = 航迹最大结束时刻，即单事件、以航迹时长为观测期。
    """
    tracks = [(sample_track(f, ds), f) for f in flights]
    t_end = max((f.end_time for _s, f in tracks), default=0.0)
    t_obs = float(t_obs) if t_obs else t_end
    if t_obs <= 0.0:
        return {p.name: -math.inf for p in points}
    out: Dict[str, float] = {}
    for pt in points:
        total = 0.0
        for (pos, dt), f in tracks:
            seg = laeq_from_samples(pos, dt, (pt.x, pt.y), model)
            # 能量按各自驻留时间加权，再归一到观测期 t_obs
            total += (10.0 ** (seg / 10.0)) * dt * len(pos)
        out[pt.name] = (10.0 * math.log10(repeat * total / t_obs)
                        if total > 0 else -math.inf)
    return out


def objective(flights: Sequence[Flight],
              points: Sequence[SensitivePoint],
              ds: float = 2.0,
              model: AcousticModel = DEFAULT,
              repeat: int = 1,
              t_obs: float | None = None) -> float:
    """降噪优化的目标函数：Σ(敏感点 LAeq 的能量 × 权重)，再合成 dB。

    用能量域加权而不是 dB 域相加：5 dB 的改善在能量上是 3 倍差距，
    直接反映"最吵的敏感点主导总暴露"的物理事实。
    """
    laeqs = laeq_at_points(flights, points, ds, model, repeat, t_obs)
    total = sum(p.weight * 10.0 ** (laeqs[p.name] / 10.0) for p in points)
    return 10.0 * math.log10(total) if total > 0 else -math.inf


def laeq_from_log(times: np.ndarray, pos: np.ndarray,
                  point: Tuple[float, float],
                  model: AcousticModel = DEFAULT) -> float:
    """由**真实**位置日志（变步长，如 MuJoCo 20 Hz 记录）算一个点的 LAeq。"""
    d = np.linalg.norm(pos - np.array([point[0], point[1], 0.0]), axis=1)
    d = np.maximum(d, 1e-3)
    lp = model.sound_power_level() - 20.0 * np.log10(d) - model.alpha * d
    return 10.0 * math.log10(float(np.mean(10.0 ** (lp / 10.0))))


# ------------------------------------------------------------------ 在线快照
def current_levels(positions: Dict[str, Sequence[float]],
                   points: Sequence[SensitivePoint],
                   model: AcousticModel = DEFAULT) -> Dict[str, float]:
    """给定各机当前位置（在线快照），算每个敏感点的瞬时合成 Lp（dB）。

    这是 ROS 节点告警判定用的同一函数（core 无 ROS 依赖，可单测）。
    """
    out: Dict[str, float] = {}
    for pt in points:
        levels = []
        for pos in positions.values():
            d = math.dist((pos[0], pos[1], pos[2]), (pt.x, pt.y, 0.0))
            if d < 1e3:
                levels.append(model.lp_at(d))
        out[pt.name] = (10.0 * math.log10(sum(10.0 ** (l / 10.0) for l in levels))
                        if levels else -math.inf)
    return out


def exceedances(levels: Dict[str, float],
                points: Sequence[SensitivePoint]) -> List[dict]:
    """找出瞬时 Lp 超限的敏感点（供节点发布 JSON 告警）。"""
    by_name = {p.name: p for p in points}
    alerts = []
    for name, lp in sorted(levels.items()):
        pt = by_name.get(name)
        if pt is None or not math.isfinite(lp):
            continue
        if lp > pt.limit:
            alerts.append(dict(point=name, kind=pt.kind, lp=round(lp, 1),
                               limit=pt.limit,
                               exceed=round(lp - pt.limit, 1)))
    return alerts


# --------------------------------------------------------------------- 栅格
def grid_extent(flights: Sequence[Flight], pad: float = 150.0
                ) -> Tuple[float, float, float, float]:
    """航迹二维包络外扩 ``pad`` 米 → (xmin, xmax, ymin, ymax)。"""
    xs = [x for f in flights for x, _ in f.path]
    ys = [y for f in flights for _, y in f.path]
    return (min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad)


def grid_footprint(flights: Sequence[Flight], n: int = 200, ds: float = 2.0,
                   pad: float = 150.0,
                   model: AcousticModel = DEFAULT,
                   extent: Tuple[float, float, float, float] | None = None,
                   repeat: int = 1,
                   t_obs: float | None = None,
                   ) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    """200×200 栅格上的 LAeq 足迹场。

    返回 (values[n, n], extent)。values[j, i] 对应
    x = xmin + i·dx, y = ymin + j·dy（行对应 y、列对应 x）。
    分块计算避免 (N 样本 × M 格点) 的大矩阵一次性驻留内存。
    """
    if not flights:
        raise ValueError("至少需要一条航迹")
    extent = extent or grid_extent(flights, pad)
    xmin, xmax, ymin, ymax = extent
    xs = np.linspace(xmin, xmax, n)
    ys = np.linspace(ymin, ymax, n)
    gx, gy = np.meshgrid(xs, ys)                     # (n, n)
    flat_x, flat_y = gx.ravel(), gy.ravel()

    tracks = [(sample_track(f, ds), f) for f in flights]
    t_end = max((f.end_time for _s, f in tracks), default=0.0)
    t_obs = float(t_obs) if t_obs else t_end
    total_energy = np.zeros(flat_x.shape)

    chunk = max(1, int(2_000_000 / max(1, flat_x.size)))
    for (pos, dt), f in tracks:
        for k0 in range(0, len(pos), chunk):
            src = pos[k0:k0 + chunk]                              # (C, 3)
            d = np.linalg.norm(src[:, None, :2] -
                               np.stack([flat_x, flat_y], 1)[None, :, :],
                               axis=2)                            # 水平距 (C, M)
            d = np.hypot(d, src[:, 2:3])                          # 叠加高度 → 3D
            np.maximum(d, 1e-3, out=d)
            lp = model.sound_power_level() - 20.0 * np.log10(d) - model.alpha * d
            total_energy += (10.0 ** (lp / 10.0)).sum(axis=0) * dt
    values = 10.0 * np.log10(np.maximum(repeat * total_energy, 1e-30) / t_obs)
    return values.reshape(n, n), extent


# -------------------------------------------------------- marching squares
def contour_segments(grid: np.ndarray, level: float,
                     extent: Tuple[float, float, float, float]
                     ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """marching squares 提取等值线段（纯 numpy，无 matplotlib）。

    返回线段列表，每段是 ((x1,y1),(x2,y2))，世界坐标由 ``extent`` 映射。
    二义性用例（5/10）按简单连通处理，不影响足迹包络的量级。
    """
    g = np.asarray(grid, dtype=float)
    ny, nx = g.shape
    xmin, xmax, ymin, ymax = extent
    dx = (xmax - xmin) / (nx - 1)
    dy = (ymax - ymin) / (ny - 1)

    # 16 种角点符号组合 → 相交边对。边编号：0=上 1=右 2=下 3=左。
    TABLE = {
        0b0000: [], 0b1111: [],
        0b0001: [(2, 3)], 0b1110: [(2, 3)],
        0b0010: [(1, 2)], 0b1101: [(1, 2)],
        0b0011: [(1, 3)], 0b1100: [(1, 3)],
        0b0100: [(0, 1)], 0b1011: [(0, 1)],
        0b0101: [(0, 3)], 0b1010: [(0, 3)],
        0b0110: [(0, 2)], 0b1001: [(0, 2)],
        0b0111: [(1, 3)], 0b1000: [(1, 3)],
    }

    def interp(p, q, vp, vq):
        t = (level - vp) / (vq - vp)
        return p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t

    segs: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
    for j in range(ny - 1):
        row0, row1 = g[j], g[j + 1]
        y0, y1 = ymin + j * dy, ymin + (j + 1) * dy
        for i in range(nx - 1):
            tl, tr, br, bl = row0[i], row0[i + 1], row1[i + 1], row1[i]
            idx = ((tl > level) << 3) | ((tr > level) << 2) | \
                  ((br > level) << 1) | (bl > level)
            pair = TABLE[idx]
            if not pair:
                continue
            x0, x1 = xmin + i * dx, xmin + (i + 1) * dx
            corners = {"tl": (tl, (x0, y0)), "tr": (tr, (x1, y0)),
                       "br": (br, (x1, y1)), "bl": (bl, (x0, y1))}
            edge_pt = {
                0: lambda: interp(corners["tl"][1], corners["tr"][1], tl, tr),
                1: lambda: interp(corners["tr"][1], corners["br"][1], tr, br),
                2: lambda: interp(corners["bl"][1], corners["br"][1], bl, br),
                3: lambda: interp(corners["tl"][1], corners["bl"][1], tl, bl),
            }
            for e1, e2 in pair:
                segs.append((edge_pt[e1](), edge_pt[e2]()))
    return segs


def contour_summary(grid: np.ndarray, extent, levels: Iterable[float]) -> List[dict]:
    """各等值级的摘要：线段数、总长度、≥level 的栅格面积（m²）。"""
    xmin, xmax, ymin, ymax = extent
    n = grid.shape[0]
    cell_area = ((xmax - xmin) / (n - 1)) * ((ymax - ymin) / (n - 1))
    out = []
    for lv in levels:
        segs = contour_segments(grid, lv, extent)
        length = sum(math.dist(a, b) for a, b in segs)
        area = float((grid >= lv).sum()) * cell_area
        out.append(dict(level=float(lv), segments=len(segs),
                        length_m=round(length, 1),
                        area_m2=round(area, 1)))
    return out
