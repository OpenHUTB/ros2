#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""几何建图 vs 语义建图的对比实验。

同一份扫描数据、同一套 A*，只换代价图：

    geometric   先占据/空闲二值化 → 只避开占据格
    semantic    按类别给通行代价（地面 1、门 1.2、植被 3、建筑/障碍 20）
    oracle      语义来自仿真真值分割（上界，用来说明网络精度的影响）

只有几何占据给 ``inf``；语义给出的高代价是有限的，避免一次误分类堵死通路。
"语义图好不好"因此不体现在成功率上，而体现在**路径平均语义代价**上。

"必须穿过可通行区域"的场景：起点与终点被植被/障碍带隔开，几何地图看不出
哪边能过，语义地图可以。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .labels import N_CLASS, TRAVERSAL_COST
from .metrics import (collision_count, path_uncertainty, travelled_through)
from .planner import (PlanConfig, astar, geometric_cost, path_length,
                      smooth_path)
from .semantic_map import SemanticGrid

__all__ = ["MAPS", "build_maps", "Scenario", "make_scenarios",
           "run_comparison", "table"]


MAPS = ("geometric", "semantic", "oracle")


@dataclass
class Scenario:
    """一组起终点。``gap_y`` 是中间可通行缺口的位置，用来构造"只能从缺口过"的题。"""
    name: str
    start: Tuple[float, float]
    goal: Tuple[float, float]

    def cells(self, grid) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        s = grid.cell_of(np.asarray([self.start], float))[0]
        g = grid.cell_of(np.asarray([self.goal], float))[0]
        return (int(s[1]), int(s[0])), (int(g[1]), int(g[0]))


def _segment_cost(cost: np.ndarray, grid: SemanticGrid, a, b, n: int = 160,
                  max_cost: float = 20.0) -> float:
    """直线段 a→b 上采样的平均语义代价（inf 记为 ``max_cost``）。"""
    t = np.linspace(0.0, 1.0, n)[:, None]
    xy = np.asarray(a, float)[None] * (1 - t) + np.asarray(b, float)[None] * t
    idx = grid.cell_of(xy)
    ok = np.all((idx >= 0) & (idx < grid.n), axis=1)
    if not ok.any():
        return 0.0
    v = cost[idx[ok, 1], idx[ok, 0]]
    v = np.where(np.isfinite(v), v, max_cost)
    return float(v.mean())


def make_scenarios(grid: SemanticGrid, n: int = 3, seed: int = 0,
                   oracle_map: Optional[np.ndarray] = None,
                   n_cand: int = 300) -> List[Scenario]:
    """在已观测区域内挑 n 组起终点。

    ``oracle_map`` 给定时按**区分度**挑：只留那些"几何上通畅、语义上不该走"
    的题——直线段在真值语义图上的平均代价高的优先。不挑的话可能全是不经
    植被/建筑的题，三张代价图会给出同一条路径，对比实验什么都说明不了
    （实测未挑时三条路径长度完全相同，均为 46.37 m）。
    """
    obs = grid.observed()
    out: List[Scenario] = []
    for sc in (Scenario("左→右", (-22.0, -12.0), (22.0, 12.0)),
               Scenario("对角", (-20.0, 14.0), (20.0, -10.0)),
               Scenario("横穿", (0.0, -22.0), (0.0, 20.0))):
        s, g = sc.cells(grid)
        if obs[s] and obs[g]:
            out.append(sc)
    if oracle_map is None or len(out) >= n:
        return out[:n]

    rng = np.random.default_rng(seed)
    idx = np.argwhere(obs)
    cand: List[Tuple[float, Scenario]] = []
    if len(idx) >= 2:
        for k in range(n_cand):
            ra, ca = idx[rng.integers(len(idx))]
            rb, cb = idx[rng.integers(len(idx))]
            pa, pb = grid.centre(ra, ca), grid.centre(rb, cb)
            if math.dist(pa, pb) < 20.0:          # 太近的题绕不绕都一样
                continue
            cand.append((_segment_cost(oracle_map, grid, pa, pb),
                         Scenario("判别%d" % k, tuple(pa), tuple(pb))))
    cand.sort(key=lambda x: -x[0])
    for _, sc in cand:                            # 取区分度最高的，且彼此不重复
        if len(out) >= n:
            break
        if all(math.dist(sc.start, o.start) + math.dist(sc.goal, o.goal) > 8.0
               for o in out):
            out.append(sc)
    return out[:n]


# --------------------------------------------------------------------------- #
def build_maps(grid: SemanticGrid, seg: Optional[np.ndarray] = None,
               oracle_grid: Optional[SemanticGrid] = None
               ) -> Dict[str, np.ndarray]:
    """构造三张代价图。``oracle_grid`` 给定时用它的语义做上界。"""
    m = {"geometric": geometric_cost(grid)}
    cls = grid.semantic()
    conf = grid.semantic_conf()
    obs = grid.observed()
    sem = TRAVERSAL_COST[cls].copy()
    sem[~obs] = 2.0
    sem[(grid.probability() > 0.7)] = np.inf
    # 语义置信度低时向中性代价插值
    low = obs & (conf < 0.6)
    sem[low] = 0.5 * sem[low] + 0.5 * 2.0
    m["semantic"] = sem

    if oracle_grid is not None:
        ocls = oracle_grid.semantic()
        oobs = oracle_grid.observed()
        orc = TRAVERSAL_COST[ocls].copy()
        orc[~oobs] = 2.0
        orc[(oracle_grid.probability() > 0.7)] = np.inf
        m["oracle"] = orc
    else:
        m["oracle"] = m["semantic"]
    return m


def run_comparison(grid: SemanticGrid, maps: Dict[str, np.ndarray],
                   scenarios: Sequence[Scenario],
                   cfg: Optional[PlanConfig] = None) -> List[Dict[str, float]]:
    """三张代价图 × 每组起终点，返回逐条指标。

    ``traversal`` 统一用 **oracle 代价图**打分：按规划器自己的代价图打分没有
    可比性（语义图把满路判成建筑时，它给自己的路径打 10.50）。几何路径切过
    植被/建筑得分高，语义路径绕开得分低。
    """
    rows: List[Dict[str, float]] = []
    conf = grid.semantic_conf()
    scorer = maps["oracle"]
    for sc in scenarios:
        start, goal = sc.cells(grid)
        for name in MAPS:
            cost = maps[name]
            r = astar(cost, start, goal, cfg)
            if not r.ok:
                rows.append({"map": name, "scenario": sc.name, "ok": 0.0,
                             "length": float("nan"), "cost": float("nan"),
                             "collision": float("nan"),
                             "uncertainty": float("nan"), "traversal": float("nan"),
                             "expand": float(r.expanded)})
                continue
            cells = smooth_path(cost, r.cells)
            xy = np.asarray([grid.centre(a, b) for a, b in cells], float)
            rows.append({
                "map": name, "scenario": sc.name, "ok": 1.0,
                "length": path_length(grid, cells),
                "cost": float(r.cost),
                "collision": float(collision_count(cost, xy, grid)),
                "uncertainty": float(path_uncertainty(conf, xy, grid)),
                "expand": float(r.expanded),
                "traversal": float(travelled_through(scorer, xy, grid)),
            })
    return rows


def summarise(rows: Sequence[Dict[str, float]]) -> Dict[str, Dict[str, float]]:
    out: Dict[str, Dict[str, float]] = {}
    for name in MAPS:
        sel = [r for r in rows if r["map"] == name]
        if not sel:
            continue
        d: Dict[str, float] = {}
        for k in ("ok", "length", "collision", "uncertainty", "expand",
                  "traversal"):
            v = [r[k] for r in sel if np.isfinite(r[k])]
            d[k] = float(np.mean(v)) if v else float("nan")
        d["n"] = float(len(sel))
        out[name] = d
    return out


def table(rows: Sequence[Dict[str, float]]) -> str:
    s = summarise(rows)
    label = {"geometric": "几何代价图", "semantic": "语义代价图",
             "oracle": "语义（真值分割）"}
    lines = ["%-16s %8s %10s %10s %12s %10s %10s"
             % ("代价图", "成功率", "路径长度", "碰撞数", "语义不确定性", "展开节点",
                "真值语义代价")]
    lines.append("-" * 84)
    for name in MAPS:
        if name not in s:
            continue
        d = s[name]
        lines.append("%-16s %7.0f%% %10.2f %10.2f %12.3f %10.0f %10.2f"
                     % (label[name], 100 * d["ok"], d["length"], d["collision"],
                        d["uncertainty"], d["expand"], d["traversal"]))
    return "\n".join(lines)
