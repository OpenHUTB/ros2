#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比实验：4 载荷 × 3 风况 × 3 轨迹 × 4 控制器。

所有运行从**同一起点** (0,0,1.2) 起飞、用同一段前 4 s 暂态，稳态指标只统计
4 s 之后。载荷档位里 ``extra``（+750 g）刻意不进训练集，用来分开"泛化"与
"在线辨识"两种能力：离线残差网只能外推，RLS 能在几秒内直接测出来。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Sequence

import numpy as np

from .controllers import TRACKERS, make_tracker
from .disturbances import PAYLOAD_LEVELS, WIND_LEVELS, make_plant
from .metrics import residual_metrics, tracking_metrics
from .plant import ResidualPlant
from .residual import ResidualNet
from .trajectories import make_trajectory

__all__ = ["RunRecord", "run_once", "run_grid", "save_results", "load_results",
           "START_POS", "DEFAULT_CONTROLLERS", "DEFAULT_SEEDS"]

#: 起点取在轨迹上（含初始偏航）。四旋翼的偏航力矩受混控预算约束，
#: 从偏航误差 90° 的地方起飞会挤掉滚转/俯仰的权限、直接翻机（见 dataset.run）。
START_ON_TRAJ = True
SETTLE = 3.0
DEFAULT_CONTROLLERS = ("pid", "geo", "geo_res", "geo_rls")
DEFAULT_PAYLOADS = ("none", "light", "heavy", "extra")
DEFAULT_WINDS = ("calm", "breeze", "windy")
DEFAULT_TRAJS = ("hover", "helix", "figure8")
DEFAULT_SEEDS = (0, 1, 2)


@dataclass
class RunRecord:
    controller: str
    payload: str
    wind: str
    traj: str
    seed: int
    metrics: Dict[str, float] = field(default_factory=dict)
    series: Dict[str, list] = field(default_factory=dict)

    def key(self) -> str:
        return "%s|%s|%s|%s|%d" % (self.controller, self.payload, self.wind,
                                   self.traj, self.seed)


def run_once(controller: str, payload: str, wind: str, traj_name: str, seed: int = 0,
             net: Optional[ResidualNet] = None, dt: float = 0.01,
             settle: float = SETTLE, keep_series: bool = True) -> RunRecord:
    cfg = make_plant(payload, wind, seed=seed)
    plant = ResidualPlant(plant=cfg, scene="open_ground", seed=seed, dt=dt)
    traj = make_trajectory(traj_name)
    tracker = make_tracker(controller, plant, traj, net=net)
    if START_ON_TRAJ:
        p0, _, _, yaw0, _ = traj.eval(0.0)
        plant.reset(pos=tuple(p0), yaw=float(yaw0))
    else:
        plant.reset(pos=START_POS)

    n = int(round(traj.duration / dt))
    t = np.empty(n)
    err = np.empty(n)
    pos = np.empty((n, 3))
    ref = np.empty((n, 3))
    d_true = np.empty((n, 3))
    d_hat = np.empty((n, 3))
    thrust = np.empty(n)

    for k in range(n):
        tk = k * dt
        u = tracker.act(tk)
        plant.step_with_control(u)
        tracker.observe()
        st = plant.state()
        p_ref = traj.eval(tk)[0]
        t[k] = tk
        pos[k] = st.pos
        ref[k] = p_ref
        err[k] = float(np.linalg.norm(st.pos - p_ref))
        d_true[k] = tracker.last_delta
        d_hat[k] = tracker.last_residual_b
        thrust[k] = float(np.sum(u))

    m = tracking_metrics(t, err, settle=settle, dt=dt)
    if tracker.has_residual:
        # 这里只报 RMSE/偏差，不报 R²：轨迹稳态段的速度几乎不变，
        # 残差近似常值，R² 的分母趋零会给出毫无意义的巨大负值。
        rm = residual_metrics(d_true, d_hat, settle=int(settle / dt))
        m["res_rmse"] = rm["res_rmse"]
        m["res_bias"] = rm["res_bias"]
    m["thrust_mean"] = float(np.mean(thrust[int(settle / dt):]))
    m["energy"] = float(np.sum(thrust ** 2) * dt)
    m["final_err"] = float(np.mean(err[-int(2.0 / dt):]))

    rec = RunRecord(controller, payload, wind, traj_name, seed, m)
    if keep_series:
        rec.series = {"t": t.tolist(), "err": err.tolist(),
                      "pos": pos.tolist(), "ref": ref.tolist(),
                      "delta_true": d_true.tolist(), "delta_hat": d_hat.tolist(),
                      "thrust": thrust.tolist()}
    return rec


def run_grid(controllers: Sequence[str] = DEFAULT_CONTROLLERS,
             payloads: Sequence[str] = DEFAULT_PAYLOADS,
             winds: Sequence[str] = DEFAULT_WINDS,
             trajs: Sequence[str] = DEFAULT_TRAJS,
             seeds: Sequence[int] = DEFAULT_SEEDS,
             net: Optional[ResidualNet] = None, keep_series: bool = False,
             verbose: bool = True) -> List[RunRecord]:
    out: List[RunRecord] = []
    total = len(controllers) * len(payloads) * len(winds) * len(trajs) * len(seeds)
    i = 0
    t0 = time.time()
    for c in controllers:
        for p in payloads:
            for w in winds:
                for tj in trajs:
                    for s in seeds:
                        i += 1
                        rec = run_once(c, p, w, tj, s, net=net,
                                       keep_series=keep_series)
                        out.append(rec)
                        if verbose:
                            print("[%3d/%3d] %-8s %-6s %-7s %-8s seed=%d  "
                                  "RMSE=%.4f m  max=%.4f m  (%.0fs)"
                                  % (i, total, c, p, w, tj, s,
                                     rec.metrics["rmse"], rec.metrics["max"],
                                     time.time() - t0), flush=True)
    return out


# --------------------------------------------------------------------------- #
def save_results(records: Sequence[RunRecord], path: str, meta: Optional[dict] = None) -> None:
    payload = {"meta": meta or {},
               "records": [asdict(r) for r in records]}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False)
    print("结果已写入 %s（%d 条记录，%.1f KB）"
          % (path, len(records), len(json.dumps(payload)) / 1024))


def load_results(path: str):
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    recs = []
    for r in d["records"]:
        recs.append(RunRecord(r["controller"], r["payload"], r["wind"], r["traj"],
                              r["seed"], r.get("metrics", {}), r.get("series", {})))
    return d.get("meta", {}), recs


def table(records: Sequence[RunRecord], traj: Optional[str] = None,
          metric: str = "rmse", unit: str = "m") -> str:
    """按 (载荷, 风况) 行列、控制器分列的纯文本表。"""
    from .metrics import format_table
    controllers = sorted({r.controller for r in records})
    rows = []
    combos = sorted({(r.payload, r.wind) for r in records},
                    key=lambda x: (list(PAYLOAD_LEVELS).index(x[0]),
                                   list(WIND_LEVELS).index(x[1])))
    for p, w in combos:
        cells = []
        for c in controllers:
            vals = [r.metrics[metric] for r in records
                    if r.controller == c and r.payload == p and r.wind == w
                    and (traj is None or r.traj == traj)]
            cells.append("%.4f" % float(np.mean(vals)) if vals else "—")
        rows.append([p, w] + cells)
    hdr = ["载荷", "风况"] + [c + " (%s)" % unit for c in controllers]
    return format_table(rows, hdr)


def table_by_traj(records: Sequence[RunRecord], metric: str = "rmse",
                  payload: str = "heavy", wind: str = "windy") -> str:
    from .metrics import format_table
    controllers = sorted({r.controller for r in records})
    trajs = sorted({r.traj for r in records})
    rows = []
    for tj in trajs:
        cells = []
        for c in controllers:
            vals = [r.metrics[metric] for r in records
                    if r.controller == c and r.payload == payload
                    and r.wind == wind and r.traj == tj]
            cells.append("%.4f" % float(np.mean(vals)) if vals else "—")
        rows.append([tj] + cells)
    return format_table(rows, ["轨迹"] + controllers)
