#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验矩阵：4 档链路 × 3 种策略 × 多种子，输出 JSON/CSV/NPZ。"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import numpy as np

from .env import build_mission, build_world
from .link import default_profiles
from .sim import RunResult, run_case
from .supervisor import SupervisorParams


def run_matrix(outdir: Path, profiles: Iterable[str] | None = None,
               policies: Sequence[str] = ("hold_last", "hover", "supervised"),
               seeds: Sequence[int] = (0, 1, 2), duration: float = 45.0,
               params: SupervisorParams | None = None,
               verbose: bool = True) -> Dict[str, object]:
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    prof_table = default_profiles()
    names = list(profiles) if profiles else list(prof_table)
    rows: List[Dict[str, object]] = []

    for pname in names:
        profile = prof_table[pname]
        for policy in policies:
            for seed in seeds:
                world = build_world()
                mission = build_mission()
                res: RunResult = run_case(
                    profile, policy, mission, world.quad, world.controller, world.scene,
                    duration=duration, seed=seed, params=params)
                rec = res.to_dict()
                rec["seed"] = int(seed)
                rows.append(rec)
                np.savez(
                    outdir / f"trace_{pname}_{policy}_s{seed}.npz",
                    t=np.asarray(res.t), pos=np.asarray(res.pos),
                    mode=np.asarray(res.mode), age=np.asarray(res.age),
                    pdr=np.asarray(res.pdr), clearance=np.asarray(res.clearance),
                    speed=np.asarray(res.speed))
                if verbose:
                    print(f"{pname:9s} {policy:10s} seed={seed} "
                          f"完成={int(res.completed)} 接触={int(res.contact)} "
                          f"净空={res.min_clearance:.2f} 用时={res.duration:.1f}s "
                          f"最大年龄={res.max_age:.2f}s 模式切换={res.switches}")

    # 汇总：同一 (profile, policy) 的多种子均值
    agg: Dict[str, Dict[str, float]] = {}
    for rec in rows:
        key = f"{rec['profile']}|{rec['policy']}"
        bucket = agg.setdefault(key, {"n": 0, "profile": rec["profile"],
                                       "policy": rec["policy"]})
        bucket["n"] += 1
        for field in ("completed", "returned", "contact", "switches"):
            bucket[field] = bucket.get(field, 0.0) + float(rec[field])
        for field in ("min_clearance_m", "max_cmd_age_s", "duration_s",
                      "progress", "max_path_deviation_m", "final_home_error_m"):
            val = rec[field]
            if val is None:
                continue
            bucket[field] = bucket.get(field, 0.0) + float(val)
    for key, b in agg.items():
        n = b.pop("n")
        for field in list(b):
            if field in ("profile", "policy"):
                continue
            if field in ("completed", "returned", "contact", "switches"):
                b[field] = round(b[field] / n, 2) if field == "switches" else round(b[field] / n, 3)
            else:
                b[field] = round(b[field] / n, 3)
        b["n"] = n

    payload = {"rows": rows, "aggregate": agg}
    (outdir / "results.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    fields = ["profile", "policy", "seed", "completed", "returned", "contact",
              "min_clearance_m", "max_cmd_age_s", "max_speed_mps", "switches",
              "final_home_error_m", "max_path_deviation_m", "duration_s", "progress"]
    with (outdir / "results.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for rec in sorted(rows, key=lambda r: (r["profile"], r["policy"], r["seed"])):
            w.writerow(rec)
    return payload
