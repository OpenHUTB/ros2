#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环评估：风扰下 NN 补偿 vs 不补偿的直接投放。"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src"))
from uav_wind_aware_delivery.control import GeometricController  # noqa: E402
from uav_wind_aware_delivery.mission import DeliveryMission, MissionConfig  # noqa: E402
from uav_wind_aware_delivery.sim import DeliverySim, SimConfig  # noqa: E402


def run_episode(wind, compensate, seed=0):
    sim = DeliverySim(SimConfig(wind=np.asarray(wind, dtype=float), substeps=2))
    mission = DeliveryMission(
        MissionConfig(target=np.array([5.0, 2.0, 0.0]), compensate=compensate,
                      release_alt=3.0, hover_time=0.8),
        model_path=os.path.join(ROOT, "models", "impact_mlp.npz"),
    )
    ctrl = GeometricController()
    rng = np.random.default_rng(seed)
    # 起飞位置略随机，避免完全理想化
    sim.cfg.start_pos = np.array([rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), 1.6])
    sim.reset(wind=np.asarray(wind, dtype=float))
    released = False
    for _ in range(5000):
        st = sim.state()
        out = mission.update(st)
        if out["release"] and not released:
            sim.release()
            released = True
        ctrl.act(sim, out["setpoint"], np.zeros(3))
        if mission.done:
            break
    st = sim.state()
    err = float(np.linalg.norm(st.payload_pos[:2] - np.array([5.0, 2.0])))
    return {
        "seed": seed,
        "wind": [round(float(w), 3) for w in wind],
        "compensate": compensate,
        "landed": bool(st.landed),
        "released": bool(st.released),
        "error_m": round(err, 4),
        "success": bool(st.landed and err <= 0.5),
        "time_s": round(float(st.t), 2),
        "release_point": [round(float(v), 3) for v in mission.rel],
        "predicted_offset": [round(float(v), 3) for v in mission.predicted_offset],
    }


def main():
    rng = np.random.default_rng(20261104)
    winds = []
    for _ in range(10):
        winds.append([rng.uniform(-4.5, 4.5), rng.uniform(-2.5, 2.5), 0.0])
    rows = []
    for i, w in enumerate(winds):
        rows.append(run_episode(w, True, seed=i))
        rows.append(run_episode(w, False, seed=i))

    def summary(comp):
        sel = [r for r in rows if r["compensate"] is comp]
        errs = np.array([r["error_m"] for r in sel])
        return {
            "episodes": len(sel),
            "success_rate": round(float(np.mean([r["success"] for r in sel])), 3),
            "mean_error_m": round(float(errs.mean()), 4),
            "p90_error_m": round(float(np.percentile(errs, 90)), 4),
            "mean_time_s": round(float(np.mean([r["time_s"] for r in sel])), 2),
        }

    comp = summary(True)
    base = summary(False)
    out = {
        "scenario": "wind-aware payload delivery, target=(5,2), release_alt=3m",
        "episodes_per_method": 10,
        "nn_compensation": comp,
        "no_compensation": base,
        "improvement_error_pct": round((1 - comp["mean_error_m"] / base["mean_error_m"]) * 100, 1),
        "rows": rows,
    }
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", "benchmark.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(json.dumps({k: out[k] for k in ("nn_compensation", "no_compensation",
                                         "improvement_error_pct")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
