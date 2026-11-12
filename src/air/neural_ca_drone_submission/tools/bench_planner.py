#!/usr/bin/env python3
"""Benchmark classical A* against the learned cost-to-go heuristic.

The learned field is a *scaled* estimate of the true cost-to-go, so the
heuristic weight trades node expansions against path optimality.  This script
sweeps the weight and writes ``results/planner_bench.csv``.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np  # noqa: E402

from ca_drone.mapping import GridMap  # noqa: E402
from ca_drone.models.planner import NeuralHeuristic, predict_field  # noqa: E402
from ca_drone.planning.astar import astar  # noqa: E402
from ca_drone.sim.world import make_world  # noqa: E402

SCENES = ["sparse", "dense", "dynamic", "corridor"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--weights", default="weights")
    ap.add_argument("--out", default="results")
    ap.add_argument("--weights-list", default="0.75,1.0,1.15,1.3")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    model, _ = NeuralHeuristic.load(os.path.join(args.weights, "planner.npz"))
    rows = []
    for scene in SCENES:
        w = make_world(scene)
        gm = GridMap.from_world(w, 0.5, inflate=0.45)
        occ = gm.plan_grid(margin=1)
        s, g = gm.to_cell(w.start), gm.to_cell(w.goal)
        t = time.time()
        r = astar(occ, s, g, 0.5, weight=1.0)
        ms = (time.time() - t) * 1e3
        rows.append({"scene": scene, "method": "classical", "weight": 1.0,
                     "expanded": r["expanded"], "cost_m": round(r["cost"], 3),
                     "plan_ms": round(ms, 2)})
        t = time.time()
        field = predict_field(model, occ, g, 0.5)
        infer_ms = (time.time() - t) * 1e3
        for wt in [float(x) for x in args.weights_list.split(",")]:
            r = astar(occ, s, g, 0.5, heuristic=field, weight=wt)
            rows.append({"scene": scene, "method": "neural", "weight": wt,
                         "expanded": r["expanded"], "cost_m": round(r["cost"], 3),
                         "plan_ms": round(infer_ms, 2)})
        print(f"[{scene}] classical expanded={rows[-len(rows) + 1]['expanded']} "
              f"field {infer_ms:.1f} ms", flush=True)
    path = os.path.join(args.out, "planner_bench.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    for r in rows:
        print("{scene:9s} {method:9s} w={weight:<4} expanded={expanded:<6d} "
              "cost={cost_m:<8.3f} ms={plan_ms}".format(**r))
    print(f"saved {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
