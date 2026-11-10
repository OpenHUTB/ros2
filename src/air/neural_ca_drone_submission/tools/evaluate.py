#!/usr/bin/env python3
"""Batch evaluation of the classical baseline and the neural policies.

Example::

    python tools/evaluate.py --weights weights --scenes all --modes expert,full,e2e \
        --seeds 3 --out results
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from collections import defaultdict

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np  # noqa: E402

from ca_drone.pipeline import NeuralStack, run_episode  # noqa: E402

SCENES = ["sparse", "dense", "dynamic", "corridor", "unseen"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--weights", default="weights")
    ap.add_argument("--scenes", default="all")
    ap.add_argument("--modes", default="expert,full,e2e")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--max-steps", type=int, default=650)
    ap.add_argument("--out", default="results")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    scenes = SCENES if args.scenes == "all" else args.scenes.split(",")
    modes = args.modes.split(",")
    stack = NeuralStack(args.weights) if any(m != "expert" for m in modes) else None
    rows = []
    t_all = time.time()
    for scene in scenes:
        for mode in modes:
            for seed in range(args.seeds):
                t = time.time()
                r = run_episode(scene, mode, stack, max_steps=args.max_steps, seed=seed)
                row = r.as_dict()
                row["seed"] = seed
                row["wall_s"] = round(time.time() - t, 2)
                rows.append(row)
                print(f"{scene:9s} {mode:6s} seed={seed} success={row['success']} "
                      f"collision={row['collision']} t={row['duration_s']}s "
                      f"clear={row['min_clearance_m']}m wall={row['wall_s']}s",
                      flush=True)
    with open(os.path.join(args.out, "episodes.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    # aggregate per (scene, mode)
    agg = defaultdict(list)
    for r in rows:
        agg[(r["scene"], r["mode"])].append(r)
    summary = {}
    for (scene, mode), rs in agg.items():
        summary.setdefault(scene, {})[mode] = {
            "success_rate": float(np.mean([r["success"] for r in rs])),
            "collision_rate": float(np.mean([r["collision"] for r in rs])),
            "duration_s": float(np.mean([r["duration_s"] for r in rs])),
            "path_length_m": float(np.mean([r["path_length_m"] for r in rs])),
            "min_clearance_m": float(np.mean([r["min_clearance_m"] for r in rs])),
            "plan_ms": float(np.mean([r["plan_ms"] for r in rs])),
            "infer_ms": float(np.mean([r["infer_ms"] for r in rs])),
            "episodes": len(rs),
        }
    with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"generated": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "total_wall_s": round(time.time() - t_all, 1),
                   "summary": summary}, f, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
