#!/usr/bin/env python3
"""Generate the four supervised datasets used by the neural modules.

Usage::

    python tools/make_data.py --out data --scale 1.0

The generator only depends on CA-Sim (NumPy + Pillow), so the whole dataset can
be rebuilt on any machine without a GPU or a ROS installation.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ca_drone.data import collect  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="data", help="output directory")
    ap.add_argument("--scale", type=float, default=1.0, help="scale all dataset sizes")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--only", default="all",
                    help="comma separated subset of perception,planner,control,e2e")
    ap.add_argument("--e2e-episodes", type=int, default=0,
                    help="override the number of e2e episodes (0 = default)")
    ap.add_argument("--e2e-limit", type=int, default=14000,
                    help="maximum number of e2e samples")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    s = max(0.05, args.scale)
    only = {"perception", "planner", "control", "e2e"} if args.only == "all" \
        else {m.strip() for m in args.only.split(",") if m.strip()}
    report = {}
    t0 = time.time()

    if "perception" in only:
        t = time.time()
        d = collect.perception_dataset(n=int(1200 * s), seed=args.seed)
        np.savez_compressed(os.path.join(args.out, "perception.npz"), **d)
        report["perception"] = {"n": int(len(d["depth"])), "seconds": round(time.time() - t, 2)}

    if "planner" in only:
        t = time.time()
        d = collect.planner_dataset(n_maps=max(4, int(50 * s)), per_map=int(130), seed=args.seed + 1)
        np.savez_compressed(os.path.join(args.out, "planner.npz"), **d)
        report["planner"] = {"n": int(len(d["x"])), "seconds": round(time.time() - t, 2)}

    if "control" in only:
        t = time.time()
        d = collect.control_dataset(episodes=max(4, int(36 * s)), seed=args.seed + 2)
        np.savez_compressed(os.path.join(args.out, "control.npz"), **d)
        report["control"] = {"n": int(len(d["x"])), "seconds": round(time.time() - t, 2)}

    if "e2e" in only:
        t = time.time()
        episodes = args.e2e_episodes or max(4, int(30 * s))
        d = collect.e2e_dataset(episodes=episodes, seed=args.seed + 3,
                                limit=args.e2e_limit)
        np.savez_compressed(os.path.join(args.out, "e2e.npz"), **d)
        report["e2e"] = {"n": int(len(d["depth"])), "seconds": round(time.time() - t, 2)}

    report["total_seconds"] = round(time.time() - t0, 2)
    with open(os.path.join(args.out, "dataset_report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
