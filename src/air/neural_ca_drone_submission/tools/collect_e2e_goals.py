#!/usr/bin/env python3
"""Collect full-stack demonstrations toward *random* goals.

The plain distillation data only contains the eastbound missions, so the goal
vector in the dataset is almost always "straight ahead" and a reactive student
learns to ignore it.  Here the teacher flies to uniformly sampled reachable
goals (including goals behind the start), which teaches the student to turn.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ca_drone.pipeline import NeuralStack, run_episode  # noqa: E402
from ca_drone.sim.world import make_world  # noqa: E402

SCENES = ["sparse", "dense", "dynamic", "corridor"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--weights", default="weights")
    ap.add_argument("--out", default=os.path.join("data", "e2e_goals.npz"))
    ap.add_argument("--episodes", type=int, default=16)
    ap.add_argument("--max-steps", type=int, default=650)
    ap.add_argument("--append", default=None)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    stack = NeuralStack(args.weights)
    X, G, Y = [], [], []
    for ep in range(args.episodes):
        scene = SCENES[ep % len(SCENES)]
        probe = make_world(scene)
        goal = None
        for _ in range(80):
            g = np.array([rng.uniform(6.0, 47.0), rng.uniform(-14.0, 14.0), probe.flight_z])
            if probe.inside(g, margin=1.0) and probe.clearance(g) > 1.2:
                goal = g
                break
        if goal is None:
            continue
        buf = []
        r = run_episode(scene, "full", stack, max_steps=args.max_steps,
                        render_every=0, seed=ep, record=buf, goal=goal)
        for depth, gfeat, cmd in buf:
            X.append(np.clip(depth / 12.0, 0, 1).astype(np.float32))
            G.append(gfeat)
            Y.append(cmd)
        print(f"[{ep + 1}/{args.episodes}] {scene} goal=({goal[0]:.0f},{goal[1]:.0f}) "
              f"success={r.success} steps={r.steps} samples={len(buf)}", flush=True)
    X, G, Y = (np.asarray(a, dtype=np.float32) for a in (X, G, Y))
    if args.append and os.path.exists(args.append):
        old = np.load(args.append)
        X = np.concatenate([old["depth"], X])
        G = np.concatenate([old["goal"], G])
        Y = np.concatenate([old["cmd"], Y])
        print(f"appended {args.append}: {len(X)} samples total")
    np.savez_compressed(args.out, depth=X, goal=G, cmd=Y)
    print(f"saved {args.out} with {len(X)} samples")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
