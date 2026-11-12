#!/usr/bin/env python3
"""Collect on-policy end-to-end data.

Two passes are supported:

``--mode full``
    The full stack flies and records its own (depth, goal, command) pairs.
    This is plain behaviour cloning and only covers states the teacher visits.

``--mode dagger``
    The *student* flies, but the full stack labels every state the student
    reaches.  This is the DAgger correction: the data now contains the states
    where the student drifts past the goal or grazes a wall, which is exactly
    where behaviour cloning fails.

The resulting archive can be appended to the existing dataset with
``--append data/e2e.npz``.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ca_drone.pipeline import NeuralStack, run_episode  # noqa: E402

SCENES = ["sparse", "dense", "dynamic", "corridor"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--weights", default="weights")
    ap.add_argument("--out", default=os.path.join("data", "e2e_dagger.npz"))
    ap.add_argument("--mode", choices=["full", "dagger"], default="dagger")
    ap.add_argument("--episodes", type=int, default=36)
    ap.add_argument("--max-steps", type=int, default=650)
    ap.add_argument("--append", default=None,
                    help="existing .npz to concatenate with the new samples")
    args = ap.parse_args()
    stack = NeuralStack(args.weights)
    X, G, Y = [], [], []
    for ep in range(args.episodes):
        scene = SCENES[ep % len(SCENES)]
        buf = []
        r = run_episode(scene, args.mode, stack, max_steps=args.max_steps,
                        render_every=0, seed=ep, record=buf)
        for depth, goal, cmd in buf:
            X.append(np.clip(depth / 12.0, 0, 1).astype(np.float32))
            G.append(goal)
            Y.append(cmd)
        print(f"[{ep + 1}/{args.episodes}] {scene} success={r.success} "
              f"steps={r.steps} samples={len(buf)} total={len(X)}", flush=True)
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
