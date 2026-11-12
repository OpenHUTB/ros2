#!/usr/bin/env python3
"""Train, evaluate, or launch the MuJoCo forklift pallet-docking controller."""

from __future__ import annotations

import argparse
import json
import select
import sys
from pathlib import Path

from mujoco_forklift_pallet_docking.policy import ImagePolicy
from mujoco_forklift_pallet_docking.simulation import ForkliftDockingSimulation, train_policy

DEFAULT_POLICY = (
    Path(__file__).parent
    / "mujoco_forklift_pallet_docking"
    / "models"
    / "pallet_docking_policy.npz"
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("train", "run", "evaluate"), default="run")
    parser.add_argument("--start-x", type=float, default=-4.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--brake-scale", type=float, default=1.0, help="自动控制的制动力校准系数，默认1.0")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--model-path", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    model_path = args.model_path or DEFAULT_POLICY
    if args.mode != "train" and model_path.is_file():
        policy = ImagePolicy.load(str(model_path))
        mse = None
    else:
        policy, mse = train_policy(seed=args.seed)
    if args.mode == "train":
        if args.model_path:
            args.model_path.parent.mkdir(parents=True, exist_ok=True)
            policy.save(str(args.model_path))
        print(json.dumps({"held_out_mse": round(mse, 6)}, ensure_ascii=False))
        return
    starts = [-5.0, -4.0, -3.0] if args.mode == "evaluate" else [args.start_x]
    results = []
    for start_x in starts:
        sim = ForkliftDockingSimulation(start_x=start_x, speed=args.speed, brake_scale=args.brake_scale)
        try:
            results.append({"start_x_m": start_x, **sim.run(policy)})
        finally:
            sim.close()
    report = {"brake_scale": args.brake_scale, "held_out_mse": None if mse is None else round(mse, 6), "scenarios": results}
    output = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
