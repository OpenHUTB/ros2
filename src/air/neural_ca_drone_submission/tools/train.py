#!/usr/bin/env python3
"""Train one or all of the four neural modules.

Examples::

    python tools/train.py --module perception --data data --epochs 4
    python tools/train.py --module all --data data --weights weights

The checkpoints are plain ``.npz`` files, so they can be loaded by the ROS nodes
and by the evaluation scripts without PyTorch.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np  # noqa: E402

from ca_drone.training import train as T  # noqa: E402

DATASETS = ["perception", "planner", "control", "e2e"]
# Must match the shipped weights and docs/results.md.  Keep this table in sync
# with the defaults of the train_* functions in ca_drone/training/train.py.
DEFAULT_EPOCHS = {"perception": 4, "planner": 12, "control": 60, "e2e": 12}


def _load(data_dir: str, name: str):
    path = os.path.join(data_dir, f"{name}.npz")
    if not os.path.exists(path):
        raise SystemExit(f"missing dataset {path}; run tools/make_data.py first")
    with np.load(path) as f:
        return {k: f[k] for k in f.files}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--module", default="all", choices=["all"] + DATASETS)
    ap.add_argument("--data", default="data")
    ap.add_argument("--weights", default="weights")
    ap.add_argument("--epochs", type=int, default=0, help="0 = module default")
    ap.add_argument("--batch", type=int, default=0)
    ap.add_argument("--lr", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.weights, exist_ok=True)
    names = DATASETS if args.module == "all" else [args.module]
    summary = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "modules": {}}
    for name in names:
        data = _load(args.data, name)
        ep = args.epochs or DEFAULT_EPOCHS[name]
        kw = {}
        if args.batch:
            kw["batch"] = args.batch
        if args.lr:
            kw["lr"] = args.lr
        fn = {"perception": T.train_perception, "planner": T.train_planner,
              "control": T.train_control, "e2e": T.train_e2e}[name]
        t = time.time()
        res = fn(data, epochs=ep, seed=args.seed, **kw)
        ckpt = os.path.join(args.weights, f"{name}.npz")
        res["model"].save(ckpt, meta={"module": name, "epochs": ep,
                                      "samples": int(len(data["depth"]) if name in ("perception", "e2e") else len(data["x"]))})
        hist_path = os.path.join(args.weights, f"{name}_history.json")
        T._save_history(res["history"], hist_path)
        summary["modules"][name] = {"params": res["model"].num_params(),
                                    "seconds": round(time.time() - t, 1),
                                    "checkpoint": ckpt,
                                    "last": res["history"]["epochs"][-1]}
        print(f"saved {ckpt}"
              f" ({os.path.getsize(ckpt) / 1024:.1f} KB)")
    with open(os.path.join(args.weights, "train_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
