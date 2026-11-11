"""三场景 × 三策略闭环评测，输出 results/benchmark.json。

所有随机性（场景布局）由 --seed 固定，默认 seed=0；结果文件同时记录
seed、MuJoCo 版本与 success 判定口径，便于复核。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import mujoco

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rov_mujoco_slam_nn_nav.nav import NavConfig, Navigation  # noqa: E402
from rov_mujoco_slam_nn_nav.sim import ROVSim  # noqa: E402

SCENES = ("pool", "pipe_field", "trench")
POLICIES = ("classical", "neural", "e2e")


def main():
    ap = argparse.ArgumentParser(description="ROV 三场景三策略闭环评测")
    ap.add_argument("--seed", type=int, default=0, help="场景随机种子（默认 0）")
    ap.add_argument("--scenes", default=",".join(SCENES))
    ap.add_argument("--policies", default=",".join(POLICIES))
    ap.add_argument("--max-time", type=float, default=90.0)
    ap.add_argument("--out", default=os.path.join("results", "benchmark.json"))
    args = ap.parse_args()

    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    policies = [p.strip() for p in args.policies.split(",") if p.strip()]
    cfg = NavConfig()
    out = {}
    for policy in policies:
        for scene in scenes:
            t0 = time.time()
            sim = ROVSim(scene=scene, seed=args.seed)
            nav = Navigation(sim, cfg, policy=policy, with_depth=(policy == "e2e"))
            r = nav.run(max_time=args.max_time)
            r["wall_time_s"] = round(time.time() - t0, 1)
            out[f"{policy}/{scene}"] = r
            print(policy, scene, json.dumps(r, ensure_ascii=False), flush=True)

    payload = {
        "seed": args.seed,
        "mujoco": mujoco.__version__,
        "max_time_s": args.max_time,
        "success_definition": (
            "success = reached(done) and not fallen（不含碰撞约束）; "
            "success_no_collision = success and collisions == 0"
        ),
        "results": out,
    }
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print("saved", args.out, "seed", args.seed)


if __name__ == "__main__":
    main()
