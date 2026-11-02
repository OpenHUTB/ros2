"""三场景 × 三策略闭环评测，输出 results/benchmark.json。"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rov_mujoco_slam_nn_nav.nav import NavConfig, Navigation  # noqa: E402
from rov_mujoco_slam_nn_nav.sim import ROVSim  # noqa: E402

SCENES = ("pool", "pipe_field", "trench")
POLICIES = ("classical", "neural", "e2e")
GOALS = {"warehouse": (2.6, 0.0), "obstacle_field": (2.5, 0.0), "corridor": (2.2, 0.0)}


def main():
    cfg = NavConfig()
    out = {}
    for policy in POLICIES:
        for scene in SCENES:
            t0 = time.time()
            sim = ROVSim(scene=scene)
            nav = Navigation(sim, cfg, policy=policy, with_depth=(policy == "e2e"))
            r = nav.run(max_time=90)
            r["wall_time_s"] = round(time.time() - t0, 1)
            out[f"{policy}/{scene}"] = r
            print(policy, scene, json.dumps(r, ensure_ascii=False), flush=True)
    os.makedirs("results", exist_ok=True)
    with open("results/benchmark.json", "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()

