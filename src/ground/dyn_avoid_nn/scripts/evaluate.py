"""闭环评测：移动障碍避让，专家 vs 神经网络。"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import DynSim, GOAL, thrust_from_twist                # noqa: E402
from oracle import build_path, DynExpert                            # noqa: E402
from nn import MLP                                                  # noqa: E402


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "dyn_planner.npz"))
        z = np.load(os.path.join(model_dir, "dyn_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def reset(self):
        self.v = self.w = None

    def act(self, feat):
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], -0.3, 1.0))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2))
        a = 0.5
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


def features(sim, obs, prev_scan):
    return np.concatenate([obs["scan"], prev_scan,
                           [sim.goal_bearing(), min(sim.goal_distance(), 20.0),
                            obs["speed"]]]).astype(np.float64)


def run(policy, seed, steps=1500):
    rng = np.random.default_rng(seed)
    sim = DynSim(seed=seed)
    obs = sim.reset(None, rng)
    path = build_path(sim.obstacles, sim.pose()[:2], GOAL)
    expert = DynExpert(path) if policy is None else None
    if hasattr(policy, "reset"):
        policy.reset()
    prev = obs["scan"].copy()

    for k in range(steps):
        if policy is None:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["movers"], obs["scan"])
        else:
            v, w = policy.act(features(sim, obs, prev))
        prev = obs["scan"].copy()
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        if sim.collided:
            return {"success": 0, "collision": 1, "steps": k, "goal_dist": sim.goal_distance()}
        if sim.reached():
            return {"success": 1, "collision": 0, "steps": k + 1, "goal_dist": sim.goal_distance()}
    return {"success": 0, "collision": 0, "steps": steps, "goal_dist": sim.goal_distance()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--out", type=str, default="results")
    args = ap.parse_args()

    model_dir = os.path.join(HERE, "..", args.model_dir)
    outdir = os.path.join(HERE, "..", args.out)
    os.makedirs(outdir, exist_ok=True)

    report = {}
    for name, mk in (("expert", lambda: None), ("nn", lambda: NNPilot(model_dir))):
        res = [run(mk(), s) for s in range(args.seeds)]
        report[name] = {
            "episodes": len(res),
            "success_rate": float(np.mean([r["success"] for r in res])),
            "collision_rate": float(np.mean([r["collision"] for r in res])),
            "avg_steps": float(np.mean([r["steps"] for r in res])),
            "avg_goal_dist": float(np.mean([r["goal_dist"] for r in res])),
        }
        print("%-7s 成功率 %.0f%%  碰撞 %.0f%%  平均步数 %.0f  末距 %.2f m" % (
            name, 100 * report[name]["success_rate"], 100 * report[name]["collision_rate"],
            report[name]["avg_steps"], report[name]["avg_goal_dist"]))

    with open(os.path.join(outdir, "dyn_eval.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print("-> results/dyn_eval.json")


if __name__ == "__main__":
    main()
