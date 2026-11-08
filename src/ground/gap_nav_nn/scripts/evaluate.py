"""闭环评测：狭窄通道通行，专家 vs 神经网络。"""

import argparse, json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import GapSim, thrust_from_twist        # noqa: E402
from oracle import build_path, GapExpert, choose_gap   # noqa: E402
from nn import MLP                                     # noqa: E402
from pilot import NNPilot, features                    # noqa: E402



def run(policy, seed, steps=1200):
    rng = np.random.default_rng(seed)
    sim = GapSim(seed=seed)
    obs = sim.reset(None, None, rng)
    path, _ = build_path(sim.start, sim.goal)
    expert = GapExpert(path) if policy is None else None
    if hasattr(policy, "reset"):
        policy.reset()
    for k in range(steps):
        if policy is None:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
        else:
            v, w = policy.act(features(sim, obs))
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

    with open(os.path.join(outdir, "gap_eval.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print("-> results/gap_eval.json")


if __name__ == "__main__":
    main()
