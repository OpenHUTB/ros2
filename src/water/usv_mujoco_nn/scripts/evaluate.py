"""闭环性能评测：神经网络策略 vs. A*+纯跟踪专家 基线。

指标：
    成功率        到达目标（距目标 < 1.5 m）
    碰撞率        撞到障碍物或边界
    路径比        实际航程 / 起终点直线距离（越小越优）
    平均航时      到达目标所用仿真步数
    建图覆盖率    结束时已观测栅格占比
"""

import argparse
import json
import os

import numpy as np

from sim_core import USVSim, ANGLE_MIN, ANGLE_MAX, MAX_RANGE, thrust_from_twist
from occupancy import OccupancyMap
from oracle import GridPlanner
from policy import NNPolicy

CROP = 21


def run_episode(sim, planner, occ, policy=None, max_steps=800):
    obs = sim.reset(randomize=True)
    pose0 = sim.pose()
    start = np.array(pose0[:2])
    goal = sim.goal.copy()
    path = planner.plan(start, goal)

    travelled = 0.0
    prev = start.copy()
    if policy is not None:
        policy.reset()

    for step in range(max_steps):
        pose = sim.pose()
        occ.update(pose, obs["scan"], ANGLE_MIN, ANGLE_MAX, MAX_RANGE)

        if policy is None:
            v, w = planner.pure_pursuit(pose, path)
        else:
            feat = np.concatenate([occ.crop(pose, CROP).reshape(-1),
                                   [obs["goal_body"][0], obs["goal_body"][1], obs["goal_dist"]]])
            v, w = policy.act(feat, obs["scan"])

        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)

        cur = np.array(sim.pose()[:2])
        travelled += float(np.linalg.norm(cur - prev))
        prev = cur

        if sim.collided:
            return {"success": 0, "collision": 1, "steps": step + 1,
                    "path_len": travelled, "coverage": occ.covered_ratio()}
        if obs["goal_dist"] < 1.5:
            straight = float(np.linalg.norm(goal - start))
            return {"success": 1, "collision": 0, "steps": step + 1,
                    "path_len": travelled, "coverage": occ.covered_ratio(),
                    "straight": straight}

    return {"success": 0, "collision": 0, "steps": max_steps,
            "path_len": travelled, "coverage": occ.covered_ratio()}


def summarize(records, name):
    n = len(records)
    succ = np.mean([r["success"] for r in records])
    coll = np.mean([r["collision"] for r in records])
    steps = np.mean([r["steps"] for r in records])
    cov = np.mean([r["coverage"] for r in records])
    ratios = [r["path_len"] / r["straight"] for r in records if r.get("straight", 0) > 1e-6]
    ratio = float(np.mean(ratios)) if ratios else float("nan")
    return {"name": name, "episodes": n, "success_rate": round(float(succ), 4),
            "collision_rate": round(float(coll), 4), "avg_steps": round(float(steps), 1),
            "avg_coverage": round(float(cov), 4), "path_ratio": round(ratio, 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--out", type=str, default="results")
    ap.add_argument("--skip-expert", action="store_true")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(here, "..", args.model_dir)
    outdir = os.path.join(here, "..", args.out)
    os.makedirs(outdir, exist_ok=True)

    sim = USVSim(seed=args.seed)
    planner = GridPlanner(sim.obstacles)
    policy = NNPolicy(model_dir)

    expert, learned = [], []
    # 用同一组随机种子，保证两条策略面对相同的起点/终点
    for k in range(args.episodes):
        if not args.skip_expert:
            sim.rng = np.random.default_rng(args.seed * 1000 + k)
            expert.append(run_episode(sim, planner, OccupancyMap(), policy=None))
        sim.rng = np.random.default_rng(args.seed * 1000 + k)
        learned.append(run_episode(sim, planner, OccupancyMap(), policy=policy))

    report = {}
    if expert:
        report["expert"] = summarize(expert, "A*+PurePursuit (expert)")
    report["learned"] = summarize(learned, "NN policy (deployed)")

    print(json.dumps(report, ensure_ascii=False, indent=2))
    with open(os.path.join(outdir, "eval_report.json"), "w") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
