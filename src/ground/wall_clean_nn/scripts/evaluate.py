#!/usr/bin/env python3
"""闭环评测：沿墙专家 vs 神经网络。"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _pkg_paths():
    """返回 (scripts_dir, models_dir)。

    源码空间用本文件目录；安装空间（rosrun 调用 lib/<包>/ 下的副本）经 rospkg
    定位 share/<包>，否则 from sim_core import 会失败、模型目录也会指到 lib/ 下。
    """
    scripts = HERE
    models = os.path.abspath(os.path.join(HERE, "..", "models"))
    try:
        import rospkg
        pkg = rospkg.RosPack().get_path("wall_clean_nn")
        cand = os.path.join(pkg, "scripts")
        if os.path.isdir(cand):
            scripts = cand
            models = os.path.join(pkg, "models")
    except Exception:
        pass
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    return scripts, models


SCRIPTS_DIR, MODELS_DIR = _pkg_paths()

from sim_core import CoverSim, thrust_from_twist                      # noqa: E402
from oracle import build_wall_path, WallExpert, sample_start           # noqa: E402
from nn import MLP                                                    # noqa: E402


def features(sim, obs, cov_size=11):
    cov = sim.local_coverage(size=cov_size).reshape(-1)
    return np.concatenate([obs["scan"], cov, [obs["speed"]]]).astype(np.float64)


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "wall_planner.npz"))
        z = np.load(os.path.join(model_dir, "wall_norm.npz"))
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


COV_TARGET = 0.95        # 沿墙覆盖判据（专家可达上限，见文档"评价指标"）
EVAL_SEED_START = 100    # 评测 seeds 100.. 与训练 seeds（0,1000,2000,...）不相交
WALL_OFFSET = 0.85       # 期望贴墙距离 (m)


def run(policy, seed, steps=9000, cov_target=COV_TARGET):
    """跑一个回合。

    success 的判据是「无碰撞且沿墙覆盖率 >= cov_target」，而不是「没撞就算成功」，
    否则超时结束（覆盖率不足）会被误记为成功。
    """
    rng = np.random.default_rng(seed)
    sim = CoverSim(seed=seed)
    start = sample_start(rng, sim.obstacles)
    obs = sim.reset(start)
    path, _ = build_wall_path(sim.obstacles, start[:2])
    expert = WallExpert(path) if policy is None else None
    if hasattr(policy, "reset"):
        policy.reset()

    errs = []
    reason = "timeout"
    k = -1
    for k in range(steps):
        if policy is None:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
        else:
            v, w = policy.act(features(sim, obs))
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        errs.append(abs(sim.wall_error() - WALL_OFFSET))
        if sim.collided:
            reason = "collision"
            break
        if expert is not None and expert.k >= len(path) - 1 and sim.speed() < 0.05:
            reason = "finished"       # 专家走完路径并停稳
            break
        if sim.wall_coverage() >= cov_target:
            reason = "covered"        # 覆盖率达标
            break

    cov = sim.wall_coverage()
    success = int((not sim.collided) and cov >= cov_target and reason != "timeout")
    return {"success": success, "collision": int(sim.collided), "reason": reason,
            "seed": int(seed), "steps": k + 1, "wall_coverage": float(cov),
            "wall_error": float(np.mean(errs)) if errs else 0.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--seed-start", type=int, default=EVAL_SEED_START,
                    help="评测起始种子（默认 %d，与训练种子错开）" % EVAL_SEED_START)
    ap.add_argument("--cov-target", type=float, default=COV_TARGET,
                    help="沿墙覆盖率判据（默认 %.2f）" % COV_TARGET)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--out", type=str, default="results")
    args = ap.parse_args()

    model_dir = MODELS_DIR if args.model_dir == "models" else args.model_dir
    outdir = os.path.join(HERE, "..", args.out)
    os.makedirs(outdir, exist_ok=True)

    seeds = [args.seed_start + i for i in range(args.seeds)]
    report = {}
    for name, mk in (("expert", lambda: None), ("nn", lambda: NNPilot(model_dir))):
        res = [run(mk(), s, cov_target=args.cov_target) for s in seeds]
        reasons = {}
        for r in res:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        report[name] = {
            "episodes": len(res),
            "seeds": seeds,
            "cov_target": args.cov_target,
            "success_rate": float(np.mean([r["success"] for r in res])),
            "collision_rate": float(np.mean([r["collision"] for r in res])),
            "avg_steps": float(np.mean([r["steps"] for r in res])),
            "wall_coverage": float(np.mean([r["wall_coverage"] for r in res])),
            "wall_error": float(np.mean([r["wall_error"] for r in res])),
            "reasons": reasons,
        }
        print("%-7s 达标 %.0f%%  碰撞 %.0f%%  平均步数 %.0f  沿墙覆盖 %.1f%%  贴墙误差 %.3f m  结束原因 %s" % (
            name, 100 * report[name]["success_rate"], 100 * report[name]["collision_rate"],
            report[name]["avg_steps"], 100 * report[name]["wall_coverage"],
            report[name]["wall_error"], reasons))

    with open(os.path.join(outdir, "wall_eval.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print("-> %s" % os.path.join(outdir, "wall_eval.json"))


if __name__ == "__main__":
    main()