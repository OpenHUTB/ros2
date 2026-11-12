#!/usr/bin/env python3
"""靠泊评测：专家 vs 神经网络。"""

import argparse
import json
import os
import sys

import numpy as np

def _ensure_scripts_on_path():
    """把运行期模块目录放进 sys.path。

    源码空间就是本文件所在目录；安装空间（rosrun/roslaunch 调用 lib/<包>/ 下的
    副本）则通过 rospkg 定位到 share/<包>/scripts，否则 from sim_core import 会失败。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        import rospkg
        cand = os.path.join(rospkg.RosPack().get_path("usv_docking_nn"), "scripts")
        if os.path.isdir(cand):
            here = cand
    except Exception:
        pass
    if here not in sys.path:
        sys.path.insert(0, here)


_ensure_scripts_on_path()

def _default_model_dir():
    """模型目录：源码空间取 <包>/models；安装空间经 rospkg 取 share/<包>/models。"""
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        import rospkg
        cand = os.path.join(rospkg.RosPack().get_path("usv_docking_nn"), "models")
        if os.path.isdir(cand):
            return cand
    except Exception:
        pass
    return os.path.abspath(os.path.join(here, "..", "models"))
from sim_core import DockSim, thrust_from_twist, TOL_Y, TOL_YAW, TOL_V
from oracle import DockExpert, build_path
from nn import MLP


# 评测 seeds 与训练 seeds（0, 1000, 2000, ...）刻意错开，
# 避免测试起点出现在训练数据里。
EVAL_SEED_START = 100


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "dock_planner.npz"))
        z = np.load(os.path.join(model_dir, "dock_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def reset(self):
        self.v = self.w = None

    def act(self, obs):
        e = obs["berth"]
        feat = np.concatenate([obs["scan"],
                               [np.clip(e[0], -3, 3), np.clip(e[1], -1, 1),
                                np.clip(e[2], -20, 20), obs["speed"]]])
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], 0.0, 1.5))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -0.8, 0.8))
        a = 0.5
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


def run(policy, seed, steps=1100):
    sim = DockSim(seed=seed)
    obs = sim.reset(seed=seed)
    if hasattr(policy, "reset"):
        policy.reset()
    expert = DockExpert(build_path(sim.start)) if policy is None else None
    for k in range(steps):
        if sim.collided:
            return {"success": 0, "collision": 1, "steps": k,
                    "y": abs(obs["berth"][0]), "yaw": abs(obs["berth"][1])}
        if sim.docked():
            return {"success": 1, "collision": 0, "steps": k,
                    "y": abs(obs["berth"][0]), "yaw": abs(obs["berth"][1])}
        if policy is None:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate())
        else:
            v, w = policy.act(obs)
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
    return {"success": int(sim.docked()), "collision": 0, "steps": steps,
            "y": abs(obs["berth"][0]), "yaw": abs(obs["berth"][1])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--out", type=str, default="results")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = _default_model_dir() if args.model_dir == "models" else args.model_dir
    outdir = os.path.join(here, "..", args.out)
    os.makedirs(outdir, exist_ok=True)

    report = {}
    for name, mk in (("expert", lambda: None), ("nn", lambda: NNPilot(model_dir))):
        recs = [run(mk(), EVAL_SEED_START + i) for i in range(args.seeds)]
        report[name] = {
            "episodes": len(recs),
            "success_rate": round(float(np.mean([r["success"] for r in recs])), 4),
            "collision_rate": round(float(np.mean([r["collision"] for r in recs])), 4),
            "avg_steps": round(float(np.mean([r["steps"] for r in recs])), 1),
            "final_y": round(float(np.mean([r["y"] for r in recs])), 4),
            "final_yaw": round(float(np.mean([r["yaw"] for r in recs])), 4),
        }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    with open(os.path.join(outdir, "dock_eval.json"), "w") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
