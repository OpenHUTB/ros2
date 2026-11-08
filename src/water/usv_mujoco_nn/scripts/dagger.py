"""DAgger 数据聚合：在策略自身访问到的状态上向专家查询动作标签。

行为克隆只见过专家轨迹上的状态，一旦偏离就进入分布外区域并迅速崩溃
（复合误差）。DAgger 让当前策略驾驶，同时记录专家在这些新状态下的
动作，把这些样本并入训练集后重训，可显著提高闭环成功率。
"""

import argparse
import os

import numpy as np

from sim_core import USVSim, ANGLE_MIN, ANGLE_MAX, MAX_RANGE, thrust_from_twist
from occupancy import OccupancyMap
from oracle import GridPlanner
from nn import MLP

CROP = 21


class Pilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "planner_mlp.npz"))
        z = np.load(os.path.join(model_dir, "norm_stats.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = None
        self.w = None

    def act(self, feat):
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], 0.0, 2.0))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2))
        a = 0.45
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


def collect(model_dir, episodes, seed, max_steps):
    sim = USVSim(seed=seed)
    planner = GridPlanner(sim.obstacles)
    pilot = Pilot(model_dir)
    X, Y = [], []

    for ep in range(episodes):
        pilot.v = pilot.w = None
        obs = sim.reset(randomize=True)
        occ = OccupancyMap()
        path = planner.plan(sim.start[:2], sim.goal)
        if path is None:
            continue

        for step in range(max_steps):
            pose = sim.pose()
            occ.update(pose, obs["scan"], ANGLE_MIN, ANGLE_MAX, MAX_RANGE)

            feat = np.concatenate([occ.crop(pose, CROP).reshape(-1),
                                   [obs["goal_body"][0], obs["goal_body"][1], obs["goal_dist"]]])
            # 标签来自专家：船体系下指向前视点的向量（归一化 5 m）
            pt = planner.lookahead_point(pose, path)
            dx, dy = pt[0] - pose[0], pt[1] - pose[1]
            cy, sy = np.cos(-pose[2]), np.sin(-pose[2])
            X.append(feat)
            Y.append([(cy * dx - sy * dy) / 5.0, (sy * dx + cy * dy) / 5.0])

            # 执行来自策略
            v_p, w_p = pilot.act(feat)
            tl, tr = thrust_from_twist(v_p, w_p)
            obs = sim.step(tl, tr)

            if sim.collided or obs["goal_dist"] < 1.5:
                break

    return np.asarray(X, dtype=np.float64), np.asarray(Y, dtype=np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=25)
    ap.add_argument("--steps", type=int, default=320)
    ap.add_argument("--seed", type=int, default=99)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--out", type=str, default="dagger_data.npz")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(here, "..", args.model_dir)
    X, Y = collect(model_dir, args.episodes, args.seed, args.steps)
    out = os.path.join(here, "..", args.out)
    np.savez(out, X=X, Y=Y)
    print("DAgger samples: %d -> %s" % (X.shape[0], out))


if __name__ == "__main__":
    main()
