"""行为克隆：移动障碍避让。

特征 = 当前 91 束雷达 + 上一帧 91 束雷达 + 目标方位 + 目标距离 + 速度
（两帧雷达让网络能自己看出障碍在动，这是本任务的"端到端"含义）
标签 = 专家的 (v, w)
"""

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import DynSim, GOAL, thrust_from_twist            # noqa: E402
from oracle import build_path, DynExpert                         # noqa: E402
from nn import MLP                                               # noqa: E402


def features(sim, obs, prev_scan):
    return np.concatenate([obs["scan"], prev_scan,
                           [sim.goal_bearing(), min(sim.goal_distance(), 20.0),
                            obs["speed"]]]).astype(np.float64)


def rollout(sim, rng, steps, fps=None):
    obs = sim.reset(None, rng)
    path = build_path(sim.obstacles, sim.pose()[:2], GOAL)
    ex = DynExpert(path)
    prev = obs["scan"].copy()
    out = []
    for _ in range(steps):
        v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["movers"], obs["scan"])
        if fps is not None:
            fps.append((features(sim, obs, prev), v, w))
        prev = obs["scan"].copy()
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        if sim.collided or sim.reached():
            break
    return obs, path


def generate(sim, episodes, steps, seed=0, noise=0.0, verbose=True):
    X, Y = [], []
    for ep in range(episodes):
        rng = np.random.default_rng(seed * 1000 + ep)
        obs = sim.reset(None, rng)
        path = build_path(sim.obstacles, sim.pose()[:2], GOAL)
        ex = DynExpert(path)
        prev = obs["scan"].copy()
        for t in range(steps):
            feat = features(sim, obs, prev)
            v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["movers"], obs["scan"])
            X.append(feat)
            Y.append([v, w])
            if noise > 0:
                v = float(np.clip(v + rng.normal(0, noise), -0.3, 1.0))
                w = float(np.clip(w + rng.normal(0, noise), -1.2, 1.2))
            prev = obs["scan"].copy()
            tl, tr = thrust_from_twist(v, w)
            obs = sim.step(tl, tr)
            if sim.collided or sim.reached():
                break
        if verbose:
            print("  episode %2d  步数 %4d  %s" % (ep, t + 1, "到达" if sim.reached() else "中断"))
    return np.asarray(X), np.asarray(Y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--steps", type=int, default=700)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.06)
    args = ap.parse_args()

    model_dir = os.path.join(HERE, "..", "models")
    os.makedirs(model_dir, exist_ok=True)

    sim = DynSim(seed=args.seed)
    t0 = time.time()
    print("[1/3] 采集专家示范 ...")
    X, Y = generate(sim, args.episodes, args.steps, seed=args.seed, noise=args.noise)
    print("      样本 %d  特征 %d  用时 %.1f s" % (len(X), X.shape[1], time.time() - t0))

    xm, xs = X.mean(0), X.std(0) + 1e-8
    ym, ys = Y.mean(0), Y.std(0) + 1e-8
    Xn, Yn = (X - xm) / xs, (Y - ym) / ys

    idx = np.random.default_rng(0).permutation(len(Xn))
    split = int(0.9 * len(idx))
    tr, va = idx[:split], idx[split:]

    print("[2/3] 训练 MLP ...")
    net = MLP([Xn.shape[1], 128, 128, 2], seed=args.seed, lr=1.2e-3)
    hist = []
    for ep in range(args.epochs):
        net.fit(Xn[tr], Yn[tr], epochs=1, batch_size=256, verbose=False)
        mse = float(np.mean((net.predict(Xn[va]) - Yn[va]) ** 2))
        hist.append(mse)
        if ep % 10 == 0 or ep == args.epochs - 1:
            print("      epoch %3d  mse=%.6f" % (ep, mse))

    print("[3/3] 保存 ...")
    net.save(os.path.join(model_dir, "dyn_planner.npz"))
    np.savez(os.path.join(model_dir, "dyn_norm.npz"), xm=xm, xs=xs, ym=ym, ys=ys)
    np.save(os.path.join(model_dir, "dyn_loss.npy"), np.asarray(hist))
    with open(os.path.join(model_dir, "dyn_report.txt"), "w", encoding="utf-8") as f:
        f.write("samples=%d\nfeatures=%d\nval_mse=%.6f\n" % (len(X), X.shape[1], hist[-1]))
    print("      完成 -> %s" % model_dir)


if __name__ == "__main__":
    main()
