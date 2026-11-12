#!/usr/bin/env python3
"""行为克隆：采集沿墙专家示范并训练 MLP。

特征 = 61 束激光雷达 + 11x11 已清扫栅格 + 当前速度
标签 = 专家的 (v, w)
"""

import argparse
import os
import sys
import time

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

from sim_core import CoverSim, thrust_from_twist, START           # noqa: E402
from oracle import build_wall_path, WallExpert, is_free, sample_start  # noqa: E402
from nn import MLP                                                # noqa: E402


def features(sim, obs, cov_size=11):
    cov = sim.local_coverage(size=cov_size).reshape(-1)
    return np.concatenate([obs["scan"], cov, [obs["speed"]]]).astype(np.float64)


# 训练起点种子 = TRAIN_SEED_BASE + seed*TRAIN_SEED_STRIDE + ep，即 0, 1000, 2000, ...
# 评测（evaluate.py）用 100.. 起始的独立种子段，两者不相交。
TRAIN_SEED_BASE = 0
TRAIN_SEED_STRIDE = 1000


def generate(sim, episodes, steps, seed=0, noise=0.0, verbose=True):
    X, Y = [], []
    for ep in range(episodes):
        rng = np.random.default_rng(TRAIN_SEED_BASE + seed * TRAIN_SEED_STRIDE + ep)
        start = sample_start(rng, sim.obstacles)
        path, _ = build_wall_path(sim.obstacles, start[:2])
        obs = sim.reset(start)
        ex = WallExpert(path)
        for t in range(steps):
            feat = features(sim, obs)
            v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
            X.append(feat)
            Y.append([v, w])
            if noise > 0:
                v = float(np.clip(v + rng.normal(0, noise), -0.3, 1.0))
                w = float(np.clip(w + rng.normal(0, noise), -1.2, 1.2))
            tl, tr = thrust_from_twist(v, w)
            obs = sim.step(tl, tr)
            if sim.collided or (ex.k >= len(path) - 1 and sim.speed() < 0.05):
                break
        if verbose:
            print("  episode %2d  步数 %4d  沿墙覆盖 %5.1f%%" % (ep, t + 1, 100 * sim.wall_coverage()))
    return np.asarray(X), np.asarray(Y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=24)
    ap.add_argument("--steps", type=int, default=1100)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.06)
    args = ap.parse_args()

    model_dir = MODELS_DIR
    os.makedirs(model_dir, exist_ok=True)

    sim = CoverSim(seed=args.seed)
    t0 = time.time()
    print("[1/3] 采集专家示范 ...")
    X, Y = generate(sim, args.episodes, args.steps, seed=args.seed, noise=args.noise)
    print("      样本 %d  特征 %d  用时 %.1f s" % (len(X), X.shape[1], time.time() - t0))

    xm, xs = X.mean(0), X.std(0) + 1e-8
    ym, ys = Y.mean(0), Y.std(0) + 1e-8
    Xn = (X - xm) / xs
    Yn = (Y - ym) / ys

    idx = np.random.default_rng(0).permutation(len(Xn))
    split = int(0.9 * len(idx))
    tr, va = idx[:split], idx[split:]

    print("[2/3] 训练 MLP ...")
    # 梯度已按 bs*outs 归一（见 nn.py），lr 相应放大一倍以保持等效步长
    net = MLP([Xn.shape[1], 128, 128, 2], seed=args.seed, lr=2.4e-3)
    hist = []
    for ep in range(args.epochs):
        net.fit(Xn[tr], Yn[tr], epochs=1, batch_size=256, verbose=False)
        pred = net.predict(Xn[va])
        mse = float(np.mean((pred - Yn[va]) ** 2))
        hist.append(mse)
        if ep % 5 == 0 or ep == args.epochs - 1:
            print("      epoch %3d  mse=%.6f" % (ep, mse))

    print("[3/3] 保存 ...")
    net.save(os.path.join(model_dir, "wall_planner.npz"))
    np.savez(os.path.join(model_dir, "wall_norm.npz"), xm=xm, xs=xs, ym=ym, ys=ys)
    np.save(os.path.join(model_dir, "wall_loss.npy"), np.asarray(hist))
    with open(os.path.join(model_dir, "wall_report.txt"), "w", encoding="utf-8") as f:
        f.write("samples=%d\nfeatures=%d\nval_mse=%.6f\n" % (len(X), X.shape[1], hist[-1]))
    print("      完成 -> %s" % model_dir)


if __name__ == "__main__":
    main()
