#!/usr/bin/env python3
"""训练：靠泊专家示范 + 行为克隆。

特征 = 激光雷达 91 束 + 泊位相对偏差 (横向 y, 艏向 yaw, 纵向距离) + 当前速度
标签 = 专家输出的 (v, w)
"""

import argparse
import os
import sys
import time

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
from sim_core import DockSim, thrust_from_twist
from oracle import DockExpert, build_path
from nn import MLP


# 训练起点种子：0, 1000, 2000, ...（与评测 seeds 100–119 不相交）
TRAIN_SEED_BASE = 0
TRAIN_SEED_STRIDE = 1000


def features(obs):
    e = obs["berth"]
    return np.concatenate([obs["scan"],
                           [np.clip(e[0], -3, 3), np.clip(e[1], -1, 1),
                            np.clip(e[2], -20, 20), obs["speed"]]])


def gen(episodes, seed, verbose=True):
    sim = DockSim(seed=seed)

    X, Y = [], []
    t0 = time.time()
    for ep in range(episodes):
        obs = sim.reset(seed=TRAIN_SEED_BASE + TRAIN_SEED_STRIDE * ep + seed)
        expert = DockExpert(build_path(sim.start))
        for step in range(1100):
            e = obs["berth"]
            if e[2] < 0.4 and abs(e[0]) < 0.3 and sim.speed() < 0.2:
                break
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate())
            X.append(features(obs))
            Y.append([v, w])
            tl, tr = thrust_from_twist(v, w)
            obs = sim.step(tl, tr)
            if sim.collided:
                break
        if verbose and (ep + 1) % 10 == 0:
            print("  episode %d/%d samples=%d %.1fs" % (ep + 1, episodes, len(X), time.time() - t0))
    return np.asarray(X), np.asarray(Y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=str, default="models")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(here, "..", args.out)
    os.makedirs(outdir, exist_ok=True)

    print("[1/3] 采集示范 ...")
    X, Y = gen(args.episodes, args.seed)
    print("      样本 %d 特征 %d" % X.shape)
    if len(X) < 200:
        raise SystemExit("样本太少，检查专家是否过早结束")

    xm, xs = X.mean(0), X.std(0)
    xs[xs < 1e-6] = 1.0
    ym, ys = Y.mean(0), Y.std(0)
    ys[ys < 1e-6] = 1.0
    Xn, Yn = (X - xm) / xs, (Y - ym) / ys

    n = len(Xn)
    idx = np.random.default_rng(args.seed).permutation(n)
    tr, va = idx[: int(n * 0.85)], idx[int(n * 0.85):]

    print("[2/3] 训练 MLP ...")
    net = MLP([Xn.shape[1], 128, 128, 2], seed=args.seed, lr=1.2e-3, l2=1e-5)
    hist = net.fit(Xn[tr], Yn[tr], epochs=args.epochs, batch_size=256)
    mse = float(np.mean((net.predict(Xn[va]) - Yn[va]) ** 2))
    print("      验证 MSE = %.6f" % mse)

    print("[3/3] 保存 ...")
    net.save(os.path.join(outdir, "dock_planner.npz"))
    np.savez(os.path.join(outdir, "dock_norm.npz"), xm=xm, xs=xs, ym=ym, ys=ys)
    np.save(os.path.join(outdir, "dock_loss.npy"), np.asarray(hist))
    with open(os.path.join(outdir, "dock_report.txt"), "w") as f:
        f.write("samples=%d\nfeatures=%d\nval_mse=%.6f\n" % (n, X.shape[1], mse))
    print("      完成 ->", outdir)


if __name__ == "__main__":
    main()
