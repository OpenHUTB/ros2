"""行为克隆：狭窄通道通行。特征 = 雷达 + 目标方位/距离 + 速度。"""

import argparse, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import GapSim, thrust_from_twist       # noqa: E402
from oracle import build_path, GapExpert              # noqa: E402
from nn import MLP                                    # noqa: E402
from pilot import features                            # noqa: E402



def generate(sim, episodes, steps, seed=0, noise=0.0, verbose=True):
    X, Y = [], []
    for ep in range(episodes):
        rng = np.random.default_rng(seed * 1000 + ep)
        obs = sim.reset(None, None, rng)
        path, _ = build_path(sim.start, sim.goal)
        ex = GapExpert(path)
        for t in range(steps):
            v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
            X.append(features(sim, obs)); Y.append([v, w])
            if noise > 0:
                v = float(np.clip(v + rng.normal(0, noise), -0.3, 1.0))
                w = float(np.clip(w + rng.normal(0, noise), -1.2, 1.2))
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
    ap.add_argument("--steps", type=int, default=600)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.05)
    args = ap.parse_args()

    model_dir = os.path.join(HERE, "..", "models")
    os.makedirs(model_dir, exist_ok=True)
    sim = GapSim(seed=args.seed)
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
    net.save(os.path.join(model_dir, "gap_planner.npz"))
    np.savez(os.path.join(model_dir, "gap_norm.npz"), xm=xm, xs=xs, ym=ym, ys=ys)
    np.save(os.path.join(model_dir, "gap_loss.npy"), np.asarray(hist))
    with open(os.path.join(model_dir, "gap_report.txt"), "w", encoding="utf-8") as f:
        f.write("samples=%d\nfeatures=%d\nval_mse=%.6f\n" % (len(X), X.shape[1], hist[-1]))
    print("      完成 -> %s" % model_dir)


if __name__ == "__main__":
    main()
