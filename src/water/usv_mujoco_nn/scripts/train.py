"""数据生成 + 神经网络训练。

流程：
1. 随机起点/终点，A* 规划全局路径；
2. 纯跟踪控制器驾驶 USV，记录每一时刻的
   局部占据栅格(21x21)、目标相对位姿 -> 控制标签 (v, w)；
3. 标准化后用 Adam 训练 MLP，保存权重与归一化参数。
"""

import argparse
import os
import time

import numpy as np

from sim_core import USVSim, N_BEAMS, ANGLE_MIN, ANGLE_MAX, MAX_RANGE, thrust_from_twist
from occupancy import OccupancyMap
from oracle import GridPlanner
from nn import MLP

CROP = 21


def build_features(occ, pose):
    grid = occ.crop(pose, size=CROP).reshape(-1)
    return grid


def gen_dataset(episodes=60, max_steps=260, seed=0, verbose=True, noise=0.0, start_policy=None):
    """采集示范数据。

    noise > 0 时对执行动作注入高斯噪声（专家带扰动），
    使数据覆盖"偏离参考轨迹后如何纠偏"的状态，缓解行为克隆的复合误差。
    """
    sim = USVSim(seed=seed)
    planner = GridPlanner(sim.obstacles)
    rng = np.random.default_rng(seed + 1)

    X, Y = [], []
    t0 = time.time()

    for ep in range(episodes):
        obs = sim.reset(randomize=True)
        occ = OccupancyMap()
        path = planner.plan(sim.start[:2], sim.goal)
        if path is None:
            continue

        for step in range(max_steps):
            pose = sim.pose()
            occ.update(pose, obs["scan"], ANGLE_MIN, ANGLE_MAX, MAX_RANGE)

            v, w = planner.pure_pursuit(pose, path)
            feat = np.concatenate([build_features(occ, pose),
                                   [obs["goal_body"][0], obs["goal_body"][1], obs["goal_dist"]]])
            X.append(feat)

            # 标签：船体系下指向专家的前视点（单位：归一化 5 m）
            pt = planner.lookahead_point(pose, path)
            dx, dy = pt[0] - pose[0], pt[1] - pose[1]
            cy, sy = np.cos(-pose[2]), np.sin(-pose[2])
            Y.append([(cy * dx - sy * dy) / 5.0, (sy * dx + cy * dy) / 5.0])

            v_exec, w_exec = v, w
            if noise > 0.0:
                v_exec = float(np.clip(v + rng.normal(0.0, noise * 0.6), 0.0, 1.6))
                w_exec = float(np.clip(w + rng.normal(0.0, noise * 1.6), -1.2, 1.2))
            tl, tr = thrust_from_twist(v_exec, w_exec)
            obs = sim.step(tl, tr)

            if sim.collided:
                break
            if obs["goal_dist"] < 1.5:
                break

        if verbose and (ep + 1) % 10 == 0:
            print("  episode %d/%d  samples=%d  %.1fs" % (ep + 1, episodes, len(X), time.time() - t0))

    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    return X, Y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--steps", type=int, default=260)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--extra", type=str, default="")
    ap.add_argument("--out", type=str, default="models")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(here, "..", args.out)
    os.makedirs(outdir, exist_ok=True)

    print("[1/3] 生成专家示范数据 ...")
    X, Y = gen_dataset(args.episodes, args.steps, args.seed, noise=args.noise)
    if args.extra:
        z = np.load(os.path.join(here, "..", args.extra))
        X = np.concatenate([X, z["X"]], axis=0)
        Y = np.concatenate([Y, z["Y"]], axis=0)
        print("      并入 DAgger 样本 %d 条" % z["X"].shape[0])
    print("      样本数 = %d，特征维度 = %d" % (X.shape[0], X.shape[1]))

    # 标准化
    xm = X.mean(axis=0)
    xs = X.std(axis=0)
    xs[xs < 1e-6] = 1.0
    ym = Y.mean(axis=0)
    ys = Y.std(axis=0)
    ys[ys < 1e-6] = 1.0

    Xn = (X - xm) / xs
    Yn = (Y - ym) / ys

    n = Xn.shape[0]
    perm = np.random.default_rng(args.seed).permutation(n)
    n_train = int(n * 0.85)
    tr, va = perm[:n_train], perm[n_train:]

    print("[2/3] 训练 MLP ...")
    net = MLP([Xn.shape[1], 128, 128, 2], seed=args.seed, lr=1.2e-3, l2=1e-5)
    hist = net.fit(Xn[tr], Yn[tr], epochs=args.epochs, batch_size=256)

    pred = net.predict(Xn[va])
    mse = float(np.mean((pred - Yn[va]) ** 2))
    mae_v = float(np.mean(np.abs((pred[:, 0] * ys[0] + ym[0]) - (Yn[va][:, 0] * ys[0] + ym[0]))))
    mae_w = float(np.mean(np.abs((pred[:, 1] * ys[1] + ym[1]) - (Yn[va][:, 1] * ys[1] + ym[1]))))
    print("      验证集 MSE = %.6f | MAE(v) = %.4f m/s | MAE(w) = %.4f rad/s" % (mse, mae_v, mae_w))

    print("[3/3] 保存模型 ...")
    net.save(os.path.join(outdir, "planner_mlp.npz"))
    np.savez(os.path.join(outdir, "norm_stats.npz"), xm=xm, xs=xs, ym=ym, ys=ys,
             crop=np.array([CROP]))
    np.save(os.path.join(outdir, "train_loss.npy"), np.asarray(hist))
    with open(os.path.join(outdir, "train_report.txt"), "w") as f:
        f.write("samples=%d\nfeatures=%d\nval_mse=%.6f\nval_mae_v=%.4f\nval_mae_w=%.4f\n"
                % (n, X.shape[1], mse, mae_v, mae_w))
    print("      完成：%s" % outdir)


if __name__ == "__main__":
    main()
