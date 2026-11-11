#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""行为克隆：采集 PD 控制器的输入输出对，训练 MLP，再闭环验证。

流程：
    ① 用 PD 控制器跑若干段随机工况，记录 (状态, 目标速度, 力矩)
    ② 训练 MLP 拟合这些样本
    ③ **闭环**用 MLP 控制 20 s，看它能不能自己站住、能不能跟上速度指令

第 ③ 步是关键：只报"训练集 MSE 小"没有意义，必须闭环不倒才算学会。
行为克隆会遇到分布偏移（MLP 走的路和 PD 不一样），所以采集时
刻意加了随机初始倾角与随机速度指令，让训练分布更宽。

用法：
    python3 tools/train_policy.py                   # 全流程
    python3 tools/train_policy.py --episodes 30 --epochs 800
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

from balance_bot import config                                        # noqa: E402
from balance_bot.control import BalanceController                     # noqa: E402
from balance_bot.env import BalanceBotEnv, describe_env               # noqa: E402
from balance_bot.policy_nn import PolicyMLP, StandardScaler, state_features   # noqa: E402


def collect(env, ctrl, episodes=30, seconds=8.0, seed=0):
    """用 PD 控制器采集样本。工况随机化，避免 MLP 只见过一种轨迹。"""
    rng = np.random.RandomState(seed)
    X, Y = [], []
    for ep in range(episodes):
        env.reset(tilt=float(rng.uniform(-0.05, 0.05)))
        # 每段随机一个目标速度，有时保持 0，有时来回变
        # 目标速度**每段固定不变**。早期版本每 2 s 换一次，结果网络学到的
        # v_target 一直在跳变，闭环跑 0.30 m/s 时 4.9 s 就倒了（v 冲到 0.654）。
        # 固定后网络才能学到"v_target → 稳态倾角"的映射。
        v_cmd = float(rng.choice([0.0, 0.0, 0.15, -0.15, 0.30, -0.30, 0.30]))
        for k in range(int(seconds * config.CONTROL_HZ)):
            st = env.state()
            tl, tr = ctrl(st, v_cmd, 0.0)
            X.append(state_features(st, v_cmd))
            Y.append([0.5 * (tl + tr)])       # 直行时左右同值，只学这一个
            env.set_torque(tl, tr)
            env.step()
            if not env.is_upright():
                break
    return np.array(X), np.array(Y)


def train(X, Y, epochs=800, lr=3e-3, seed=0, verbose=True):
    rng = np.random.RandomState(seed)
    scaler = StandardScaler().fit(X)
    xs = scaler.transform(X)
    idx = rng.permutation(len(xs))
    n_val = max(1, int(0.15 * len(xs)))
    val, tr = idx[:n_val], idx[n_val:]
    net = PolicyMLP(seed=seed)
    hist = []
    for ep in range(epochs):
        p = rng.permutation(tr)
        for i in range(0, len(p), 64):
            net.backward(xs[p[i:i + 64]], Y[p[i:i + 64]], lr=lr)
        if ep % 200 == 0 or ep == epochs - 1:
            v = float(np.mean((net.predict(xs[val]) - Y[val]) ** 2))
            hist.append((ep, v))
            if verbose:
                print("   epoch %3d  val MSE %.5f" % (ep, v))
    return net, scaler, hist


def closed_loop(env, net, scaler, v_target=0.0, seconds=20.0, tilt=0.02):
    """用 MLP 闭环控制，返回 (是否站住, 最大倾角, 末速度, 末位置)。"""
    env.reset(tilt=tilt)
    worst = 0.0
    st = env.state()
    for _ in range(int(seconds * config.CONTROL_HZ)):
        st = env.state()
        tau = float(net.predict(scaler.transform(
            state_features(st, v_target).reshape(1, -1)))[0, 0])
        tau = max(-config.TORQUE_MAX, min(config.TORQUE_MAX, tau))
        env.set_torque(tau, tau)
        st = env.step()
        worst = max(worst, abs(st.theta))
        if not env.is_upright():
            return False, worst, st.v, st.x, st.t
    return True, worst, st.v, st.x, st.t


def main(argv=None):
    ap = argparse.ArgumentParser(description="平衡策略行为克隆")
    ap.add_argument("--episodes", type=int, default=25)
    ap.add_argument("--epochs", type=int, default=800)
    ap.add_argument("--scene", default=None)
    args = ap.parse_args(argv)

    print("[环境] %s" % describe_env())
    env = BalanceBotEnv(config.resolve_scene_path(args.scene))
    ctrl = BalanceController()

    print("\n① 采集（%d 段，随机初始倾角与目标速度）" % args.episodes)
    X, Y = collect(env, ctrl, episodes=args.episodes)
    print("   样本 %d 条，输入维度 %d，力矩均值 %.3f N·m" % (len(X), X.shape[1], Y.mean()))

    print("\n② 训练 MLP（4→16→16→1）")
    net, scaler, hist = train(X, Y, epochs=args.epochs)

    out_dir = config.DEFAULT_OUTPUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    net.save(str(out_dir / "balance_policy.npz"))
    scaler.save(str(out_dir / "balance_scaler.npz"))

    print("\n③ 闭环验证（MLP 自己控制，20 s）")
    rows = []
    for name, v_t in (("静止站立", 0.0), ("前进 0.30 m/s", 0.30)):
        ok, worst, v, x, t = closed_loop(env, net, scaler, v_target=v_t)
        print("   %-14s %s  最大倾角 %5.2f°  末速度 %+.3f m/s  末位置 %+.2f m  用时 %.1f s"
              % (name, "✓ 站住" if ok else "✗ 倒了", math.degrees(worst), v, x, t))
        rows.append(dict(case=name, ok=int(ok), worst_deg=round(math.degrees(worst), 2),
                         v=round(v, 3), x=round(x, 3)))

    print("\n④ 对照（PD 控制器，同样 20 s）")
    pd_rows = []
    for name, v_t in (("静止站立", 0.0), ("前进 0.30 m/s", 0.30)):
        env.reset(tilt=0.02)
        worst = 0.0
        for _ in range(int(20 * config.CONTROL_HZ)):
            tl, tr = ctrl(env.state(), v_t, 0.0)
            env.set_torque(tl, tr)
            st = env.step()
            worst = max(worst, abs(st.theta))
            if not env.is_upright():
                break
        print("   %-14s %s  最大倾角 %5.2f°  末速度 %+.3f m/s"
              % (name, "✓ 站住" if env.is_upright() else "✗ 倒了",
                 math.degrees(worst), st.v))
        pd_rows.append(dict(case=name, ok=int(env.is_upright()),
                            worst_deg=round(math.degrees(worst), 2), v=round(st.v, 3)))

    metrics = dict(nn=rows, pd=pd_rows, samples=int(len(X)),
                   train_hist=[[int(e), float(v)] for e, v in hist])
    with open(out_dir / "policy.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print("\n[导出] %s/balance_policy.npz, policy.json" % out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
