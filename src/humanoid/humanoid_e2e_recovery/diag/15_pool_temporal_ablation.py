#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""15 改善尝试对比：池化分辨率与多帧时序，能不能把 3.5 s 上限推上去。

回答的问题：第 4 节说"单帧、无速度信息"是架构上限，那有没有便宜的补法——
把 16×16 池化换成 32×32？把两帧拼成速度线索？本脚本把这四条路线各跑一遍，
用同一次采集的数据做对照，给出闭环站稳时间。

用法:
    python3 diag/15_pool_temporal_ablation.py --episodes 8
    python3 diag/15_pool_temporal_ablation.py --reuse      # 复用已采数据集

实测(云主机, 8 回合 4 272 条):
    16 格基线         val 2.35 N·m² | 闭环 2.63 s 跌倒
    32 格池化         val 1.48 N·m² | 闭环 1.72 s 跌倒（离线更好看，闭环更差）
    双帧 6 ms         val 1.42 N·m² | 闭环 2.85 s 跌倒
    双帧 102 ms       val 2.09 N·m² | 闭环站满 10 s（这一档才入库）
结论：离线 val_mse 更好看不等于闭环能站，唯一起作用的是"时间差足够大"——
这正是当前入库权重改用双帧 102 ms 的依据。

注意：数据按 32 格采（同样 64×64 渲染，不增加耗时），16 格由 2×2 均值派生，
两者等价于直接 16 格池化，保证对照口径一致。
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import MODEL_PATH, ROOT, add_path_args, run_episode  # noqa: E402

sys.path.insert(0, ROOT)
from humanoid_e2e_recovery.dataset import TORQUE_LIMIT, X_STD_FLOOR, _img_to_vec  # noqa: E402
from humanoid_e2e_recovery.expert import expert_action  # noqa: E402
from humanoid_e2e_recovery.sim import DEFAULT_STAND_TARGET, E2ESim  # noqa: E402
from humanoid_e2e_recovery.train import train  # noqa: E402

POOL = 32
SIDE = 16                     # 训练都用 16 格（256 维），32 格只用 1024 维那一档
EP_LEN = 534                  # 每回合样本数（warmup 400 + 恢复段，record_every=3）
# 中间产物目录。用 tempfile 而不是 /tmp：Windows 下 /tmp 不可写，
# 本脚本要能在任意平台上复现这四档对照。
WORK = tempfile.mkdtemp(prefix="diag15_")


def work(name):
    return os.path.join(WORK, name)


def collect(model_path, out_path, n_episodes=8, seed=0):
    rng = np.random.default_rng(seed)
    sim = E2ESim(model_path)
    X, Y = [], []
    kept, t0 = 0, time.time()
    for ep in range(n_episodes):
        sim.reset()
        for k in range(1200):
            u0 = expert_action(sim)
            if k % 3 == 0:
                X.append(_img_to_vec(sim.render_front(gray=True), size=POOL))
                Y.append(np.clip(u0, -TORQUE_LIMIT, TORQUE_LIMIT).astype(np.float32))
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u0)
            if sim.fallen():
                break
        ang = rng.choice([0, 45, 90, 135, 180, 225, 270, 315])
        f = rng.uniform(40, 100)
        sim.apply_push((f * np.cos(np.radians(ang)), f * np.sin(np.radians(ang)), 0),
                       steps=40, target=DEFAULT_STAND_TARGET, balance=expert_action)
        ok = True
        for k in range(400):
            u = expert_action(sim)
            if k % 3 == 0:
                X.append(_img_to_vec(sim.render_front(gray=True), size=POOL))
                Y.append(np.clip(u, -TORQUE_LIMIT, TORQUE_LIMIT).astype(np.float32))
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
            if sim.fallen():
                ok = False
                break
        kept += 1 if ok else 0
        print(f"  [采集] {ep + 1}/{n_episodes} samples={len(X)} 站住={kept}", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    np.savez(out_path, X=np.stack(X), Y=np.stack(Y))
    sim.close()
    print(f"[采集] {len(Y)} 条 | 站住 {kept}/{n_episodes} | {time.time() - t0:.0f}s")
    return np.load(out_path)


def fit(X, Y, out_w, in_dim, seed=0):
    ds = work("_diag15_%d.npz" % in_dim)
    np.savez(ds, X=X, Y=Y,
             X_mean=X.mean(axis=0), X_std=np.maximum(X.std(axis=0), X_STD_FLOOR),
             Y_mean=Y.mean(axis=0),
             Y_scale=np.where(Y.std(axis=0) > 1e-6, Y.std(axis=0), 1.0))
    train(ds, out_w, in_dim=in_dim, seed=seed, verbose=False)
    return np.load(out_w)


def policy_from(w, pool, lag_renders=0):
    W1, b1, W2, b2, W3, b3 = w["W1"], w["b1"], w["W2"], w["b2"], w["W3"], w["b3"]
    ym, ys, xm, xs = w["Y_mean"], w["Y_scale"], w["X_mean"], w["X_std"]
    st = {"hist": [], "u": np.zeros(15)}

    def policy(sim, refresh):
        cur = _img_to_vec(sim.render_front(gray=True), size=pool).astype(np.float64)
        st["hist"].append(cur)
        if len(st["hist"]) > lag_renders + 1:
            st["hist"].pop(0)
        if refresh:
            x = np.concatenate([st["hist"][0], cur]) if lag_renders else cur
            x = (x - xm) / xs
            h1 = np.maximum(x @ W1 + b1, 0.0)
            h2 = np.maximum(h1 @ W2 + b2, 0.0)
            st["u"] = np.clip((h2 @ W3 + b3) * ys + ym, -TORQUE_LIMIT, TORQUE_LIMIT)
        return st["u"]
    return policy


def closed_loop(w, pool, lag_renders=0, seconds=10.0, render_every=2):
    sim = E2ESim(MODEL_PATH)
    pol = policy_from(w, pool, lag_renders)
    first = pol(sim, True)[3]
    expert0 = expert_action(sim)[3]
    fell, tilt = run_episode(sim, pol, seconds=seconds,
                             render_every=render_every, enable_render=True)
    sim.close()
    tout = f"跌倒 {fell:.2f}s" if fell is not None else f"站满 {seconds:.0f}s"
    print(f"   首帧 网络 {first:6.2f} / 专家 {expert0:5.2f} | 闭环 {tout}，"
          f"倾角 {np.degrees(tilt):.1f}°", flush=True)
    return first, fell


def main():
    p = argparse.ArgumentParser(description=__doc__)
    add_path_args(p, out=os.path.join(ROOT, "data", "diag_pool32_ds.npz"))
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--reuse", action="store_true", help="复用已采的 32 格数据集")
    a = p.parse_args()

    if a.reuse and os.path.exists(a.out):
        d = np.load(a.out)
        print(f"[采集] 复用 {a.out} {d['X'].shape}")
    else:
        d = collect(MODEL_PATH, a.out, n_episodes=a.episodes)
    X32, Y = d["X"].astype(np.float64), d["Y"].astype(np.float64)
    X16 = X32.reshape(len(X32), SIDE, 2, SIDE, 2).mean(axis=(2, 4)).reshape(len(X32), -1)
    print(f"[数据] {X32.shape} → 16 格 {X16.shape} | X 逐维 std "
          f"{X32.std(axis=0).mean():.5f} → {X16.std(axis=0).mean():.5f} | "
          f"踝 pitch {Y[:, 3].mean():.1f}/{Y[:, 3].std():.1f}")

    res = {}

    print(f"[工作目录] {WORK}（tempfile，跨平台可写）", flush=True)
    print("[A] 16 格基线", flush=True)
    w = fit(X16, Y, work("diag15_A.npz"), 256)
    res["A"] = closed_loop(w, 16)

    print("[B] 32 格池化", flush=True)
    w = fit(X32, Y, work("diag15_B.npz"), 1024)
    res["B"] = closed_loop(w, 32)

    keep = np.array([i for i in range(1, len(X16)) if i % EP_LEN != 0])
    print("[C] 双帧 6 ms", flush=True)
    w = fit(np.concatenate([X16[keep - 1], X16[keep]], axis=1), Y[keep],
            work("diag15_C.npz"), 512)
    res["C"] = closed_loop(w, 16, lag_renders=1, render_every=3)

    lag = 17                      # 17 × 6 ms ≈ 102 ms
    keep = np.array([i for i in range(lag, len(X16)) if i % EP_LEN >= lag])
    print("[D] 双帧 102 ms", flush=True)
    w = fit(np.concatenate([X16[keep - lag], X16[keep]], axis=1), Y[keep],
            work("diag15_D.npz"), 512)
    res["D"] = closed_loop(w, 16, lag_renders=25, render_every=2)

    print("\n=== 汇总 ===")
    for k, (first, fell) in res.items():
        t = f"{fell:.2f}s" if fell is not None else "站满"
        print(f"{k}: 首帧 {first:.2f} N·m | 闭环 {t}")
    print("对照：入库双帧权重闭环 10 s 站满（诊断 16 可离线复算留出集指标）")


if __name__ == "__main__":
    main()
