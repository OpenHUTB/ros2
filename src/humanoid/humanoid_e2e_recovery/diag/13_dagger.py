#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""13 DAgger 一轮（失败路线，保留作证据）：用混合策略采样、专家打标签、合并重训。

思路：行为克隆的误差累积（covariate shift）可以用 DAgger 缓解 ——
让网络自己跑出"它会到达的状态"，再用专家在这些状态上重标。
结果：5802 条数据（4272 原始 + 1530 新采），纯网络闭环 1.95 s，
比不做 DAgger 的 3.53 s 更差。

为什么失败（结论写在 09 里）：缺口是没有速度信息的控制本身，
DAgger 只补状态分布，不补信息，网络仍然学不出阻尼。

用法:
    python3 diag/13_dagger.py --dataset data/e2e_dataset.npz
    python3 diag/13_dagger.py --dataset data/e2e_dataset.npz --episodes 4 --seconds 5

实测(云主机): 采集 1530 条，合并后 5802 条，闭环 1.95 s 跌倒。
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (DATASET, DEFAULT_STAND_TARGET, MODEL_PATH, WEIGHTS,  # noqa: E402
                     load_xy, nn_policy, require, run_episode)

from humanoid_e2e_recovery.dataset import _img_to_vec, X_STD_FLOOR, TORQUE_LIMIT  # noqa: E402
from humanoid_e2e_recovery.train import train  # noqa: E402
from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402
from humanoid_e2e_recovery.expert import expert_action  # noqa: E402


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    p = argparse.ArgumentParser(description="DAgger 一轮")
    p.add_argument("--dataset", default=DATASET, help="原始数据集")
    p.add_argument("--weights", default=WEIGHTS, help="原始权重（DAgger 的初始策略）")
    p.add_argument("--episodes", type=int, default=6)
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--aug", default=os.path.join(root, "data", "diag_dagger_ds.npz"))
    p.add_argument("--out", default=os.path.join(root, "data", "diag_dagger_policy.npz"))
    a = p.parse_args()
    require(a.dataset, "先 python3 main.py --collect 生成数据集")
    os.makedirs(os.path.dirname(a.aug), exist_ok=True)

    pdt = E2EPredictor(a.weights)
    sim = E2ESim(MODEL_PATH)
    rng = np.random.default_rng(7)
    Xa, Ya = [], []

    t0 = time.time()
    for ep in range(a.episodes):
        beta = 0.5 if ep < a.episodes - 2 else 0.35
        sim.reset()
        for i in range(350):
            if i % 2 == 0:
                img = sim.render_front(gray=True)
                net_u = pdt.predict_from_image(img)
                exp_u = np.clip(expert_action(sim), -TORQUE_LIMIT, TORQUE_LIMIT)
                # 标签始终是专家力矩；状态由混合策略驱动，覆盖网络自己会到的分布
                Xa.append(_img_to_vec(img))
                Ya.append(exp_u.astype(np.float32))
                u = beta * exp_u + (1 - beta) * net_u
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
            if sim.fallen():
                break

        # 每回合再补一段突发推力，扩大偏离稳态的覆盖
        ang = rng.uniform(0, 2 * np.pi)
        f = rng.uniform(40, 100)
        sim.apply_push((f * np.cos(ang), f * np.sin(ang), 0), steps=40,
                       target=DEFAULT_STAND_TARGET, balance=expert_action)
        for i in range(160):
            if i % 2 == 0:
                img = sim.render_front(gray=True)
                net_u = pdt.predict_from_image(img)
                exp_u = np.clip(expert_action(sim), -TORQUE_LIMIT, TORQUE_LIMIT)
                Xa.append(_img_to_vec(img))
                Ya.append(exp_u.astype(np.float32))
                u = beta * exp_u + (1 - beta) * net_u
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
            if sim.fallen():
                break
    sim.close()
    print("[dagger] 采集 {} 条，用时 {:.0f}s".format(len(Ya), time.time() - t0))

    X0, Y0 = load_xy(a.dataset)
    X = np.concatenate([X0, np.stack(Xa)])
    Y = np.concatenate([Y0, np.stack(Ya)])
    X_mean = X.mean(0)
    X_std = np.maximum(X.std(0), X_STD_FLOOR)
    Y_mean = Y.mean(0)
    Y_scale = np.where(Y.std(0) > 1e-6, Y.std(0), 1.0)
    np.savez(a.aug, X=X, Y=Y, X_mean=X_mean, X_std=X_std,
             Y_mean=Y_mean, Y_scale=Y_scale)
    print("[dagger] 合并后 {} 条 -> {}".format(len(Y), a.aug))

    train(a.aug, a.out, epochs=a.epochs, verbose=False)
    p2 = E2EPredictor(a.out)
    print("[dagger] 训练完成 -> {}".format(a.out))

    sim = E2ESim(MODEL_PATH)
    sim.reset()
    fell, tilt = run_episode(sim, nn_policy(p2), a.seconds)
    print("\n[闭环] 纯网络（无任何专家介入）: {}，最大倾角 {:.1f}°".format(
        "跌倒于 {:.2f}s".format(fell) if fell else "站稳 {}s".format(a.seconds),
        np.degrees(tilt)))
    sim.close()


if __name__ == "__main__":
    main()
