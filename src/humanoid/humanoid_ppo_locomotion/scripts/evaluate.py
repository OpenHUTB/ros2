#!/usr/bin/env python3
"""策略性能评价：速度跟踪误差、存活时长、跌倒率、推理延迟。

用法：
    python scripts/evaluate.py --weights weights/policy_g1.pt --episodes 10 --vx 0.0
    python scripts/evaluate.py --weights saved_models/stage1/ppo_g1.zip --episodes 10
输出 CSV 到 --csv（默认 saved_models/eval.csv），并在终端打印汇总。

``--weights`` 支持两种格式，按后缀自动判断：
    *.pt   TorchScript（仓库内自带，体积小，与 ROS 2 推理节点同一份文件）
    *.zip  Stable-Baselines3 存档（只有重训后才会产生）
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from humanoid_ppo_locomotion.g1_env import G1WalkEnv, EPISODE_STEPS  # noqa: E402


def load_actor(weights: str):
    """返回 ``obs -> action`` 的确定性策略函数，按后缀区分 TorchScript / SB3。"""
    if weights.endswith(".pt"):
        import torch

        module = torch.jit.load(weights, map_location="cpu").eval()

        def act(obs):
            with torch.no_grad():
                return module(torch.from_numpy(np.asarray(obs, dtype=np.float32))
                              .unsqueeze(0)).squeeze(0).numpy()

        return act

    from stable_baselines3 import PPO

    model = PPO.load(weights, device="cpu")
    return lambda obs: model.predict(obs, deterministic=True)[0]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.environ.get("G1_MODEL", "models/g1/scene_29dof.xml"))
    ap.add_argument("--weights", default="weights/policy_g1.pt")
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--vx", type=float, default=0.6)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--csv", default="saved_models/eval.csv")
    args = ap.parse_args()

    actor = load_actor(args.weights)
    env = G1WalkEnv(model_path=args.model, cmd_vx_range=(args.vx, args.vx),
                    cmd_wz_range=(args.wz, args.wz))

    rows = []
    for ep in range(args.episodes):
        obs, _ = env.reset(options={"cmd": np.array([args.vx, args.wz])})
        vx_err, done, steps = [], False, 0
        t_infer = []
        x0 = float(env.sim.data.qpos[0])
        while not done and steps < EPISODE_STEPS:
            t0 = time.perf_counter()
            action = actor(obs)
            t_infer.append((time.perf_counter() - t0) * 1000)
            obs, _, term, trunc, _ = env.step(action)
            vx_err.append(abs(float(env.sim.base_lin_vel_body()[0]) - args.vx))
            steps += 1
            done = term or trunc
        dx = float(env.sim.data.qpos[0]) - x0
        rows.append(
            dict(
                episode=ep,
                steps=steps,
                seconds=round(steps / 50.0, 2),
                fell=int(term),
                dist_x=round(dx, 3),
                mean_vx=round(dx / max(steps / 50.0, 1e-6), 3),
                vx_mae=round(float(np.mean(vx_err)), 4),
                infer_ms=round(float(np.mean(t_infer)), 3),
            )
        )
        print(rows[-1])

    os.makedirs(os.path.dirname(os.path.abspath(args.csv)) or ".", exist_ok=True)
    with open(args.csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    fell = sum(r["fell"] for r in rows)
    print("\n=== 汇总 ===")
    print(f"指令 vx          : {args.vx:+.2f} m/s")
    print(f"回合数           : {len(rows)}")
    print(f"跌倒率           : {fell / len(rows):.1%}")
    print(f"平均存活         : {np.mean([r['seconds'] for r in rows]):.2f} s")
    print(f"平均前进位移     : {np.mean([r['dist_x'] for r in rows]):+.3f} m")
    print(f"平均前向速度     : {np.mean([r['mean_vx'] for r in rows]):+.3f} m/s"
          f"（指令 {args.vx:+.2f}）")
    print(f"前向速度 MAE     : {np.mean([r['vx_mae'] for r in rows]):.3f} m/s")
    print(f"单帧推理延迟     : {np.mean([r['infer_ms'] for r in rows]):.2f} ms")
    print(f"明细已写入       : {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
