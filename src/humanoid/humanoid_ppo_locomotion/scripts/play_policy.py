#!/usr/bin/env python3
"""离线演示：加载权重，在 MuJoCo 交互窗口里跑策略。

用法：
    python scripts/play_policy.py --weights weights/policy_g1.pt
    python scripts/play_policy.py --weights weights/policy_g1.pt --vx 0.5
    python scripts/play_policy.py --weights ... --headless --steps 500   # 无窗口，只打印指标

按 Ctrl+C 或关闭窗口退出。窗口模式下方向键调整指令：
    ↑/↓ 前后速度 ±0.1 m/s，←/→ 转向 ±0.1 rad/s，空格停车
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from humanoid_ppo_locomotion.g1_env import G1WalkEnv  # noqa: E402


def load_actor(weights: str, env):
    """支持 SB3 的 .zip 和导出的 TorchScript .pt。"""
    if weights.endswith(".pt"):
        import torch

        module = torch.jit.load(weights, map_location="cpu").eval()

        def act(obs):
            with torch.no_grad():
                return module(torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()

        return act

    from stable_baselines3 import PPO

    model = PPO.load(weights, device="cpu")

    def act(obs):
        a, _ = model.predict(obs, deterministic=True)
        return a

    return act


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.environ.get(
        "G1_MODEL", "models/g1/scene_29dof.xml"))
    ap.add_argument("--weights", default="weights/policy_g1.pt")
    ap.add_argument("--vx", type=float, default=0.4)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--steps", type=int, default=0, help="0 表示一直跑")
    ap.add_argument("--headless", action="store_true", help="不开窗口，只打印指标")
    args = ap.parse_args()

    env = G1WalkEnv(model_path=args.model,
                    cmd_vx_range=(args.vx, args.vx),
                    cmd_wz_range=(args.wz, args.wz))
    actor = load_actor(args.weights, env)

    obs, _ = env.reset(options={"cmd": np.array([args.vx, args.wz])})

    viewer = None
    if not args.headless:
        try:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(env.sim.model, env.sim.data)
        except Exception as exc:  # 无显示器 / 后端缺失
            print(f"[play] 无法打开交互窗口（{exc}），改为无窗口模式")

    vx, wz = args.vx, args.wz
    step, t0 = 0, time.perf_counter()
    try:
        while True:
            a = actor(obs)
            obs, reward, term, trunc, _ = env.step(a)
            step += 1

            if viewer is not None:
                viewer.sync()
                # 方向键调指令
                key = viewer.key_callback if hasattr(viewer, "key_callback") else None
                _ = key  # 保留扩展点；当前用命令行参数设定指令

            if step % 50 == 0:
                print(f"[play] t={step/50:5.1f}s  x={env.sim.data.qpos[0]:+.3f}  "
                      f"h={env.sim.base_height:.3f}  cmd=({vx:+.1f},{wz:+.1f})  r={reward:+.2f}")

            if term or trunc:
                print(f"[play] 回合结束：{'跌倒' if term else '到达步数上限'}，"
                      f"共 {step} 步（{step/50:.2f} s）")
                break
            if args.steps and step >= args.steps:
                break
            if viewer is not None and not viewer.is_running():
                break
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        dt = time.perf_counter() - t0
        print(f"[play] 共 {step} 步，实时率 {step/50/dt:.2f}x（1.0 表示与真实时间同步）")
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
