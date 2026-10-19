#!/usr/bin/env python3
"""录制行走演示动图（无 GUI，离屏渲染 + PIL 合成）。

用法：
    python scripts/record_gif.py --weights weights/policy_g1.pt --upper-demo \
        --out docs/img/humanoid/humanoid_ppo_locomotion_demo.gif

``--weights`` 支持 TorchScript（.pt）与 Stable-Baselines3 存档（.zip），按后缀自动判断。
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from humanoid_ppo_locomotion.g1_env import G1WalkEnv  # noqa: E402
from humanoid_ppo_locomotion.sim_core import UPPER_JOINTS  # noqa: E402

_UI = {n: i for i, n in enumerate(UPPER_JOINTS)}

# 上半身演示关键帧：(控制步, {关节名: 目标角})，帧间线性插值
UPPER_KEYFRAMES: list[tuple[int, dict[str, float]]] = [
    (0, {}),
    (60, {"left_shoulder_pitch": 1.00, "right_shoulder_pitch": 1.00}),
    (220, {"left_shoulder_pitch": 1.00, "right_shoulder_pitch": 1.00,
           "left_elbow": 1.00, "right_elbow": 1.00}),
    (380, {"waist_pitch": 0.45, "left_shoulder_roll": 0.70, "right_shoulder_roll": -0.70,
           "left_shoulder_pitch": 0.80, "right_shoulder_pitch": 0.80}),
    (540, {"waist_pitch": -0.30, "waist_yaw": 0.45}),
    (700, {"waist_yaw": -0.45, "left_elbow": 1.60, "right_elbow": 1.60,
           "left_shoulder_pitch": 0.40, "right_shoulder_pitch": 0.40}),
    (860, {}),
    (1000, {}),
]


def upper_demo_targets(step: int) -> np.ndarray:
    """按关键帧插值出 17 维上半身目标角。"""
    kfs = UPPER_KEYFRAMES
    if step <= kfs[0][0]:
        cur = kfs[0][1]
        return np.array([cur.get(n, 0.0) for n in UPPER_JOINTS])
    for (s0, d0), (s1, d1) in zip(kfs, kfs[1:]):
        if step <= s1:
            a = (step - s0) / max(s1 - s0, 1)
            return np.array(
                [d0.get(n, 0.0) * (1 - a) + d1.get(n, 0.0) * a for n in UPPER_JOINTS]
            )
    last = kfs[-1][1]
    return np.array([last.get(n, 0.0) for n in UPPER_JOINTS])


def load_actor(weights: str, torchscript: bool | None = None):
    """``torchscript=None`` 时按后缀自动判断（.pt -> TorchScript，其余按 SB3）。"""
    if torchscript is None:
        torchscript = weights.endswith(".pt")
    if torchscript:
        import torch

        module = torch.jit.load(weights, map_location="cpu").eval()

        def act(obs):
            with torch.no_grad():
                return module(torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()

        return act

    from stable_baselines3 import PPO

    model = PPO.load(weights, device="cpu")

    def act(obs):
        action, _ = model.predict(obs, deterministic=True)
        return action

    return act


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.environ.get("G1_MODEL", "models/g1/scene_29dof.xml"))
    ap.add_argument("--weights", default="weights/policy_g1.pt")
    ap.add_argument("--torchscript", action="store_true",
                    help="强制按 TorchScript 加载（默认按 .pt 后缀自动判断）")
    ap.add_argument("--out", default="docs/img/humanoid/humanoid_ppo_locomotion_demo.gif")
    ap.add_argument("--steps", type=int, default=500)
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--every", type=int, default=4, help="每 N 个控制步取一帧")
    ap.add_argument("--vx", type=float, default=0.6)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--size", default="480x360")
    ap.add_argument("--upper-demo", action="store_true",
                    help="播放上半身动作演示（摆臂/弯腰），展示平衡器补偿")
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    env = G1WalkEnv(model_path=args.model, cmd_vx_range=(args.vx, args.vx),
                    cmd_wz_range=(args.wz, args.wz))
    obs, _ = env.reset(options={"cmd": np.array([args.vx, args.wz])})
    actor = load_actor(args.weights, True if args.torchscript else None)
    renderer = env.sim.make_renderer(width=w, height=h)

    frames = []
    for i in range(args.steps):
        action = actor(obs)
        if args.upper_demo:
            # 上半身按脚本走，腿部仍由策略 + 平衡基元负责
            env.sim.step(env.base_targets() + env.residual_scale * action,
                         upper_demo_targets(i))
            obs = env._obs()
            terminated = env.sim.base_height < 0.5
            truncated = False
        else:
            obs, reward, terminated, truncated, info = env.step(action)
        if i % args.every == 0:
            frames.append(Image.fromarray(env.sim.render(renderer)))
        if terminated or truncated:
            print(f"[gif] 第 {i} 步结束（terminated={terminated}）")
            break

    if not frames:
        print("[gif] 没有采集到帧")
        return 1

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    frames[0].save(
        args.out, save_all=True, append_images=frames[1:],
        duration=int(1000 / args.fps), loop=0, optimize=True,
    )
    size_mb = os.path.getsize(args.out) / 1e6
    print(f"[gif] 已保存 {args.out}（{len(frames)} 帧，{size_mb:.2f} MB）")
    if size_mb > 10:
        print("[gif] 警告：超过 10 MB，请减少 --steps 或 --size")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
