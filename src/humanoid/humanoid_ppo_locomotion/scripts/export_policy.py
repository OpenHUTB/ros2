#!/usr/bin/env python3
"""把 SB3 PPO 权重导出为 TorchScript，供 ROS2 节点在无 SB3 依赖下推理。

用法：
    python scripts/export_policy.py --weights saved_models/stage1/ppo_g1.zip \
        --out weights/policy_g1.pt

导出后的模块输入 ``(1, 40)`` 观测，输出 ``(1, 12)`` **残差动作**（已裁剪到 [-1, 1]）。

注意：输出是残差，不是最终关节角。运行时由 ``sim_node`` 叠加平衡基元::

    targets = ComBalance.targets(NOMINAL, com_err, com_vx) + RESIDUAL_SCALE * action
"""

from __future__ import annotations

import argparse
import os

import torch
from stable_baselines3 import PPO


class PolicyModule(torch.nn.Module):
    """只保留 actor 前向路径：obs -> action。"""

    def __init__(self, policy_net: torch.nn.Sequential, action_net: torch.nn.Linear):
        super().__init__()
        self.policy_net = policy_net
        self.action_net = action_net

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        h = self.policy_net(obs)
        return torch.clamp(self.action_net(h), -1.0, 1.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="saved_models/stage1/ppo_g1.zip")
    ap.add_argument("--out", default="weights/policy_g1.pt")
    args = ap.parse_args()

    model = PPO.load(args.weights, device="cpu")
    sb3_policy = model.policy

    module = PolicyModule(
        sb3_policy.mlp_extractor.policy_net, sb3_policy.action_net
    ).eval()

    obs_dim = sb3_policy.observation_space.shape[0]
    dummy = torch.zeros(1, obs_dim)
    with torch.no_grad():
        traced = torch.jit.trace(module, dummy)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    traced.save(args.out)

    size_mb = os.path.getsize(args.out) / 1e6
    print(f"[export] 观测维度 {obs_dim} -> 动作维度 {module.action_net.out_features}")
    print(f"[export] 已保存 {args.out}（{size_mb:.2f} MB）")

    # 自检：TorchScript 输出与原始策略一致
    with torch.no_grad():
        a_traced = traced(dummy)
        a_ref = torch.clamp(module(dummy), -1.0, 1.0)
    print(f"[export] 一致性检查 max|Δ| = {float((a_traced - a_ref).abs().max()):.2e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
