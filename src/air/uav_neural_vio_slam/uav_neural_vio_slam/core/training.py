#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VIO 前端训练与推理。

训练按**定长片段**做：从每集里切 32 帧，片段内隐状态连续、片段间清零。
隐状态跨片段携带（stateful）但梯度只回传一步，等价于把隐状态当成一个
额外的输入特征 —— 这正是实时 VIO 的真实处境，网络在任何时刻都只能看到
"到当前为止"的信息。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .dataset import VioDataset, VioEpisode
from .network import VioNet

__all__ = ["TrainConfig", "train_vio", "predict_episode", "frontend_metrics",
           "OnlineFrontend"]


@dataclass
class TrainConfig:
    chunk: int = 32
    batch: int = 4
    epochs: int = 30
    lr: float = 2e-3
    weight_decay: float = 1e-5
    grad_clip: float = 1.0
    val_episodes: Tuple[int, ...] = (2,)
    seed: int = 0


def _chunks(ep: VioEpisode, T: int) -> List[int]:
    return list(range(0, max(len(ep) - 1, 1), T))


def train_vio(ds: VioDataset, cfg: Optional[TrainConfig] = None,
              net: Optional[VioNet] = None, verbose: bool = True
              ) -> Tuple[VioNet, Dict[str, List[float]]]:
    from .nn import Adam
    cfg = cfg or TrainConfig()
    net = net or VioNet(seed=cfg.seed)
    opt = Adam(net.layers, lr=cfg.lr, weight_decay=cfg.weight_decay,
               clip=cfg.grad_clip)
    val_idx = set(cfg.val_episodes)
    train_eps = [e for i, e in enumerate(ds.episodes) if i not in val_idx]

    hist = {"train": [], "val": []}
    rng = np.random.default_rng(cfg.seed)
    for ep_i in range(cfg.epochs):
        starts = [(e, s) for e in train_eps for s in _chunks(e, cfg.chunk)]
        rng.shuffle(starts)
        tot, nb = 0.0, 0
        for b in range(0, len(starts), cfg.batch):
            grp = starts[b:b + cfg.batch]
            if len(grp) < 2:
                continue
            h = net.init_hidden(len(grp))
            net.zero_grad()
            loss = 0.0
            for t in range(cfg.chunk):
                sel = [(e, s) for (e, s) in grp if s + t < len(e)]
                if not sel:
                    break
                depth = np.stack([e.depth[s + t] for (e, s) in sel]).astype(np.float32)
                depth = _depth_input(depth)
                imu = np.stack([e.imu[s + t] for (e, s) in sel]).astype(np.float32)
                dpt = np.stack([e.dp[s + t] for (e, s) in sel]).astype(np.float32)
                drt = np.stack([e.dr[s + t] for (e, s) in sel]).astype(np.float32)
                h = h[:len(sel)]
                dp, dr, ls, h = net.forward_step(depth, imu, h)
                info = net.backward_step(dp, dr, ls, dpt, drt)
                loss += info["loss_p"] + info["loss_r"]
            opt.step()
            tot += loss
            nb += 1
        hist["train"].append(tot / max(nb, 1))
        v = _val_loss(net, ds, list(val_idx), cfg.chunk)
        hist["val"].append(v)
        if verbose and (ep_i % 5 == 0 or ep_i == cfg.epochs - 1):
            print("  epoch %3d  训练损失=%.5f  验证损失=%.5f"
                  % (ep_i, hist["train"][-1], v))
    hist["best_val"] = float(min(hist["val"])) if hist["val"] else float("nan")
    return net, hist


def _depth_input(q: np.ndarray) -> np.ndarray:
    """uint8 深度 → (B, 2, H, W)：归一化深度 + 有效掩码。

    掩码是必需的：超量程像素为 0，若不区分"没有回波"和"距离为 0"，
    墙面与天空会变成同一个输入。
    """
    d = q.astype(np.float32)
    valid = (d > 0).astype(np.float32)
    z = np.clip(d / 255.0, 0.0, 1.0) * valid
    return np.stack([z, valid], axis=1)


def _val_loss(net: VioNet, ds: VioDataset, idx: Sequence[int], T: int) -> float:
    tot, nb = 0.0, 0
    for i in idx:
        e = ds.episodes[i]
        h = net.init_hidden(1)
        for t in range(len(e)):
            depth = _depth_input(e.depth[t][None])
            dp, dr, ls, h = net.forward_step(depth, e.imu[t][None], h)
            ep, er = dp[0] - e.dp[t], dr[0] - e.dr[t]
            w = np.exp(-ls[0])
            tot += 0.5 * float(np.mean(ep ** 2 * w[:3] + ls[0, :3]))
            tot += 0.5 * float(np.mean(er ** 2 * w[3:] + ls[0, 3:]))
            nb += 1
    return tot / max(nb, 1)


# --------------------------------------------------------------------------- #
def predict_episode(net: VioNet, ep: VioEpisode, batch_depth: bool = False
                    ) -> Dict[str, np.ndarray]:
    """在整个 episode 上跑一遍前端，返回逐帧的增量估计与不确定度。"""
    n = len(ep)
    dp = np.empty((n, 3))
    dr = np.empty((n, 3))
    sg = np.empty((n, 6))
    h = net.init_hidden(1)
    for t in range(n):
        depth = _depth_input(ep.depth[t][None])
        a, b, s, h = net.predict(depth, ep.imu[t][None].astype(np.float32), h)
        dp[t], dr[t], sg[t] = a[0], b[0], s[0]
    return {"dp": dp, "dr": dr, "sigma": sg}


class OnlineFrontend:
    """逐帧推理，跨帧保持 GRU 隐状态。ROS 节点每收到一帧深度调一次 ``step``。"""

    def __init__(self, net: VioNet):
        self.net = net
        self.h = net.init_hidden(1)

    def reset(self) -> None:
        self.h = self.net.init_hidden(1)

    def step(self, depth_u8: np.ndarray, imu: np.ndarray
             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``depth_u8`` 为 (48,64) uint8；``imu`` 为 (10,6)。

        返回 (dp(3,), dr(3,), sigma(6,))。前端输出的 dp/dr 是**与真值同尺度**的
        位移增量与欧拉角增量，不能在这里做任何滤波或缩放。
        """
        d = _depth_input(np.asarray(depth_u8)[None])
        a, b, s, self.h = self.net.predict(
            d, np.asarray(imu, dtype=np.float32)[None], self.h)
        return a[0], b[0], s[0]


def frontend_metrics(pred: Dict[str, np.ndarray], ep: VioEpisode) -> Dict[str, float]:
    """前端的逐帧增量误差与不确定度标定质量。"""
    ep_ = pred["dp"] - ep.dp
    er_ = pred["dr"] - ep.dr
    return {
        "dp_rmse": float(np.sqrt(np.mean(ep_ ** 2))),
        "dr_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(er_ ** 2)))),
        "dp_bias": float(np.linalg.norm(ep_.mean(axis=0))),
        # 不确定度标定：|误差| / σ 的均值应接近 0.8（高斯分布下 E|x|/σ = √(2/π)）
        "calib_p": float(np.mean(np.abs(ep_) / np.maximum(pred["sigma"][:, :3], 1e-6))),
        "calib_r": float(np.mean(np.abs(er_) / np.maximum(pred["sigma"][:, 3:], 1e-6))),
    }
