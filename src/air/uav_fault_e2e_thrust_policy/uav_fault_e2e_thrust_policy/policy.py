"""三种网络结构：扁平 MLP / 单帧 CNN / 三帧堆叠 CNN。

三者输入输出完全一致（深度图 + 14 维本体量 → 4 个归一化桨推力），差别只在
"怎么看待深度图"：

* ``mlp``：把 32×32 深度图直接展平。参数量最大、没有任何空间先验，
  用来证明"卷积在这个任务上是必要的"；
* ``cnn``：两层卷积 + 全局平均池化，单帧输入，参数量最小；
* ``cnn_stack``：把最近三帧深度图当三个通道堆起来，卷积核因此能看到
  墙面逼近的速度信息（单帧只能看到当前位置）。

这三种结构就是课程任务⑤要求的"同一算法、不同网络结构"对比。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import obs as O

ARCHS = ("mlp", "cnn", "cnn_stack")
ARCH_VERSION = 1
DEPTH_STACK = 3
HIDDEN = 128


def _torch():
    try:
        import torch  # noqa: F401
    except ImportError as exc:  # pragma: no cover - 只在没装 torch 时走到
        raise ImportError(
            "需要 PyTorch：pip install torch （CPU 版即可）。"
            "本模块的训练与推理都基于 torch。"
        ) from exc
    return __import__("torch")


def build_module(arch: str = "cnn"):
    """构造网络（延迟导入 torch，``import`` 本模块不需要 torch）。"""
    if arch not in ARCHS:
        raise ValueError(f"未知结构 {arch!r}，可选：{ARCHS}")
    torch = _torch()
    nn = torch.nn
    channels = DEPTH_STACK if arch == "cnn_stack" else 1

    class PolicyNet(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.arch = arch
            self.stack = DEPTH_STACK if arch == "cnn_stack" else 1
            if arch == "mlp":
                self.trunk = nn.Sequential(
                    nn.Flatten(),
                    nn.Linear(O.DEPTH_DIM * self.stack + O.PROPRIO_DIM, HIDDEN),
                    nn.ReLU(),
                    nn.Linear(HIDDEN, HIDDEN // 2),
                    nn.ReLU(),
                )
                head_in = HIDDEN // 2
            else:
                self.trunk = nn.Sequential(
                    nn.Conv2d(channels, 16, 3, padding=1), nn.ReLU(),
                    nn.Conv2d(16, 32, 3, padding=1, stride=2), nn.ReLU(),
                    nn.Conv2d(32, 32, 3, padding=1, stride=2), nn.ReLU(),
                    nn.AdaptiveAvgPool2d(1),
                    nn.Flatten(),
                    nn.Linear(32, HIDDEN),
                    nn.ReLU(),
                )
                head_in = HIDDEN + O.PROPRIO_DIM
            self.head = nn.Sequential(
                nn.Linear(head_in, HIDDEN // 2), nn.ReLU(),
                nn.Linear(HIDDEN // 2, 4),
            )

        def forward(self, depth, proprio):
            if arch == "mlp":
                x = self.trunk(torch.cat([depth.flatten(1), proprio], dim=1))
                return self.head(x)
            feat = self.trunk(depth)
            # 卷积塔输出与本体量拼接：本体量（速度、健康度）不该被空间卷积稀释
            return self.head(torch.cat([feat, proprio], dim=1))

    torch.manual_seed(0)
    return PolicyNet()


def n_parameters(module) -> int:
    return int(sum(p.numel() for p in module.parameters() if p.requires_grad))


def save_checkpoint(path, module, meta: dict | None = None) -> None:
    torch = _torch()
    import json
    from pathlib import Path

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "arch_version": ARCH_VERSION,
        "arch": {"kind": module.arch, "hidden": HIDDEN, "stack": DEPTH_STACK,
                 "depth_size": O.DEPTH_SIZE, "proprio_dim": O.PROPRIO_DIM},
        "state_dict": module.state_dict(),
        "meta": meta or {},
    }
    torch.save(payload, path)
    meta_path = path.with_suffix(".json")
    meta_path.write_text(
        json.dumps({"arch": payload["arch"], "meta": payload["meta"]},
                   ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def load_checkpoint(path, map_location: str = "cpu"):
    torch = _torch()
    payload = torch.load(path, map_location=map_location, weights_only=False)
    arch = payload.get("arch", {}).get("kind", "cnn")
    module = build_module(arch)
    module.load_state_dict(payload["state_dict"])
    module.eval()
    return module, payload.get("meta", {})


def predict(module, obs_batch: dict) -> np.ndarray:
    """numpy 观测 → numpy 指令（4 维，0~1）。"""
    torch = _torch()
    module.eval()
    with torch.no_grad():
        depth = torch.as_tensor(np.asarray(obs_batch["depth"], dtype=np.float32))
        proprio = torch.as_tensor(np.asarray(obs_batch["proprio"], dtype=np.float32))
        # 网络是线性输出（训练目标是归一化指令），部署时统一夹到 [0, 1]：
        # 训练集里没有越界标签，越界只可能来自训练分布外的输入。
        return np.clip(module(depth, proprio).numpy(), 0.0, 1.0)


def stack_depth(history, module) -> np.ndarray:
    """按网络需要的帧数从历史里取最近 k 帧，返回 ``(1, k, S, S)``。

    ``mlp`` / ``cnn`` 只要一帧，``cnn_stack`` 要三帧；调用方不用关心具体结构。
    """
    k = int(getattr(module, "stack", 1))
    frames = list(history)[-k:]
    if len(frames) < k:
        frames = [frames[0]] * (k - len(frames)) + frames
    return np.stack(frames, axis=0)[None].astype(np.float32)


def predict_one(module, obs: dict) -> np.ndarray:
    depth = np.asarray(obs["depth"], dtype=np.float32)[None, :, :, :]
    proprio = np.asarray(obs["proprio"], dtype=np.float32)[None, :]
    return predict(module, {"depth": depth, "proprio": proprio})[0]
