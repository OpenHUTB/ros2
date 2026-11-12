"""两个小网络的模型定义、检查点读写与 numpy 推理。

1. :class:`LandingScoreNet`——可降落性评分网络。输入 1×R×C 的高度图（未观测处填 1），
   输出同尺寸 0~1 评分图。全卷积（3 个 3×3 卷积、无池化、无全连接），所以换网格尺寸
   （比如 48² → 64²）不用改结构，参数量只有几千个，CPU 单帧 < 1 ms。
2. :class:`GuidanceNet`——迫降制导网络。输入 13 维状态（相对落点位置、速度、高度、
   四桨推力权限、剩余可用推力占比），输出 3 维期望速度。用行为克隆学"专家迫降律"。

torch 只在需要时导入（模块级 import torch 会让 `import uav_fault_emergency_landing`
在没有 torch 的机器上直接挂掉，而这个包里 diff/评分/地图都是纯 numpy 的）。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

ARCH_VERSION = 1

#: 评分网络隐藏通道数
SCORE_CHANNELS = (16, 16)
#: 制导网络隐藏层宽度
GUIDANCE_HIDDEN = (64, 64)
#: 制导网络输入/输出维度
GUIDANCE_IN = 13
GUIDANCE_OUT = 3


def _torch():
    try:
        import torch  # noqa: F401
    except Exception as exc:  # pragma: no cover - 只在缺依赖时触发
        raise ImportError(
            "该功能需要 PyTorch：pip install torch --index-url https://download.pytorch.org/whl/cpu"
        ) from exc
    import torch

    return torch


def build_score_module(channels: Tuple[int, int] = SCORE_CHANNELS):
    """构造可降落性评分网络（全卷积）。

    ``channels`` 必须与检查点里 ``arch.channels`` 一致——``load_checkpoint()``
    会把检查点里存的结构传进来，这样以后改通道数也不会把旧权重读废。
    """
    torch = _torch()
    nn = torch.nn

    class LandingScoreNet(nn.Module):
        def __init__(self, channels: Tuple[int, int] = SCORE_CHANNELS):
            super().__init__()
            c1, c2 = channels
            self.channels = tuple(int(c) for c in channels)
            self.features = nn.Sequential(
                nn.Conv2d(1, c1, 3, padding=1), nn.ReLU(),
                nn.Conv2d(c1, c2, 3, padding=1), nn.ReLU(),
            )
            self.head = nn.Conv2d(c2, 1, 3, padding=1)

        def forward(self, x):
            return torch.sigmoid(self.head(self.features(x)))

    return LandingScoreNet(tuple(channels))


def build_guidance_module(hidden: Tuple[int, ...] = GUIDANCE_HIDDEN):
    """构造迫降制导网络；``hidden`` 同理来自检查点的 ``arch.hidden``。"""
    torch = _torch()
    nn = torch.nn
    hidden_default = tuple(int(h) for h in hidden)

    class GuidanceNet(nn.Module):
        def __init__(self, hidden: Tuple[int, ...] = hidden_default):
            super().__init__()
            self.hidden = tuple(int(h) for h in hidden)
            layers = []
            prev = GUIDANCE_IN
            for h in self.hidden:
                layers += [nn.Linear(prev, h), nn.Tanh()]
                prev = h
            layers += [nn.Linear(prev, GUIDANCE_OUT)]
            self.net = nn.Sequential(*layers)

        def forward(self, x):
            return self.net(x)

    return GuidanceNet(hidden_default)


# --------------------------------------------------------------------------- #
# 检查点
# --------------------------------------------------------------------------- #
def save_checkpoint(path: str | Path, kind: str, module: Any,
                    meta: Optional[Dict[str, Any]] = None) -> Path:
    """保存权重 + 结构信息（``arch``），并在旁边写一份可读的 ``.json``。"""
    torch = _torch()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    arch = {"kind": kind, "arch_version": ARCH_VERSION}
    if kind == "score":
        arch["channels"] = list(getattr(module, "channels", SCORE_CHANNELS))
    elif kind == "guidance":
        arch["hidden"] = list(getattr(module, "hidden", GUIDANCE_HIDDEN))
    else:
        raise ValueError(f"未知模型类型 {kind!r}")
    payload = {"arch": arch, "state_dict": module.state_dict(), "meta": meta or {}}
    torch.save(payload, str(path))
    path.with_suffix(".json").write_text(
        json.dumps({"arch": arch, "meta": meta or {}}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return path


def load_checkpoint(path: str | Path, map_location: str = "cpu") -> Tuple[Any, Dict[str, Any]]:
    """按检查点里存的 ``arch`` 重建网络，返回 ``(module, meta)``。

    结构参数（``channels`` / ``hidden``）从检查点里读，而不是用模块默认值：
    否则以后调整了通道数或隐藏层，旧权重会因为 ``size mismatch`` 直接读不出来。
    """
    torch = _torch()
    payload = torch.load(str(path), map_location=map_location, weights_only=False)
    arch = payload.get("arch") or {}
    version = arch.get("arch_version")
    if version is not None and int(version) > ARCH_VERSION:
        raise ValueError(
            f"检查点用的网络版本 {version} 比当前代码 {ARCH_VERSION} 新，"
            "请更新代码或重新训练")
    kind = arch.get("kind")
    if kind == "score":
        module = build_score_module(tuple(arch.get("channels") or SCORE_CHANNELS))
    elif kind == "guidance":
        module = build_guidance_module(tuple(arch.get("hidden") or GUIDANCE_HIDDEN))
    else:
        raise ValueError(f"检查点里没有合法的 arch.kind：{arch!r}")
    module.load_state_dict(payload["state_dict"])
    module.eval()
    return module, payload.get("meta") or {}


def n_parameters(module: Any) -> int:
    return int(sum(p.numel() for p in module.parameters()))


# --------------------------------------------------------------------------- #
# numpy 推理（部署路径不依赖 torch 的 autograd，也不建计算图）
# --------------------------------------------------------------------------- #
def predict_score(module: Any, grid: np.ndarray) -> np.ndarray:
    """``grid``: (R,C) 或 (N,R,C) → 评分图 (R,C) / (N,R,C)。"""
    torch = _torch()
    arr = np.asarray(grid, dtype=np.float32)
    single = arr.ndim == 2
    x = torch.from_numpy(arr[None, None] if single else arr[:, None])
    with torch.no_grad():
        out = module(x).numpy()
    return out[0, 0] if single else out[:, 0]


def predict_guidance(module: Any, features: np.ndarray) -> np.ndarray:
    """``features``: (13,) 或 (N,13) → 期望加速度 (3,) / (N,3)。"""
    torch = _torch()
    arr = np.asarray(features, dtype=np.float32)
    single = arr.ndim == 1
    x = torch.from_numpy(arr[None, :] if single else arr)
    with torch.no_grad():
        out = module(x).numpy()
    return out[0] if single else out
