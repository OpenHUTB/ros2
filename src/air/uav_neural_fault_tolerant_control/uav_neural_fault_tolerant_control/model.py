"""神经网络残差分配器（PyTorch）。

结构很小：12 → 64 → 64 → 4 的全连接 + tanh，输出的是**残差**而不是从零开始的
控制量，所以初始化就等价于"近似分配"，训练只负责学饱和工况下的修正。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import features as F

ARCH_VERSION = 1
DEFAULT_HIDDEN = (64, 64)


def _torch():
    try:
        import torch
    except ImportError as exc:  # pragma: no cover - 环境问题
        raise ImportError(
            "需要 PyTorch：pip install torch --index-url "
            "https://download.pytorch.org/whl/cpu"
        ) from exc
    return torch


def build_module():
    """延迟构造 nn.Module 子类，这样没装 torch 时别的子模块仍可导入。"""
    torch = _torch()
    nn = torch.nn

    class AllocationResidualNet(nn.Module):
        """``(12,) -> (4,)`` 的残差分配网络。"""

        def __init__(self, feature_dim: int = F.FEATURE_DIM,
                     rotor_count: int = F.ROTOR_COUNT,
                     hidden=DEFAULT_HIDDEN, dropout: float = 0.0):
            super().__init__()
            self.feature_dim = int(feature_dim)
            self.rotor_count = int(rotor_count)
            self.hidden = tuple(int(h) for h in hidden)
            self.dropout = float(dropout)

            layers: list = []
            prev = self.feature_dim
            for width in self.hidden:
                layers.append(nn.Linear(prev, width))
                layers.append(nn.Tanh())
                if self.dropout > 0.0:
                    layers.append(nn.Dropout(self.dropout))
                prev = width
            layers.append(nn.Linear(prev, self.rotor_count))
            self.net = nn.Sequential(*layers)

            # 输出层零初始化：未训练时残差为 0，等价于近似分配，不会比基线更差
            last = self.net[-1]
            if isinstance(last, nn.Linear):
                nn.init.zeros_(last.weight)
                nn.init.zeros_(last.bias)

        def forward(self, x):  # x: (B, 12)
            return self.net(x)

        def n_parameters(self) -> int:
            return int(sum(p.numel() for p in self.parameters()))

        def arch(self) -> dict:
            return {
                "arch_version": ARCH_VERSION,
                "feature_dim": self.feature_dim,
                "rotor_count": self.rotor_count,
                "hidden": list(self.hidden),
                "dropout": self.dropout,
            }

    return AllocationResidualNet


def build_model(feature_dim: int = F.FEATURE_DIM,
                rotor_count: int = F.ROTOR_COUNT, hidden=DEFAULT_HIDDEN,
                dropout: float = 0.0):
    """构造一个网络实例（缺省配置下 5252 个参数，见 ``n_parameters()``）。"""
    cls = build_module()
    return cls(feature_dim=feature_dim, rotor_count=rotor_count,
               hidden=hidden, dropout=dropout)


def save_checkpoint(path, model, meta: dict) -> Path:
    """保存权重 + 结构 + 元信息；同名 ``.json`` 里放可读的元信息。"""
    torch = _torch()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"state_dict": model.state_dict(), "arch": model.arch(),
               "meta": meta or {}}
    torch.save(payload, path)
    path.with_suffix(".json").write_text(
        json.dumps({"arch": model.arch(), "meta": meta or {}},
                   ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def load_checkpoint(path, map_location: str = "cpu"):
    """读回 ``(model, meta)``；按检查点里记录的结构重建，避免尺寸不匹配。"""
    torch = _torch()
    payload = torch.load(Path(path), map_location=map_location, weights_only=False)
    arch = payload.get("arch") or {}
    model = build_model(
        feature_dim=int(arch.get("feature_dim", F.FEATURE_DIM)),
        rotor_count=int(arch.get("rotor_count", F.ROTOR_COUNT)),
        hidden=tuple(arch.get("hidden", DEFAULT_HIDDEN)),
        dropout=float(arch.get("dropout", 0.0)),
    )
    model.load_state_dict(payload["state_dict"])
    model.eval()
    return model, payload.get("meta", {})


def predict_numpy(model, features: np.ndarray) -> np.ndarray:
    """单样本或一批样本的前向推理（numpy 进 / numpy 出，不带上梯度）。"""
    torch = _torch()
    x = np.asarray(features, dtype=np.float32)
    single = x.ndim == 1
    if single:
        x = x[None, :]
    with torch.no_grad():
        out = model(torch.from_numpy(x)).cpu().numpy()
    return out[0] if single else out
