"""故障检测网络：一维卷积 + GRU 回归四桨健康度。

输入是一段滑窗特征 ``(window, 13)``，输出四个桨的增益估计 ``ĝ ∈ (0, 1)``，
``1`` 表示该桨完好，``0`` 表示该桨已经出不了力。

为什么是这个形状：

* **卷积在前**——故障的特征是"指令推力与实测比力/角速度之间的关系被破坏"，
  这是局部时序上的形态变化，卷积核能把它提出来；
* **GRU 在后**——健康度是随时间演化的状态（渐进退化要看着它一路掉下去），
  循环层比单帧分类稳定得多，也不容易在噪声上抖出孤立报警；
* **输出四维 + sigmoid**——四个桨共用一套权重，网络必须靠时序线索区分是哪个桨坏了，
  这比"四分类 + 健康"更省参数，也天然支持多桨同时失效。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import torch
from torch import nn

from .features import FEATURE_DIM, ROTOR_COUNT

#: 检查点里记录的结构版本，改网络结构时递增，加载时会校验
ARCH_VERSION = 1


class FaultDetectorNet(nn.Module):
    """``(B, window, 13)`` -> ``(B, 4)`` 健康度。"""

    def __init__(
        self,
        feature_dim: int = FEATURE_DIM,
        rotor_count: int = ROTOR_COUNT,
        channels: tuple[int, int, int] = (32, 32, 48),
        gru_hidden: int = 64,
        head_dim: int = 64,
        dropout: float = 0.1,
    ) -> None:
        super().__init__()
        c1, c2, c3 = channels
        self.feature_dim = int(feature_dim)
        self.rotor_count = int(rotor_count)
        # 记下结构超参，检查点里要一并保存，否则换尺寸训练的模型读不回来
        self.channels = (int(c1), int(c2), int(c3))
        self.gru_hidden = int(gru_hidden)
        self.head_dim = int(head_dim)
        self.dropout = float(dropout)

        self.conv = nn.Sequential(
            nn.Conv1d(feature_dim, c1, kernel_size=5, padding=2),
            nn.BatchNorm1d(c1),
            nn.ReLU(inplace=True),
            nn.Conv1d(c1, c2, kernel_size=5, padding=2),
            nn.BatchNorm1d(c2),
            nn.ReLU(inplace=True),
            nn.MaxPool1d(2),
            nn.Conv1d(c2, c3, kernel_size=3, padding=1),
            nn.BatchNorm1d(c3),
            nn.ReLU(inplace=True),
            nn.MaxPool1d(2),
        )
        self.gru = nn.GRU(c3, gru_hidden, num_layers=1, batch_first=True)
        self.head = nn.Sequential(
            nn.Linear(gru_hidden, head_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(head_dim, rotor_count),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.dim() != 3:
            raise ValueError(f"输入应为 (B, window, feature)，收到 {tuple(x.shape)}")
        h = x.transpose(1, 2)          # (B, F, W)
        h = self.conv(h)               # (B, C, W/4)
        h = h.transpose(1, 2)          # (B, T, C)
        out, _ = self.gru(h)
        last = out[:, -1]              # 取最后一个时刻的隐状态
        return torch.sigmoid(self.head(last))

    # ---------------------------------------------------------------- 辅助
    def n_parameters(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)


def build_model(**kwargs: Any) -> FaultDetectorNet:
    """按默认结构建网（参数留给单元测试调小尺寸用）。"""
    return FaultDetectorNet(**kwargs)


def save_checkpoint(path: str | Path, model: FaultDetectorNet, meta: dict) -> Path:
    """保存权重 + 结构/预处理元信息，推理时只加载这一个文件。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "arch_version": ARCH_VERSION,
        "feature_dim": model.feature_dim,
        "rotor_count": model.rotor_count,
        "arch": {
            "channels": list(model.channels),
            "gru_hidden": model.gru_hidden,
            "head_dim": model.head_dim,
            "dropout": model.dropout,
        },
        "state_dict": model.state_dict(),
        "meta": meta,
    }
    torch.save(payload, path)
    with path.with_suffix(".json").open("w", encoding="utf-8") as handle:
        json.dump({"arch_version": ARCH_VERSION, "meta": meta}, handle,
                  ensure_ascii=False, indent=2)
    return path


def load_checkpoint(path: str | Path) -> tuple[FaultDetectorNet, dict]:
    """加载 ``save_checkpoint`` 写出的检查点，返回 (模型, meta)。"""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"找不到检查点 {path}")
    payload = torch.load(path, map_location="cpu", weights_only=False)
    version = int(payload.get("arch_version", 0))
    if version != ARCH_VERSION:
        raise ValueError(f"检查点结构版本 {version} 与代码 {ARCH_VERSION} 不一致")

    arch = dict(payload.get("arch") or {})
    model = FaultDetectorNet(
        feature_dim=int(payload["feature_dim"]),
        rotor_count=int(payload["rotor_count"]),
        channels=tuple(arch.get("channels", (32, 32, 48))),
        gru_hidden=int(arch.get("gru_hidden", 64)),
        head_dim=int(arch.get("head_dim", 64)),
        dropout=float(arch.get("dropout", 0.1)),
    )
    model.load_state_dict(payload["state_dict"])
    model.eval()
    return model, dict(payload.get("meta", {}))
