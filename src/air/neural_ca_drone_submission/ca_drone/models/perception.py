"""Depth-to-obstacle segmentation network (the perception module)."""

from __future__ import annotations

from typing import Dict

import numpy as np

from ..nn.autograd import Tensor, concat, maxpool2d, upsample2d
from ..nn.layers import Conv2d
from ..nn.optim import Module


class PerceptionNet(Module):
    """Small U-Net: 1x72x96 depth -> 1x72x96 obstacle logits."""

    def __init__(self):
        self.c1 = Conv2d(1, 12, 3)
        self.c2 = Conv2d(12, 24, 3)
        self.c3 = Conv2d(24, 32, 3)
        self.c4 = Conv2d(32, 48, 3)
        self.c5 = Conv2d(48 + 32, 32, 3)
        self.c6 = Conv2d(32 + 24, 24, 3)
        self.c7 = Conv2d(24 + 12, 16, 3)
        self.head = Conv2d(16, 1, 1, padding=0)

    def forward(self, x: Tensor) -> Tensor:
        e1 = self.c1(x).relu()
        p1 = maxpool2d(e1)
        e2 = self.c2(p1).relu()
        p2 = maxpool2d(e2)
        e3 = self.c3(p2).relu()
        p3 = maxpool2d(e3)
        e4 = self.c4(p3).relu()
        u3 = upsample2d(e4, (e3.shape[2], e3.shape[3]))
        d3 = self.c5(concat([u3, e3], axis=1)).relu()
        u2 = upsample2d(d3, (e2.shape[2], e2.shape[3]))
        d2 = self.c6(concat([u2, e2], axis=1)).relu()
        u1 = upsample2d(d2, (e1.shape[2], e1.shape[3]))
        d1 = self.c7(concat([u1, e1], axis=1)).relu()
        return self.head(d1)

    def parameters(self) -> Dict[str, Tensor]:
        d = {}
        for name in ("c1", "c2", "c3", "c4", "c5", "c6", "c7", "head"):
            for k, v in getattr(self, name).parameters().items():
                d[f"{name}.{k}"] = v
        return d


def normalize_depth(depth: np.ndarray, max_range: float = 12.0) -> np.ndarray:
    d = np.clip(np.asarray(depth, dtype=np.float64) / max_range, 0.0, 1.0)
    return d[None, None, :, :] if d.ndim == 2 else d[:, None, :, :]


def predict_mask(model: PerceptionNet, depth: np.ndarray, max_range: float = 12.0) -> np.ndarray:
    x = Tensor(normalize_depth(depth, max_range))
    logits = model(x).data[0, 0]
    return 1.0 / (1.0 + np.exp(-np.clip(logits, -60, 60)))
