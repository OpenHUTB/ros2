"""在线推理：把训练好的网络包装成"喂一帧特征，出一组健康度"的对象。

两种用法：

* ``HealthEstimator.push(step_features)`` —— 供 ROS 节点/实时演示用，
  内部维护一个定长滑窗缓冲，缓冲没填满时返回 ``None``；
* ``estimate_sequence(features)`` —— 供离线评测与绘图用，对整段序列做滑窗推理。

两条路径共用同一个 ``_predict``，保证"训练时验证的东西"和"部署时跑的东西"一致。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

from .features import FEATURE_DIM, ROTOR_COUNT
from .model import FaultDetectorNet, load_checkpoint


def _predict(model: FaultDetectorNet, windows: np.ndarray, batch_size: int = 512) -> np.ndarray:
    """对 ``(n, window, feature)`` 批量推理，返回 ``(n, 4)``。"""
    if windows.shape[0] == 0:
        return np.zeros((0, ROTOR_COUNT), dtype=np.float64)
    model.eval()
    outputs = []
    with torch.no_grad():
        for start in range(0, windows.shape[0], batch_size):
            chunk = torch.from_numpy(np.ascontiguousarray(windows[start : start + batch_size])).float()
            outputs.append(model(chunk).cpu().numpy())
    return np.concatenate(outputs, axis=0).astype(np.float64)


class HealthEstimator:
    """滑窗缓冲 + 模型推理。"""

    def __init__(self, checkpoint: str | Path, window: int | None = None,
                 device: str = "cpu") -> None:
        self.model, self.meta = load_checkpoint(checkpoint)
        self.checkpoint = str(checkpoint)
        self.window = int(window or self.meta.get("window", 100))
        if self.window < 1:
            raise ValueError(f"window 必须 >= 1，收到 {self.window}")
        self.device = device
        self._buffer = np.zeros((0, FEATURE_DIM), dtype=np.float32)
        self._last: np.ndarray | None = None

    # ------------------------------------------------------------------ 实时
    def reset(self) -> None:
        """清空缓冲（换一次飞行时调用）。"""
        self._buffer = np.zeros((0, FEATURE_DIM), dtype=np.float32)
        self._last = None

    @property
    def ready(self) -> bool:
        return self._buffer.shape[0] >= self.window

    @property
    def last(self) -> np.ndarray | None:
        return None if self._last is None else self._last.copy()

    def push(self, step_features: np.ndarray) -> np.ndarray | None:
        """推入一帧特征；缓冲填满后返回该时刻的四桨健康度估计。"""
        step = np.asarray(step_features, dtype=np.float32).reshape(1, FEATURE_DIM)
        self._buffer = np.concatenate([self._buffer, step], axis=0)[-self.window :]
        if not self.ready:
            return None
        self._last = _predict(self.model, self._buffer.reshape(1, self.window, FEATURE_DIM))[0]
        return self._last.copy()

    def estimate(self, windows: np.ndarray) -> np.ndarray:
        """对一批已经切好的窗口做推理。"""
        return _predict(self.model, np.asarray(windows, dtype=np.float32))


def estimate_sequence(
    checkpoint: str | Path,
    features: np.ndarray,
    window: int | None = None,
    stride: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """对整段 ``(T, 13)`` 特征做滑窗推理。

    返回 ``(predictions, end_index)``：前者 ``(n, 4)``，后者是每条的结束下标，
    方便和时间轴对齐。
    """
    estimator = HealthEstimator(checkpoint, window=window)
    window = estimator.window
    features = np.asarray(features, dtype=np.float32)
    n_steps = features.shape[0]
    if n_steps < window:
        return np.zeros((0, ROTOR_COUNT), dtype=np.float64), np.zeros((0,), dtype=np.int64)

    ends = np.arange(window - 1, n_steps, max(1, int(stride)), dtype=np.int64)
    windows = np.stack([features[end - window + 1 : end + 1] for end in ends])
    return estimator.estimate(windows), ends
