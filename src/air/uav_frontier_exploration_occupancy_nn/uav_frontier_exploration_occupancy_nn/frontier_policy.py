#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""前沿候选打分网络：候选特征 -> 可探索价值评分。

纯 numpy 两层 MLP，输入 4 维候选特征，隐藏层 6 单元，输出标量评分；
权重按探索启发式结构预置，不依赖深度学习框架。
"""

from __future__ import annotations

import numpy as np

INPUT_DIM = 4
HIDDEN_DIM = 6


class FrontierScoreNetwork:
    """对候选前沿点打分并选择目标。"""

    def __init__(self) -> None:
        self.w1, self.b1 = self._hidden_weights()
        self.w2, self.b2 = self._score_weights()

    @staticmethod
    def _hidden_weights() -> tuple[np.ndarray, np.ndarray]:
        w1 = np.zeros((HIDDEN_DIM, INPUT_DIM), dtype=float)
        w1[0, 0] = 1.0   # 距离
        w1[1, 1] = 1.0   # 相对方位
        w1[2, 2] = 1.0   # 邻域未知格数
        w1[3, 3] = 1.0   # 局部自由密度
        b1 = np.zeros(HIDDEN_DIM, dtype=float)
        return w1, b1

    @staticmethod
    def _score_weights() -> tuple[np.ndarray, np.ndarray]:
        w2 = np.zeros((1, HIDDEN_DIM), dtype=float)
        w2[0, 0] = -0.5   # 越近越优
        w2[0, 1] = -0.3   # 方位越正对越优
        w2[0, 2] = 1.4    # 未知邻域越多越优
        w2[0, 3] = 0.3    # 局部可达自由空间越多越优
        b2 = np.array([0.0])
        return w2, b2

    def score(self, features: np.ndarray) -> float:
        x = np.asarray(features, dtype=float).reshape(INPUT_DIM)
        hidden = np.tanh(self.w1 @ x + self.b1)
        return float(self.w2[0] @ hidden + self.b2[0])

    def forward(self, candidates: list[dict]) -> dict:
        """对候选列表打分，返回评分、最佳候选下标与最佳候选。"""
        if not candidates:
            return {"scores": np.zeros(0), "index": -1, "target": None}
        scores = np.array([self.score(c["features"]) for c in candidates])
        index = int(np.argmax(scores))
        return {"scores": scores, "index": index, "target": candidates[index]}

    def parameters(self) -> dict[str, np.ndarray]:
        return {"w1": self.w1, "b1": self.b1, "w2": self.w2, "b2": self.b2}
