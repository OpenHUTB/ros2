#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地形通行度神经网络：扇区地形特征 -> 三扇区可通行度与速度指令。

实现为纯 numpy 两层 MLP：输入 9（三扇区各高度/台阶/坡度），隐藏层 8，
输出三扇区通行度 logits 经 softmax 归一化，以及一路 tanh 速度指令。
权重按通行度反馈结构预置，不依赖深度学习框架。
"""

from __future__ import annotations

import numpy as np

INPUT_DIM = 9
HIDDEN_DIM = 8
SECTORS = 3


class PassabilityNetwork:
    """地形可通行性评估与决策网络。"""

    def __init__(self) -> None:
        self.w1, self.b1 = self._hidden_weights()
        self.w_logits, self.b_logits = self._logits_weights()
        self.w_speed, self.b_speed = self._speed_weights()

    @staticmethod
    def _hidden_weights() -> tuple[np.ndarray, np.ndarray]:
        w1 = np.zeros((HIDDEN_DIM, INPUT_DIM), dtype=float)
        # 每个扇区的高度与台阶各复制到一个隐藏单元
        w1[0, 0] = 1.0
        w1[1, 1] = 1.0
        w1[2, 3] = 1.0
        w1[3, 4] = 1.0
        w1[4, 6] = 1.0
        w1[5, 7] = 1.0
        w1[6, 2] = 0.5
        w1[6, 5] = 0.5
        w1[6, 8] = 0.5
        b1 = np.zeros(HIDDEN_DIM, dtype=float)
        b1[7] = 1.0
        return w1, b1

    @staticmethod
    def _logits_weights() -> tuple[np.ndarray, np.ndarray]:
        w = np.zeros((SECTORS, HIDDEN_DIM), dtype=float)
        # 某扇区高度/台阶越大，其通行度 logits 越低
        w[0, 0] = -2.2
        w[0, 1] = -0.6
        w[1, 2] = -2.2
        w[1, 3] = -0.6
        w[2, 4] = -2.2
        w[2, 5] = -0.6
        b = np.zeros(SECTORS, dtype=float)
        return w, b

    @staticmethod
    def _speed_weights() -> tuple[np.ndarray, np.ndarray]:
        w = np.zeros((1, HIDDEN_DIM), dtype=float)
        w[0, 6] = -0.3      # 坡度越大速度越低
        b = np.array([0.60])
        return w, b

    def forward(self, features: np.ndarray) -> dict:
        """前向推理。

        返回:
            字典：``passability`` 三扇区通行度（和为 1），
            ``speed`` 速度指令 [-1, 1]，``sector`` 最优扇区下标。
        """
        x = np.asarray(features, dtype=float).reshape(INPUT_DIM)
        hidden = np.tanh(self.w1 @ x + self.b1)
        logits = self.w_logits @ hidden + self.b_logits
        passability = self._softmax(logits)
        speed = float(np.tanh(self.w_speed @ hidden + self.b_speed)[0])
        sector = int(np.argmax(passability))
        return {"passability": passability, "speed": speed, "sector": sector}

    @staticmethod
    def _softmax(logits: np.ndarray) -> np.ndarray:
        logits = logits - np.max(logits)
        exp = np.exp(logits)
        return exp / np.sum(exp)

    def parameters(self) -> dict[str, np.ndarray]:
        return {"w1": self.w1, "b1": self.b1, "w_logits": self.w_logits,
                "b_logits": self.b_logits, "w_speed": self.w_speed,
                "b_speed": self.b_speed}
