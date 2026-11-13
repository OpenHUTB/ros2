#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""神经网络循迹策略：感知特征 -> 前飞指令与偏航指令。

实现为纯 numpy 的两层多层感知机（输入 3，隐藏层 8，输出 2），
隐藏层与输出层均使用 tanh 激活。权重按反馈控制结构预置为确定值，
不依赖 PyTorch / TensorFlow，离线测试与仿真均可直接前向推理。
"""

from __future__ import annotations

import numpy as np

INPUT_DIM = 3
HIDDEN_DIM = 8
OUTPUT_DIM = 2


class TrackPolicyNetwork:
    """下视循迹 MLP 策略网络。"""

    def __init__(self) -> None:
        self.w1, self.b1 = self._hidden_weights()
        self.w2, self.b2 = self._output_weights()

    @staticmethod
    def _hidden_weights() -> tuple[np.ndarray, np.ndarray]:
        w1 = np.zeros((HIDDEN_DIM, INPUT_DIM), dtype=float)
        # 前两个隐藏单元复制横向偏移，中间两个复制航向误差，最后一个复制速度
        w1[0, 0] = 1.0
        w1[1, 0] = 1.0
        w1[2, 1] = 1.0
        w1[3, 1] = 1.0
        w1[4, 2] = 1.0
        b1 = np.zeros(HIDDEN_DIM, dtype=float)
        return w1, b1

    @staticmethod
    def _output_weights() -> tuple[np.ndarray, np.ndarray]:
        w2 = np.zeros((OUTPUT_DIM, HIDDEN_DIM), dtype=float)
        # 输出 0：前飞指令，固定巡航基础，偏移越大前飞越收敛
        w2[0, 4] = 0.20
        w2[0, 0] = -0.20
        # 输出 1：偏航指令，跟随偏移与航向误差转向标线
        w2[1, 0] = 1.10
        w2[1, 2] = 0.90
        b2 = np.zeros(OUTPUT_DIM, dtype=float)
        b2[0] = 0.45
        return w2, b2

    def forward(self, features: np.ndarray) -> np.ndarray:
        """前向推理。

        参数:
            features: 长度 3 的归一化特征 [偏移, 航向, 速度]。

        返回:
            长度 2 的指令 [前飞指令, 偏航指令]，取值 [-1, 1]。
        """
        x = np.asarray(features, dtype=float).reshape(INPUT_DIM)
        hidden = np.tanh(self.w1 @ x + self.b1)
        out = np.tanh(self.w2 @ hidden + self.b2)
        return out

    def parameters(self) -> dict[str, np.ndarray]:
        """返回网络权重，便于测试核对非零与结构。"""
        return {"w1": self.w1, "b1": self.b1, "w2": self.w2, "b2": self.b2}
