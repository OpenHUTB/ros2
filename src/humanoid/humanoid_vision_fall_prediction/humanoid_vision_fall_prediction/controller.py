"""
controller.py —— 基于预测结果的平衡控制
=========================================

策略：
  * normal   → 站姿 PD（不额外修正）
  * forward  → 踝背屈（ankle_pitch 增大）+ 髋前倾（hip_pitch 增大），抵消前倒
  * backward → 踝跖屈（ankle_pitch 减小）+ 髋后倾
  * left     → 踝外翻（ankle_roll 增大）+ 髋侧倾
  * right    → 踝内翻（ankle_roll 减小）+ 髋侧倾

修正叠加在站姿目标之上，修正量随置信度线性缩放。
"""

from __future__ import annotations

import numpy as np
from .sim import DEFAULT_STAND_TARGET

# 修正方向（与 DEFAULT_STAND_TARGET 的 15 维关节向量对齐）
# 关节顺序：
#   0 hip_roll_l, 1 hip_pitch_l, 2 knee_l, 3 ankle_pitch_l, 4 ankle_roll_l
#   5 hip_roll_r, 6 hip_pitch_r, 7 knee_r, 8 ankle_pitch_r, 9 ankle_roll_r
#   10 shoulder_pitch_l, 11 elbow_l, 12 shoulder_pitch_r, 13 elbow_r, 14 neck
_CORRECTION = {
    1: np.array([ 0,  0.12, 0,  0.10,  0,   0,  0.12, 0,  0.10,  0,   0, 0, 0, 0, 0]),  # forward
    2: np.array([ 0, -0.12, 0, -0.10,  0,   0, -0.12, 0, -0.10,  0,   0, 0, 0, 0, 0]),  # backward
    3: np.array([ 0.08, 0, 0, 0,  0.10, -0.08, 0, 0, 0, -0.10,   0, 0, 0, 0, 0]),  # left
    4: np.array([-0.08, 0, 0, 0, -0.10,  0.08, 0, 0, 0,  0.10,   0, 0, 0, 0, 0]),  # right
}


class BalanceController:
    """根据预测类别输出修正后的关节目标角。"""

    def __init__(self, gain: float = 0.6, confidence_threshold: float = 0.70):
        self.gain = float(gain)
        self.threshold = float(confidence_threshold)
        self.last_label = 0
        self.last_conf = 0.0

    def target_for(self, cls: int, conf: float) -> np.ndarray:
        """返回 15 维关节目标角。"""
        self.last_label = cls
        self.last_conf = conf
        if cls == 0 or conf < self.threshold:
            return DEFAULT_STAND_TARGET.copy()
        corr = _CORRECTION.get(cls, np.zeros(15))
        scale = self.gain * conf
        return DEFAULT_STAND_TARGET + scale * corr
