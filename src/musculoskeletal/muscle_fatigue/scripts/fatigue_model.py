# -*- coding: utf-8 -*-
"""疲劳动力学：从 muscle_model 暴露疲劳相关接口。

单独拆出来是为了让"疲劳"和"Hill 肌肉本体"在引用上清晰分开：
- FatigueParams  疲劳因子的一阶动力学参数
- fatigue_dynamics  推进疲劳因子 phi 一个时间步
- fatigued_muscle_force  用 phi 缩放后的肌肉总力
"""

from __future__ import annotations

from muscle_model import (
    FatigueParams,
    fatigue_dynamics,
    fatigued_muscle_force,
)

__all__ = ["FatigueParams", "fatigue_dynamics", "fatigued_muscle_force"]
