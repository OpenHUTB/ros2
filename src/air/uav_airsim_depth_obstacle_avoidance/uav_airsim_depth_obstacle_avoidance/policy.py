"""专家策略与网络策略。

专家**只读深度图**，不看真值位姿，所以它产生的标签不含任何位姿信息泄漏：
网络要学的是「看到这样的深度分布该往哪拐」，而不是「读出答案」。

左右两侧深度接近时如果不加滞回，专家会每帧翻转转向方向，无人机原地打转，
所以这里用一个带状态的 ``ExpertPolicy``。
"""
from __future__ import annotations

import numpy as np

# 动作归一化尺度
YAW_RATE_MAX = 1.0     # rad/s，归一化 1.0 对应这个值
ALT_DELTA_MAX = 0.5    # m，归一化 1.0 对应高度目标变化这么多

NEAR_WARN = 0.35       # 归一化深度，约 10.5 m
NEAR_DANGER = 0.15     # 约 4.5 m
SIDE_HYSTERESIS = 0.04  # 左右深度差小于这个值时不切换转向方向


def _column_min(depth: np.ndarray, x0: int, x1: int) -> float:
    region = depth[:, x0:x1]
    return float(np.percentile(region, 10))


class ExpertPolicy:
    """反应式避障专家：中间越近转得越狠，贴脸时同时爬升。"""

    def __init__(self):
        self._side = 0.0   # 上一次选定的转向方向
        self._yaw_prev = 0.0

    def act(self, depth: np.ndarray):
        h, w = depth.shape
        roi = depth[h // 3:, :]
        third = w // 3
        d_left = _column_min(roi, 0, third)
        d_mid = _column_min(roi, third, 2 * third)
        d_right = _column_min(roi, 2 * third, w)

        if d_mid > NEAR_WARN:
            yaw = 0.05 * (d_right - d_left)
            alt = 0.0
            self._side = 0.0
        else:
            urgency = (NEAR_WARN - d_mid) / (NEAR_WARN - NEAR_DANGER + 1e-6)
            urgency = float(np.clip(urgency, 0.0, 1.0))

            diff = d_right - d_left
            if abs(diff) > SIDE_HYSTERESIS:
                self._side = 1.0 if diff > 0 else -1.0
            elif self._side == 0.0:
                self._side = 1.0
            yaw = self._side * (0.35 + 0.55 * urgency)

            alt = 0.0
            if d_mid < NEAR_DANGER:
                alt = 0.5 + 0.5 * urgency

        # 指令平滑，避免一帧一个极端值
        yaw = 0.5 * self._yaw_prev + 0.5 * float(np.clip(yaw, -1.0, 1.0))
        self._yaw_prev = yaw
        alt = float(np.clip(alt, -1.0, 1.0))
        return np.array([yaw, alt], dtype=np.float32)


def expert_action(depth: np.ndarray):
    """无状态便捷入口（每次新建专家，等价于单帧决策）。"""
    return ExpertPolicy().act(depth)


class NeuralPolicy:
    """把网络输出还原为物理量。"""

    def __init__(self, net, z_ref=-5.0):
        self.net = net
        self.z_ref = z_ref

    def act(self, depth: np.ndarray):
        yaw_norm, alt_norm = self.net.predict(depth)
        yaw_rate = float(yaw_norm) * YAW_RATE_MAX
        z_target = float(self.z_ref - float(alt_norm) * ALT_DELTA_MAX)
        z_target = float(np.clip(z_target, -9.0, -3.0))
        return yaw_rate, z_target
