"""ROV 平面运动学：一阶推力滞后 + 海流漂移 + 带噪里程计。

与轮式底盘的两点不同：
1. 推进器有惯性，速度指令不能瞬时生效 —— 用一阶滞后建模；
2. 无 DVL 的 ROV 里程计测不到海流，海流漂移是**系统性**的，
   里程计越走越歪，这正是水下 SLAM 必须存在的理由。

真值位姿与里程计位姿分离，EKF-SLAM 只允许看到里程计（预测步）
和视觉观测（更新步），保证离线与 ROS 两条链路同一套逻辑。
"""

import math

import numpy as np

from . import params as P


def wrap(a):
    """角度归一化到 (-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def integrate(pose, v, w, dt, cur=(0.0, 0.0)):
    """按圆弧积分一步，再叠加世界系海流位移；w→0 时退化为直线。"""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    if abs(w) < 1e-9:
        nx = x + v * math.cos(th) * dt
        ny = y + v * math.sin(th) * dt
    else:
        r = v / w
        nth = wrap(th + w * dt)
        nx = x + r * (math.sin(nth) - math.sin(th))
        ny = y - r * (math.cos(nth) - math.cos(th))
        th = nth
    return np.array([nx + cur[0] * dt, ny + cur[1] * dt, wrap(th)])


class RovDrive:
    """真值 ROV + 里程计。所有噪声都由外部传入的 rng 决定，便于复现。"""

    def __init__(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64).copy()
        self.odom = self.pose.copy()
        self.u = 0.0            # 实际纵向速度（推进器滞后后的）

    def step(self, u_cmd, w, dt, rng, current=(0.0, 0.0), slip_scale=1.0):
        u_cmd = float(np.clip(u_cmd, 0.0, P.MAX_V))
        w = float(np.clip(w, -P.MAX_W, P.MAX_W))
        # 一阶滞后：推进器从当前速度向指令速度收敛
        alpha = min(1.0, dt / P.SURGE_TAU)
        self.u += (u_cmd - self.u) * alpha
        self.pose = integrate(self.pose, self.u, w, dt, current)

        v_meas = self.u * (1.0 + rng.normal(0.0, P.ODOM_SIGMA_V * slip_scale))
        w_meas = w * (1.0 + rng.normal(0.0, P.ODOM_SIGMA_W * slip_scale))
        # 里程计测不到海流：积分里没有 current 项 —— 系统性漂移
        self.odom = integrate(self.odom, v_meas, w_meas, dt)
        return self.pose


def twist_to_thrust(u, w):
    """(u, w) → 四推进器分配（前后各一对纵倾/横倾），仅用于日志/可视化。"""
    return max(0.0, u - 0.5 * abs(w)), max(0.0, u + 0.5 * abs(w))
