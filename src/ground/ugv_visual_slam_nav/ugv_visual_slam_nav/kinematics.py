"""差速驱动底盘：圆弧精确积分 + 带噪里程计。

真值位姿与里程计位姿分离，EKF-SLAM 只允许看到里程计（预测步）
和视觉观测（更新步），保证"用里程计做 SLAM"这件事在离线与 ROS
两条链路上是同一套逻辑。
"""

import math

import numpy as np

from . import params as P


def wrap(a):
    """角度归一化到 (-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def integrate(pose, v, w, dt):
    """按圆弧（而非直线近似）积分一步，w→0 时退化为直线。"""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    if abs(w) < 1e-9:
        return np.array([x + v * math.cos(th) * dt,
                         y + v * math.sin(th) * dt,
                         wrap(th)])
    r = v / w
    nth = wrap(th + w * dt)
    return np.array([x + r * (math.sin(nth) - math.sin(th)),
                     y - r * (math.cos(nth) - math.cos(th)),
                     nth])


class DiffDrive:
    """真值底盘 + 里程计。所有噪声都由外部传入的 rng 决定，便于复现。"""

    def __init__(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64).copy()
        self.odom = self.pose.copy()
        self._slip = 0.0

    def step(self, v, w, dt, rng, slip_scale=1.0):
        v = float(np.clip(v, -P.MAX_V, P.MAX_V))
        w = float(np.clip(w, -P.MAX_W, P.MAX_W))
        self.pose = integrate(self.pose, v, w, dt)

        # 打滑：一个缓慢变化的随机偏置，模拟轮子打滑/地面不平
        self._slip = 0.9 * self._slip + 0.1 * rng.normal(0.0, P.ODOM_SIGMA_SLIP * slip_scale)
        v_meas = v * (1.0 + rng.normal(0.0, P.ODOM_SIGMA_V * slip_scale))
        w_meas = w * (1.0 + rng.normal(0.0, P.ODOM_SIGMA_W * slip_scale)) + self._slip
        self.odom = integrate(self.odom, v_meas, w_meas, dt)
        return self.pose


def twist_to_wheels(v, w):
    """(v, w) → 左右轮线速度，仅用于日志/可视化。"""
    half = P.WHEELBASE / 2.0
    return v - w * half, v + w * half
