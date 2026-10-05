"""任务2-B：给定参考轨迹的神经网络跟踪控制。

参考轨迹在水平面内为圆形（可换方形/螺旋），固定飞行高度。控制器为 ``MLPPolicy``：
    状态  s = [e_x, e_y, e_z, e_heading]   （相对当前目标航点的误差）
    输出  u = [v_x, v_y, v_z] ∈ [-1,1]     （归一化速度指令）
网络输出乘上速度上限 ``v_max``。为做到开箱即用，输出层权重被初始化为一个
PD 增益矩阵（等价比例控制器），后续可用行为克隆在示教数据上微调。
"""

import numpy as np

from common.models import MLPPolicy

try:  # pragma: no cover
    import torch
    _HAS_TORCH = True
except ImportError:  # pragma: no cover
    torch = None
    _HAS_TORCH = False


def circle_trajectory(radius=5.0, z=-8.0, n=60):
    """水平面圆轨迹，返回 (n,3) 的 ENU 航点；z 取负表示在地面上方。"""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.stack([radius * np.cos(t), radius * np.sin(t),
                     np.full(n, z)], axis=1)


def square_trajectory(side=6.0, z=-8.0, n_per_side=15):
    pts = []
    for (x, y) in [(0, 0), (side, 0), (side, side), (0, side)]:
        pass
    corners = np.array([[0, 0], [side, 0], [side, side], [0, side], [0, 0]])
    for i in range(len(corners) - 1):
        for k in np.linspace(0, 1, n_per_side):
            p = corners[i] * (1 - k) + corners[i + 1] * k
            pts.append([p[0], p[1], z])
    return np.array(pts)


class NeuralTracker:
    def __init__(self, client, waypoints=None, v_max=2.0, close_thr=0.8):
        self.client = client
        self.wps = waypoints if waypoints is not None else circle_trajectory()
        self.v_max = v_max
        self.close_thr = close_thr
        self.policy = MLPPolicy(state_dim=4, act_dim=3)
        self._init_pd_weights()
        self.idx = 0

    def _init_pd_weights(self):
        """把 MLP 输出层初始化为比例控制：u = -K s / v_max。"""
        if not _HAS_TORCH:
            return
        with torch.no_grad():
            # state = [ex,ey,ez,heading] -> u = [-ex,-ey,-ez,0]/v_max
            k = 1.0 / self.v_max
            w = self.policy.net[-2].weight  # 最后一个 Linear（Tanh 前）
            w.zero_()
            w[0, 0] = -k
            w[1, 1] = -k
            w[2, 2] = -k

    def _next_waypoint(self, pos):
        """选择距离 <= close_thr 就前进，否则沿用当前航点。"""
        target = self.wps[self.idx]
        if np.linalg.norm(pos - target) < self.close_thr:
            self.idx = (self.idx + 1) % len(self.wps)
            target = self.wps[self.idx]
        return target

    def on_tick(self, client):
        pos = client.get_position()
        target = self._next_waypoint(pos)
        err = target - pos
        heading = client.get_yaw()
        state = np.array([err[0], err[1], err[2], heading], dtype=np.float32)
        if _HAS_TORCH:
            with torch.no_grad():
                u = self.policy(torch.from_numpy(state).unsqueeze(0)).squeeze(0).numpy()
        else:  # 退化 PD
            u = np.array([-state[0], -state[1], -state[2], 0.0], dtype=np.float32) / self.v_max
        u = np.clip(u, -1, 1)
        client.move_by_velocity(u[0] * self.v_max, u[1] * self.v_max, u[2] * self.v_max)

    def run(self, seconds=40, rate_hz=10.0):
        self.client.start()
        self.client.spin(seconds, on_tick=self.on_tick, rate_hz=rate_hz)
        self.client.destroy()
