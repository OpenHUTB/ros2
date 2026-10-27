"""对比实验用的轻量 2D 导航环境（无需 AirSim 即可训练/评测）。

状态：机器人当前位置 (x, y) 与到目标的单位向量，共 4 维；
动作：连续速度 (vx, vy) ∈ [-1,1]（PPO/SAC），或离散方向（DQN，8 方向）；
奖励：靠近目标 +1，撞障碍 -10，到目标 +100。
"""

import numpy as np


class Nav2DEnv:
    def __init__(self, size=10.0, obstacles=None):
        self.size = size
        self.goal = np.array([size - 1, size - 1], dtype=np.float32)
        # 若干圆形障碍
        self.obstacles = obstacles or [(size / 2, size / 2, 1.0),
                                       (size / 3, size * 0.7, 0.8)]
        self.pos = None

    def reset(self):
        self.pos = np.array([1.0, 1.0], dtype=np.float32)
        return self._state()

    def _state(self):
        d = self.goal - self.pos
        n = np.linalg.norm(d) + 1e-6
        return np.concatenate([self.pos / self.size, d / n]).astype(np.float32)

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        self.pos = self.pos + action * 0.5
        self.pos = np.clip(self.pos, 0, self.size)
        # 碰撞
        for (ox, oy, r) in self.obstacles:
            if np.hypot(self.pos[0] - ox, self.pos[1] - oy) < r:
                return self._state(), -10.0, True, {}
        dist = np.linalg.norm(self.goal - self.pos)
        if dist < 0.8:
            return self._state(), 100.0, True, {}
        return self._state(), -0.1 + 1.0 / (dist + 1e-3), False, {}
