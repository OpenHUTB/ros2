"""极简环岛汇入闭环仿真（无 CARLA，纯运动学）。

ego 车沿直道驶向环岛入口（x=0 为停车线，x>0 为已进入环岛），
环岛内有固定车流 NPC 周期性经过冲突点；把 NPC 在 ego 参考系里近似成
"前方纵向距离 oncoming_x + 横向 oncoming_y"，和数据集渲染用同一套几何。

每步：
  - 控制器给 (steer, throttle, brake)
  - ego 速度/位置更新
  - 检测是否与 NPC 冲突（碰撞）或成功完成汇入（越过 x_finish）
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dataset import render_frame


@dataclass
class RoundaboutEnv:
    """单 ego + 固定 NPC 的环岛汇入仿真。"""

    ego_accel: float = 3.0          # 油门最大加速度 m/s^2
    ego_decel: float = 6.0         # 刹车最大减速度 m/s^2
    x_finish: float = 8.0           # 越过该 x 视为成功汇入
    collision_x: float = 1.5        # 进入环岛后与 NPC 冲突的 x 区间
    dt: float = 0.1
    max_steps: int = 200

    npc_phase: float = 0.0          # NPC 相位（固定车流下每 episode 重置）
    x: float = -6.0
    v: float = 0.0
    lateral: float = 0.0
    t: float = 0.0
    done: bool = False
    crashed: bool = False
    finished: bool = False

    def reset(self, npc_phase: float = 0.0) -> np.ndarray:
        self.npc_phase = npc_phase
        self.x, self.v, self.lateral = -6.0, 0.0, 0.0
        self.t = 0.0
        self.done = self.crashed = self.finished = False
        return self._observe()

    def _npc_state(self):
        """NPC 在 ego 参考系的近似位置。

        简化：NPC 绕环岛匀速，周期 10s，oncoming_x 在 2~12m 间正弦变化。
        """
        t = self.t + self.npc_phase
        oncoming_x = 7.0 + 5.0 * np.cos(t * (2 * np.pi / 10.0))
        oncoming_y = 1.5 * np.sin(t * (2 * np.pi / 10.0))
        return oncoming_x, oncoming_y

    def _observe(self) -> np.ndarray:
        oncoming_x, oncoming_y = self._npc_state()
        return render_frame(self.lateral, oncoming_x, oncoming_y, seed=1000 + int(self.t * 10))

    def step(self, action: np.ndarray):
        """action = (steer, throttle, brake)。返回 (obs, speed, done, info)。"""
        steer, throttle, brake = float(action[0]), float(action[1]), float(action[2])
        self.v += (throttle * self.ego_accel - brake * self.ego_decel) * self.dt
        self.v = float(np.clip(self.v, 0.0, 8.0))
        self.x += self.v * self.dt
        self.lateral += steer * 0.3  # 方向修正横向

        oncoming_x, oncoming_y = self._npc_state()
        self.t += self.dt

        info = {"oncoming_x": oncoming_x, "oncoming_y": oncoming_y, "x": self.x, "v": self.v}

        # 碰撞：ego 已进入环岛冲突区 且 NPC 距离过近
        if self.x >= -self.collision_x and self.x <= self.collision_x and oncoming_x < 3.0:
            self.crashed = True
            self.done = True
        # 成功汇入
        if self.x >= self.x_finish:
            self.finished = True
            self.done = True
        if self.t >= self.max_steps * self.dt:
            self.done = True

        return self._observe(), self.v, self.done, info
