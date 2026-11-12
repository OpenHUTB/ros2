"""闭环迫降控制器：选落点 → 速度指令 → 几何控制器 → 推力分配 → 四桨指令。

三种模式（评测与演示都按这三种比较）：

- ``vertical``：不建图、不选点，原地以固定速度下降。这是"什么都不做"的基线，
  真实场景里单桨失效后最常见的默认行为。
- ``expert``：用规则评分选落点 + 手写专家律给速度指令。
- ``neural``：用神经网络评分选落点 + 神经网络给速度指令。

后两级共用同一套底层：``GeometricController`` 算姿态与总推力，``ls_allocation`` 在
"有效推力"空间做加权最小二乘分配（按可控桨数切换行权重，桨失效时主动放弃偏航）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import allocation as A
from . import landing as L
from . import mapping as M
from . import model as Md

MODES = ("vertical", "expert", "neural")

#: 建图节流：每这么多步把一帧点云并进高度图（0.1 s 一帧足够，点云很便宜）
MAP_EVERY = 10

#: 到位门限：水平距离小于这个半径才允许压低高度
ARRIVE_RADIUS = 1.2
#: 未到位时的最低高度 = 已知最高障碍 + 该余量
MAP_MARGIN = 0.8


@dataclass
class PlanState:
    """控制器对外暴露的内部状态，方便记录与画图。"""

    target_xy: Optional[np.ndarray] = None
    target_score: float = 0.0
    velocity_setpoint: Optional[np.ndarray] = None
    desired: Optional[np.ndarray] = None
    commands: Optional[np.ndarray] = None


class ForcedLandingController:
    """故障后的迫降闭环控制器。"""

    def __init__(self, params, mixer, base_controller, effectiveness,
                 cfg: Optional[L.LandingConfig] = None, mode: str = "expert",
                 score_net=None, guidance_net=None):
        if mode not in MODES:
            raise ValueError(f"未知模式 {mode!r}，可选：{MODES}")
        self.params = params
        self.mixer = mixer
        self.base = base_controller
        self.cfg = cfg or L.LandingConfig()
        self.mode = mode
        self.score_net = score_net
        self.guidance_net = guidance_net

        self.effectiveness = np.asarray(effectiveness, dtype=float)
        self.gains = np.ones(4, dtype=float)
        self.bias = np.zeros(4, dtype=float)
        self.height_map = M.HeightMap()
        self.plan = PlanState()
        self._steps = 0
        self._v_last: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ #
    # 状态注入
    # ------------------------------------------------------------------ #
    def set_health(self, gains, bias=None) -> None:
        self.gains = np.asarray(gains, dtype=float).reshape(4)
        self.bias = np.zeros(4) if bias is None else np.asarray(bias, dtype=float).reshape(4)

    def reset(self) -> None:
        self.height_map.reset()
        self.plan = PlanState()
        self._steps = 0
        self._v_last = None

    def row_weights(self) -> np.ndarray:
        """按可控桨数切优先级权重：有桨彻底失效时主动放弃偏航。"""
        controllable = int(np.sum(self.gains > A.GAIN_EPS))
        if controllable >= 4:
            return np.array([A.THRUST_WEIGHT, 1.0, 1.0, 1.0])
        return np.array([A.THRUST_WEIGHT, 10.0, 10.0, 0.0])

    # ------------------------------------------------------------------ #
    # 建图与选点
    # ------------------------------------------------------------------ #
    def update_map(self, points) -> None:
        """把一帧世界系点云并进高度图（未到节流步数就直接跳过）。"""
        self.height_map.update(points)

    def score_map(self, position_xy=None) -> np.ndarray:
        """当前模式下的落点评分图。"""
        if self.mode == "neural":
            if self.score_net is None:
                raise RuntimeError("neural 模式需要加载评分网络")
            grid = M.height_map_input(self.height_map)
            # predict_score 支持 (R,C) 与 (N,R,C)，这里把 batch 维压掉
            out = np.asarray(Md.predict_score(self.score_net, grid), dtype=float)
            return out.reshape(self.height_map.shape)
        return M.rule_landing_score(self.height_map, position_xy=position_xy)

    def choose_target(self, position_xy, refresh: bool = False):
        """选落点；``vertical`` 模式直接用当前位置正下方。"""
        if self.mode == "vertical":
            xy = np.asarray(position_xy, dtype=float).reshape(2)
            self.plan.target_xy = xy
            self.plan.target_score = 0.0
            return xy
        if self.plan.target_xy is not None and not refresh:
            return self.plan.target_xy
        score = self.score_map(position_xy)
        picked = L.select_target(score, self.height_map, position_xy=position_xy)
        if picked is None:
            # 没找到可降落处：先原地悬停等下一帧图，比乱飞安全
            self.plan.target_xy = None
            self.plan.target_score = 0.0
            return None
        row, col, value = picked
        center = self.height_map.cell_center(row, col)
        self.plan.target_xy = np.asarray(center, dtype=float)
        self.plan.target_score = float(value)
        return self.plan.target_xy

    # ------------------------------------------------------------------ #
    # 速度指令
    # ------------------------------------------------------------------ #
    def velocity_setpoint(self, state, target_xy) -> np.ndarray:
        pos = np.asarray(state.pos, dtype=float).reshape(3)
        vel = np.asarray(state.vel, dtype=float).reshape(3)
        z_target = self.cfg.touch_z
        altitude = max(float(pos[2] - z_target), 0.0)
        rel = np.array([target_xy[0] - pos[0], target_xy[1] - pos[1], -altitude])

        if self.mode == "vertical":
            return np.array([0.0, 0.0, -float(np.clip(0.4 + 0.25 * altitude, 0.4, 1.6))])
        if self.mode == "neural":
            if self.guidance_net is None:
                raise RuntimeError("neural 模式需要加载制导网络")
            feat = L.guidance_features(rel, vel, self.gains, altitude)
            out = Md.predict_guidance(self.guidance_net, feat)
            return np.asarray(out, dtype=float).reshape(3)
        return L.expert_velocity(rel, vel, self.gains, altitude, self.cfg)

    def _arrival_gate(self, state, target_xy, v_des) -> np.ndarray:
        """到位门限：还没飞到目标正上方时不许降到已知最高障碍以下。

        四旋翼在底座里没有碰撞体，会直接穿过楼板——如果一边下降一边平飞，
        就会从楼里穿过去再落到地面，指标上表现为"撞到障碍物"。
        这一层是安全包络，不改变专家律/网络本身的输出，只是把"先到位再下降"
        这条常识强制加上。
        """
        pos = np.asarray(state.pos, dtype=float).reshape(3)
        dist = float(np.hypot(target_xy[0] - pos[0], target_xy[1] - pos[1]))
        if dist <= ARRIVE_RADIUS:
            return v_des
        ceiling = 0.0
        if bool(self.height_map.observed.any()):
            seen = np.where(self.height_map.observed, self.height_map.height, 0.0)
            ceiling = float(np.max(seen))
        altitude = max(float(pos[2]) - self.cfg.touch_z, 0.0)
        if altitude < ceiling + MAP_MARGIN:
            return np.array([v_des[0], v_des[1], max(float(v_des[2]), 0.0)])
        return v_des

    # ------------------------------------------------------------------ #
    # 主循环单步
    # ------------------------------------------------------------------ #
    def act(self, state, lidar_points=None) -> np.ndarray:
        """给一个状态与可选的一帧点云，返回四桨指令。"""
        self._steps += 1
        if lidar_points is not None and self.mode != "vertical":
            self.update_map(lidar_points)

        pos = np.asarray(state.pos, dtype=float).reshape(3)
        yaw = float(getattr(state, "yaw", 0.0))

        target_xy = self.choose_target(pos[:2], refresh=self._steps % MAP_EVERY == 0)
        if target_xy is None:
            # 还没找到落点：原地保持当前高度，别乱飞
            v_des = np.zeros(3)
        else:
            v_des = self.velocity_setpoint(state, target_xy)
            v_des = self._arrival_gate(state, target_xy, v_des)
        self.plan.velocity_setpoint = v_des

        # 位置设定点取"当前位置"、速度设定点交给制导律：外环退化成纯速度跟踪，
        # 这样下降速度完全由制导律（专家或网络）决定，不会被位置误差项抢走。
        p_d = pos.copy()
        a_d = np.zeros(3)
        thrust, tau = self.base.compute(state, p_d, v_des, a_d, yaw)
        desired = np.array([thrust, tau[0], tau[1], tau[2]], dtype=float)
        self.plan.desired = desired

        u = A.ls_allocation(self.effectiveness, desired, self.params, self.gains,
                            v_ref=self._v_last, row_weights=self.row_weights())
        self._v_last = A.effective_thrust(u, self.gains, self.bias)
        commands = A.clamp_thrusts(u, self.params)
        self.plan.commands = commands
        return commands
