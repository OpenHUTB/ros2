#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""穿越门竞速环境：状态 = 深度图 + 当前速度，动作 = 机体系速度指令。

动作不是直接给四个桨推力，也不是给加速度，而是给**机体系速度**——与课程
接口 `/uav/cmd_vel` 同一语义，策略可以直接接到真机上；速度指令再由底座已有的
"速度积分成参考轨迹 + 几何控制器跟踪"那一层执行，端到端网络只负责决策。

门框判定是几何的：门是位于 x = 5(i+1) 的竖直矩形框，开口 2.2×2.2，
无人机质心穿过该平面且落在开口内即算通过。判定不依赖任何感知，
避免"感知对了但判定错了"污染奖励。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from uav_mujoco_sim.core import GeometricController, Quadrotor, quat_to_mat
from uav_mujoco_sim.core.scenes import make_scene
from uav_mujoco_sim.core.sensors import CameraConfig, VisionSensors

__all__ = ["EnvConfig", "Gate", "GateRacingEnv", "GateTrack"]

SIM_DT = 0.01
CTRL_HZ = 20.0                    # 策略频率
FRAME_DT = 1.0 / CTRL_HZ
DEPTH_MIN, DEPTH_MAX = 0.2, 12.0
DEPTH_HW = (48, 64)


def depth_to_u8(d: np.ndarray) -> np.ndarray:
    a = np.asarray(d, np.float32)
    q = np.rint((a - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * 254.0) + 1.0
    q[(a <= 1e-6) | (a < DEPTH_MIN) | (a > DEPTH_MAX)] = 0.0
    return np.clip(q, 0, 255).astype(np.uint8)


@dataclass
class Gate:
    idx: int
    x: float
    y: float
    z: float
    half: float = 1.1             # 开口半宽/半高

    def contains(self, p: np.ndarray) -> bool:
        return abs(p[1] - self.y) <= self.half and abs(p[2] - self.z) <= self.half


class GateTrack:
    """从场景对象里解析出门的位置（与 ``scenes.gate_track`` 的布局一一对应）。"""

    def __init__(self, scene) -> None:
        self.gates: List[Gate] = []
        posts: Dict[float, List] = {}
        for o in scene.objects:
            if not o.name.startswith("gate_post"):
                continue
            posts.setdefault(round(o.pos[0], 3), []).append(o)
        for i, (x, os_) in enumerate(sorted(posts.items())):
            ys = [o.pos[1] for o in os_]
            zs = [o.pos[2] for o in os_]
            self.gates.append(Gate(i, x, float(np.mean(ys)), float(np.mean(zs))))

    def __len__(self) -> int:
        return len(self.gates)

    @property
    def z_avg(self) -> float:
        return float(np.mean([g.z for g in self.gates])) if self.gates else 2.0


@dataclass
class EnvConfig:
    scene: str = "gate_track"
    seed: int = 0
    max_time: float = 40.0
    start_x: float = 0.0
    start_y: float = 0.0
    start_z: float = 2.0
    target_speed: float = 3.0      # 完成时间折算用的参考速度
    max_horiz: float = 4.0
    max_vert: float = 2.0
    max_yaw_rate: float = 3.0      # 参考航向的最大角速率（rad/s）
    depth_hw: Tuple[int, int] = DEPTH_HW
    #: 前进奖励系数。过门奖励占主导：一轮 270 帧、3 m/s，系数 1.0 时进度分 810，
    #: 8 道门共 80 分，"直着飞不穿门"与"全穿门"几乎同分。取 0.2 后进度 162、
    #: 过门 400。
    reward_progress: float = 0.2
    #: 只给**干净穿过开口**发奖励。擦框时 ``passed`` 不增加，等于相对少拿 50 分
    #: ——这就是擦框的惩罚，不需要另设一项负奖励去重复扣。
    reward_gate: float = 50.0
    reward_energy: float = 0.01
    reward_smooth: float = 0.05
    #: 只压住大倾角，不惩罚正常前飞所需的倾斜（3 m/s 巡航约 20°，系数取 2.0
    #: 时整轮罚 108 分，把过门奖励又淹了）。
    reward_tilt: float = 0.5

    def label(self) -> str:
        return "%s/seed%d/vmax%.1f" % (self.scene, self.seed, self.max_horiz)


class GateRacingEnv:
    """一个 episode = 依次穿过全部门。"""

    def __init__(self, cfg: Optional[EnvConfig] = None):
        self.cfg = cfg or EnvConfig()
        self.scene = make_scene(self.cfg.scene, seed=self.cfg.seed)
        self.track = GateTrack(self.scene)
        self.quad = Quadrotor(scene=self.scene, dt=SIM_DT, substeps=4)
        self.ctrl = GeometricController(self.quad.params)
        self.cam = CameraConfig(width=self.cfg.depth_hw[1],
                                height=self.cfg.depth_hw[0], quality="fast",
                                depth_max_range=DEPTH_MAX)
        self.vision: Optional[VisionSensors] = None
        self._dt = FRAME_DT
        self._sub = int(round(FRAME_DT / SIM_DT))
        self._v_filt = np.zeros(3)
        self._ref = np.zeros(3)
        self._ref_yaw = 0.0
        self._prev_action = np.zeros(3)
        self._prev_depth = None
        self._t = 0.0
        self.next_gate = 0
        self.n_collide = 0
        self.n_passed = 0

    # ------------------------------------------------------------------ 生命周期
    def _ensure_vision(self) -> None:
        """建立渲染器。深度图是这个任务的**唯一输入**，没有"跳过渲染"的模式。

        早期版本沿用底座的 ``headless`` 开关（在底座里是"不渲染视觉传感器"，
        对键盘遥控/规划是合理的），但搬到这里就成了陷阱：训练全用
        ``--headless`` 跑，策略吃到的是一张全零图，只能退化成"照平均速度平飞"，
        BC 的横向输出塌到 ±0.05 m/s 而训练损失看着还在降。
        """
        if self.vision is None:
            self.vision = VisionSensors(self.quad.model, self.cam)

    def reset(self, seed: Optional[int] = None, jitter: float = 0.0) -> Dict[str, np.ndarray]:
        self._ensure_vision()
        cfg = self.cfg
        rng = np.random.default_rng(np.random.randint(0, 1 << 30) if seed is None
                                    else seed)
        p = np.array([cfg.start_x, cfg.start_y, cfg.start_z])
        if jitter > 0:
            p = p + rng.uniform(-jitter, jitter, 3)
            p[2] = max(p[2], 1.0)
        self.quad.reset(pos=tuple(p), yaw=0.0)
        self._v_filt = np.zeros(3)
        self._ref = p.copy()
        self._ref_yaw = 0.0
        self._prev_action = np.zeros(3)
        self._t = 0.0
        self.next_gate = 0
        self.n_collide = 0
        self.n_passed = 0
        self._prev_pos = self.quad.state().pos.copy()
        self._prev_depth = None
        return self.observe()

    # ------------------------------------------------------------------ 观测
    def observe(self) -> Dict[str, np.ndarray]:
        self._ensure_vision()
        st = self.quad.state()
        img = depth_to_u8(self.vision.depth(self.quad.data))
        R = quat_to_mat(st.quat)
        vel_b = R.T @ st.vel
        return {"depth": img,
                "vel": vel_b.astype(np.float32),
                "pos": st.pos.astype(np.float32),
                "quat": st.quat.astype(np.float32),
                "tilt": np.array([st.roll, st.pitch], np.float32)}

    # ------------------------------------------------------------------ 步进
    def _command(self, action: np.ndarray) -> np.ndarray:
        """机体系速度指令 → 参考轨迹。

        映射用的是**参考航向**（航向系），不是无人机的实时姿态。用实时姿态会
        形成正反馈：机身一倾斜，``R @ a`` 就跟着转，参考点被拽向侧面，姿态环
        为了追上又倾得更多——竞速里几个门下来就直接翻过去（实测倾斜到 170°、
        掉到 −1.5 km）。底座原有的 ``VelocityCommander`` 用实时姿态没问题，是
        因为它的轨迹本来就平缓；这里把姿态从回路里摘出去。
        """
        cfg = self.cfg
        a = np.asarray(action, float).reshape(3)
        a = np.array([np.clip(a[0], -cfg.max_horiz, cfg.max_horiz),
                      np.clip(a[1], -cfg.max_horiz, cfg.max_horiz),
                      np.clip(a[2], -cfg.max_vert, cfg.max_vert)])
        c, s = math.cos(self._ref_yaw), math.sin(self._ref_yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        v_world = Rz @ a
        tau = 0.18                                  # 一阶低通，抑制指令抖动
        alpha = min(1.0, self._dt / tau)
        self._v_filt = self._v_filt + (v_world - self._v_filt) * alpha
        self._ref = self._ref + self._v_filt * self._dt
        self._ref[2] = float(np.clip(self._ref[2], 0.6, 9.0))
        # 机头朝向跟随速度方向；限制转向速率，避免航向突变
        vh = self._v_filt[:2]
        if np.linalg.norm(vh) > 0.5:
            want = math.atan2(vh[1], vh[0])
            d = math.atan2(math.sin(want - self._ref_yaw),
                           math.cos(want - self._ref_yaw))
            lim = cfg.max_yaw_rate * self._dt
            self._ref_yaw += max(-lim, min(lim, d))
        return a

    def step(self, action: np.ndarray):
        cfg = self.cfg
        a = self._command(action)
        p_prev = self.quad.state().pos.copy()
        for _ in range(self._sub):
            self.quad.step_with_control(
                self.ctrl.act(self.quad, self._ref, self._v_filt,
                              np.zeros(3), self._ref_yaw))
        self._t += self._dt
        st = self.quad.state()
        p = st.pos.copy()

        # 门通过判定。一次只结算一个门，无论成功还是擦框都要推进门序：
        # 擦了框却不推进，下一步会因"已越过平面"再记一次碰撞，一次失误记两笔。
        passed = 0
        while self.next_gate < len(self.track.gates):
            g = self.track.gates[self.next_gate]
            if p[0] <= g.x:
                break
            if p_prev[0] < g.x and g.contains(p):
                passed += 1                     # 这一步正好穿过开口
                self.n_passed += 1
            else:
                self.n_collide += 1             # 擦框，或从门框外侧绕过
            self.next_gate += 1

        speed_fwd = float(np.linalg.norm(p[:2] - p_prev[:2])) / self._dt
        tilt = math.hypot(st.roll, st.pitch)
        r = (cfg.reward_progress * speed_fwd
             + cfg.reward_gate * passed
             - cfg.reward_energy * float(np.sum(a ** 2))
             - cfg.reward_smooth * float(np.sum((a - self._prev_action) ** 2))
             - cfg.reward_tilt * tilt)
        self._prev_action = a
        done = (self.next_gate >= len(self.track.gates)) or (self._t >= cfg.max_time)
        # passed 只数**干净穿过开口**的门；consumed 是消耗掉的门数（含擦框），
        # 两者不等说明有门是蹭过去的。只报 next_gate 会让"8/8"看起来全过。
        info = {"passed": self.n_passed, "consumed": self.next_gate,
                "collisions": self.n_collide,
                "time": self._t, "tilt": tilt,
                "pos": p.copy(),
                "completed": self.next_gate >= len(self.track.gates)}
        return self.observe(), float(r), bool(done), info

    # ------------------------------------------------------------------ 工具
    @property
    def ref_yaw(self) -> float:
        """参考航向。专家要在这个坐标系里算误差，才与 ``_command`` 的映射一致。"""
        return self._ref_yaw

    def expert_target(self) -> np.ndarray:
        """下一个门开口中心的机体系方向（专家与奖励整形都用它）。"""
        st = self.quad.state()
        g = (self.track.gates[min(self.next_gate, len(self.track.gates) - 1)]
             if len(self.track.gates) else None)
        if g is None:
            return st.pos.copy()
        return np.array([g.x, g.y, g.z])

    def close(self) -> None:
        if self.vision is not None:
            self.vision.close()
            self.vision = None

    def __del__(self):
        try:
            self.close()
        except Exception:                                       # noqa: BLE001
            pass
