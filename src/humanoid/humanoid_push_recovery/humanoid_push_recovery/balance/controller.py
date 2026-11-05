# -*- coding: utf-8 -*-
"""平衡控制器：组合踝策略与髋策略，输出关节力矩修正量。

组织结构参考（Apache License 2.0）：
    PrageethM1702/Bipedal-Walking-and-Push-Recovery
    https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery
    （其 ``control/main_controller.py`` 按「踝策略 → 髋策略」分级调用；
     本站按 MuJoCo 力矩控制模型改写，并保留本站已验证的判据与限幅逻辑）

为什么输出的是力矩而不是关节目标
--------------------------------
踝 pitch 关节的实际活动范围只有 ±0.9 rad（见 ``models/humanoid_balance.xml``）。
若用「关节目标」表达平衡修正，为获得足够的恢复力矩需要下发远超关节行程的目标
（例如 3 rad ≈ 172°），既不合理、又会被关节限位截断。因此平衡控制器直接输出
**关节力矩修正量 (N·m)**，由 ``sim_core.step()`` 叠加到站姿 PD 力矩上。

符号约定（由 ``test_standalone.py`` 的符号自检项实测标定）
---------------------------------------------------------
  * 踝 pitch **正力矩 → 质心向 −x（后）移动**；
  * 踝 roll  **正力矩 → 质心向 +y（左）移动**；
  * 髋 pitch **正力矩 → 质心向 +x（前）移动**；
  * 髋 roll  **正力矩 → 质心向 −y（右）移动**。
符号写反会形成正反馈直接推倒机器人。
"""

import numpy as np

from ..sim_core import JOINT_NAMES
from .ankle_strategy import AnkleStrategy
from .capture_point import capture_point
from .hip_strategy import HipStrategy
from .support_polygon import distance_to_boundary


def stability_margin(com, com_vel, height, polygon=None, use_capture_point=True):
    """稳定裕度 (m)：**捕获点（或质心投影）到支撑多边形边界的最近距离**。

    参数:
        com, com_vel: 质心位置与速度
        height:       质心高度 (m)，用于捕获点的 ω = √(g/h)
        polygon:      支撑多边形顶点 (N×2)。为 None 时返回 NaN
                      （调用方应显式传入 ``sim.foot_polygon()``）
        use_capture_point:
                      True  —— 用捕获点 ξ 计算裕度（动态判据：裕度 > 0 表示仅靠
                               踝/髋策略仍可停下，< 0 表示必须跨步）；
                      False —— 用质心投影计算裕度（静态判据）。

    返回值 > 0 表示仍在支撑多边形内，< 0 表示已越界。
    """
    if polygon is None or len(polygon) < 3:
        return float("nan")
    point = capture_point(com, com_vel, height) if use_capture_point \
        else np.asarray(com, dtype=float)[:2]
    return distance_to_boundary(point, polygon)


class BalanceController:
    """抗扰恢复控制器：踝策略（主） + 髋策略（增强）。

    ``compute(sim)`` 返回 15 维**关节力矩修正量** (N·m)，由 ``sim_core.step()``
    叠加到站姿 PD 力矩上。
    """

    def __init__(self, params=None):
        p = params or {}
        # 踝策略（矢状面 + 侧向），力矩量纲：
        #   倾角 PD 项来自上游开源实现，质心位置/速度项为本站为消除稳态偏差保留
        self.ankle = AnkleStrategy(
            kp_lean=float(p.get("kp_ankle_lean", 300.0)),
            kd_lean=float(p.get("kd_ankle_lean", 75.0)),
            kp_pos=float(p.get("kp_ankle", 5400.0)),
            kd_pos=float(p.get("kd_ankle", 1440.0)),
            kp_lean_roll=float(p.get("kp_ankle_roll_lean", 270.0)),
            kd_lean_roll=float(p.get("kd_ankle_roll_lean", 68.0)),
            kp_pos_roll=float(p.get("kp_ankle_roll", 4800.0)),
            kd_pos_roll=float(p.get("kd_ankle_roll", 600.0)),
            torque_limit_pitch=float(p.get("ankle_torque_limit", 300.0)),
            torque_limit_roll=float(p.get("ankle_roll_torque_limit", 180.0)),
        )
        # 髋策略
        self.hip_enable = bool(p.get("hip_enable", True))
        self.hip = HipStrategy(
            kp_pos=float(p.get("kp_hip", 480.0)),
            kd_pos=float(p.get("kd_hip", 72.0)),
            torque_limit_pitch=float(p.get("hip_torque_limit", 54.0)),
            torque_limit_roll=float(p.get("hip_roll_torque_limit", 180.0)),
            relax=float(p.get("hip_relax", 0.0)),
        )
        self.kp_hip_roll = float(p.get("kp_hip_roll", 1600.0))
        self.kd_hip_roll = float(p.get("kd_hip_roll", 240.0))
        # 目标质心水平位置；None 表示「以复位时的质心为参考」（推荐）
        self.com_ref = None if p.get("com_ref") is None else np.asarray(p["com_ref"], dtype=float)
        # 诊断量
        self.last = {}

    def reset(self, sim=None):
        """复位控制器状态；传入 sim 时把当前质心位置作为目标参考。"""
        self.last = {}
        self.hip.reset()
        if sim is not None and self.com_ref is None:
            self.com_ref = sim.com_position()[:2].copy()

    # ------------------------------------------------------------------
    def compute(self, sim):
        """按当前状态计算关节力矩修正量 (N·m)。"""
        state = sim.read_state()
        com = state["com"]
        vel = state["com_vel"]
        ref = self.com_ref if self.com_ref is not None else np.zeros(2)
        ex = float(com[0] - ref[0])
        ey = float(com[1] - ref[1])
        vx = float(vel[0])
        vy = float(vel[1])

        # --- 踝策略：躯干倾角 PD（上游原律）+ 质心偏差反馈 ---
        ankle_tau, ankle_roll_tau = self.ankle.compute(
            state["lean_x"], state["lean_y"],
            state["lean_x_rate"], state["lean_y_rate"],
            (ex, ey), (vx, vy))

        # --- 髋策略：质心偏差 + 速度反馈（踝饱和后继续把质心拉回） ---
        hip_tau = hip_roll_tau = 0.0
        if self.hip_enable:
            hip_tau, _ = self.hip.compute((ex, ey), (vx, vy))
            hip_roll_tau = float(np.clip(
                self.kp_hip_roll * ey + self.kd_hip_roll * vy,
                -self.hip.torque_limit_roll, self.hip.torque_limit_roll))

        torque = np.zeros(len(JOINT_NAMES))
        for name, value in (("ankle_pitch_l", ankle_tau), ("ankle_pitch_r", ankle_tau),
                            ("ankle_roll_l", ankle_roll_tau),
                            ("ankle_roll_r", ankle_roll_tau),
                            ("hip_pitch_l", hip_tau), ("hip_pitch_r", hip_tau),
                            ("hip_roll_l", hip_roll_tau), ("hip_roll_r", hip_roll_tau)):
            torque[JOINT_NAMES.index(name)] = value

        self.last = {
            "ex": ex, "ey": ey, "vx": vx, "vy": vy,
            "lean_x": state["lean_x"], "lean_y": state["lean_y"],
            "ankle_tau": ankle_tau, "ankle_roll_tau": ankle_roll_tau,
            "hip_tau": hip_tau, "hip_roll_tau": hip_roll_tau,
            "ankle_saturated": self.ankle.saturated,
            "zmp": state["zmp"], "tilt": state["tilt"],
        }
        return torque
