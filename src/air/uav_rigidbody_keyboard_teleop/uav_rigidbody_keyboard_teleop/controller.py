#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥控稳定环：杆量指令 -> 合力/合力矩（高度保持 + 姿态 PD 稳定）。

外环：油门杆量转化为垂速期望，高度环补偿总推力；
内环：姿态杆量转化为目标倾斜角，姿态 PD 环输出三轴力矩。
控制器本身不依赖 ROS，可在离线仿真中直接运行。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dynamics import QuadParams, Quadrotor


@dataclass
class ControllerParams:
    kp_z: float = 1.0       # 高度环增益（保留参数位，垂速前馈）
    kp_vz: float = 1.2       # 垂速环：垂速误差 -> 总推力增量
    kp_att: float = 12.0    # 姿态环：姿态角误差 -> 力矩
    kd_att: float = 0.35    # 姿态角速度阻尼
    max_tilt: float = 0.25  # 最大允许倾斜角 rad（约 14°）
    max_yaw_rate: float = 1.5


class TeleopController:
    """高度保持 + 姿态 PD 稳定控制器。"""

    def __init__(self, params: ControllerParams | None = None,
                 quad_params: QuadParams | None = None):
        self.p = params or ControllerParams()
        self.robot = Quadrotor(quad_params or QuadParams())

    def update(self, state, sticks: dict) -> dict:
        """根据当前状态与杆量指令计算合力/合力矩。

        参数:
            state: 当前 State。
            sticks: 杆量字典，throttle/pitch/roll/yaw 取值 [-1, 1]。

        返回:
            wrench 字典：thrust, tau_x, tau_y, tau_z。
        """
        cp = self.p

        # 高度保持：油门杆量 -> 垂速期望，垂速误差补偿总推力
        vz_des = float(np.clip(sticks["throttle"], -1.0, 1.0)) * 1.2
        thrust = self.robot.hover_thrust() + cp.kp_vz * (vz_des - state.vel[2])

        # 姿态目标：杆量 -> 目标倾斜角 / 偏航速率
        pitch_target = float(np.clip(sticks["pitch"], -1.0, 1.0)) * cp.max_tilt
        roll_target = float(np.clip(sticks["roll"], -1.0, 1.0)) * cp.max_tilt
        yaw_rate_target = float(np.clip(sticks["yaw"], -1.0, 1.0)) * cp.max_yaw_rate

        tau_x = cp.kp_att * (roll_target - state.rpy[0]) \
            - cp.kd_att * state.angular_vel[0]
        tau_y = cp.kp_att * (pitch_target - state.rpy[1]) \
            - cp.kd_att * state.angular_vel[1]
        # 偏航通道：阻尼偏航角速度，偏航杆量作为偏航速率目标前馈
        tau_z = -cp.kd_att * state.angular_vel[2] \
            + 0.05 * (yaw_rate_target - state.angular_vel[2])

        return {"thrust": float(thrust), "tau_x": float(tau_x),
                "tau_y": float(tau_y), "tau_z": float(tau_z)}
