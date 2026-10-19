# -*- coding: utf-8 -*-
"""踝策略（Ankle Strategy）：抵抗扰动时的第一道防线。

改写自（Apache License 2.0）：
    PrageethM1702/Bipedal-Walking-and-Push-Recovery
    https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery
    （原文件 ``balance/ankle_strategy.py``，Copyright (c) the original authors）

原律与本站改动
--------------
原项目（PyBullet 位置控制机器人）的踝策略是一条**躯干倾角 PD 律**：

    d_ankle_pitch = kp_pitch · pitch + kd_pitch · pitch_rate
    d_ankle_roll  = kp_roll  · roll  + kd_roll  · roll_rate

输出是「踝关节目标角偏置」。本站模型是 MuJoCo **力矩控制**机器人，踝 pitch 关节
行程只有 ±0.9 rad，用关节目标表达平衡修正需要下发远超行程的角度、且会被限位截断，
因此本实现有三处必要改动：

  1. **输出改为关节力矩 (N·m)**，限幅同量纲（``ankle_torque_limit`` 等）；
  2. 倾角输入改用躯干轴相对竖直方向的倾斜分量 ``lean_x``（前倾）/ ``lean_y``（左倾）
     及其速率（``sim_core.torso_lean()`` 与 ``LEAN_RATE_FILTER`` 低通后的差分）；
  3. 在倾角项之外**保留质心位置/速度反馈项**（``kp_pos`` / ``kd_pos``）：
     纯倾角 PD 会留下稳态倾角偏差（重力力矩与 PD 力矩的平衡点不在竖直位），
     实测静立漂移达 10~25 mm；加入质心反馈后静立漂移回到 0.65 mm 量级，
     且前向 120 N 推扰的最大偏移由 43.6 mm 降到 34.5 mm。

关于倾角速率项：``lean`` 由旋转矩阵直接差分得到，噪声较大，未低通时哪怕很小的
``kd_lean`` 也会把控制器推向高频震荡（实测 kp=600/kd=300 时静立即倒）。本站默认
对速率做一阶低通（``sim_core.LEAN_RATE_FILTER = 0.7``），并在该前提下整定出
``kp_lean = 300 N·m/rad``、``kd_lean = 75 N·m·s/rad``。

原理：躯干出现小偏差时，踝关节力矩使压力中心（CoP）向倾倒反方向移动，把质心推回
支撑域内。脚掌有限，踝策略的作用范围有限，超过后需要髋策略或跨步。
"""

import numpy as np


class AnkleStrategy:
    """踝策略：输出踝 pitch / roll 关节力矩 (N·m)。

    参数（量纲均为力矩级）:
        kp_lean, kd_lean:   躯干倾角 PD 增益（上游原律），N·m/rad、N·m·s/rad
        kp_pos, kd_pos:     质心位置 / 速度反馈增益（本站整定补充），N·m/m、N·m·s/m
        torque_limit_pitch / torque_limit_roll: 力矩限幅 (N·m)
    """

    def __init__(self, kp_lean=300.0, kd_lean=75.0,
                 kp_pos=5400.0, kd_pos=1440.0,
                 kp_lean_roll=270.0, kd_lean_roll=68.0,
                 kp_pos_roll=4800.0, kd_pos_roll=600.0,
                 torque_limit_pitch=300.0, torque_limit_roll=180.0):
        self.kp_lean = float(kp_lean)
        self.kd_lean = float(kd_lean)
        self.kp_pos = float(kp_pos)
        self.kd_pos = float(kd_pos)
        self.kp_lean_roll = float(kp_lean_roll)
        self.kd_lean_roll = float(kd_lean_roll)
        self.kp_pos_roll = float(kp_pos_roll)
        self.kd_pos_roll = float(kd_pos_roll)
        self.torque_limit_pitch = float(torque_limit_pitch)
        self.torque_limit_roll = float(torque_limit_roll)
        self.saturated = False

    def compute(self, lean_x, lean_y, lean_x_rate, lean_y_rate, com_err_xy, com_vel_xy):
        """返回 ``(tau_ankle_pitch, tau_ankle_roll)``，单位 N·m。

        符号约定（与本站模型实测一致，见 ``test_standalone.py`` 符号自检项）：
          踝 pitch 正力矩 → 质心向 −x（后）移动，故前倾 / 质心偏前时取正力矩；
          踝 roll  正力矩 → 质心向 +y（左）移动，故左倾 / 质心偏左时取**负**力矩。
        """
        ex, ey = float(com_err_xy[0]), float(com_err_xy[1])
        vx, vy = float(com_vel_xy[0]), float(com_vel_xy[1])

        tau_pitch = (self.kp_lean * lean_x + self.kd_lean * lean_x_rate
                     + self.kp_pos * ex + self.kd_pos * vx)
        tau_roll = -(self.kp_lean_roll * lean_y + self.kd_lean_roll * lean_y_rate
                     + self.kp_pos_roll * ey + self.kd_pos_roll * vy)

        tau_pitch = float(np.clip(tau_pitch,
                                  -self.torque_limit_pitch, self.torque_limit_pitch))
        tau_roll = float(np.clip(tau_roll,
                                 -self.torque_limit_roll, self.torque_limit_roll))
        self.saturated = (abs(tau_pitch) >= self.torque_limit_pitch - 1e-9 or
                          abs(tau_roll) >= self.torque_limit_roll - 1e-9)
        return tau_pitch, tau_roll
