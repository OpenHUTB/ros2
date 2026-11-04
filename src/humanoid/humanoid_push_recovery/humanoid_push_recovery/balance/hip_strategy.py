# -*- coding: utf-8 -*-
"""髋策略（Hip Strategy）：超出踝策略权限后的第二道防线。

改写自（Apache License 2.0）：
    PrageethM1702/Bipedal-Walking-and-Push-Recovery
    https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery
    （原文件 ``balance/hip_strategy.py``，Copyright (c) the original authors）

本站改动：
  * 同踝策略，输出由「髋关节目标角偏置」改为**髋关节力矩 (N·m)**（本站为力矩控制
    模型）；
  * 原实现以质心速度为唯一输入（``target = −k·v``）并带一阶松弛滤波；本站模型在
    整定中发现仅用速度项时稳态抗扰不足，因此在速度项之外**保留已整定的质心位置
    项**（等效于原律 + 位置反馈），松弛滤波默认关闭（``relax=0``）以保持已验证的
    动态特性。

原理：髋关节快速屈伸使躯干相对质心产生角动量，角动量变化通过地面反力作用把质心
拉回；也就是人失衡时「抡胳膊/扭腰」的同一原理。本站模型没有独立躯干自由度，
「躯干」即骨盆，因此策略通过左右髋 pitch / roll 的力矩实现。
"""

import numpy as np


class HipStrategy:
    """髋策略：输出髋 pitch / roll 关节力矩 (N·m)。

    参数:
        kp_pos, kd_pos:      质心位置 / 速度反馈增益，N·m/m、N·m·s/m
        torque_limit_pitch / torque_limit_roll: 力矩限幅 (N·m)
        relax:               一阶松弛系数，0 表示关闭（本站默认）
        kp_vel, kd_vel:      向后兼容别名
    """

    def __init__(self, kp_pos=480.0, kd_pos=72.0,
                 torque_limit_pitch=54.0, torque_limit_roll=180.0,
                 relax=0.0, kp_vel=None, kd_vel=None):
        self.kp_pos = float(kp_pos if kp_vel is None else kp_vel)
        self.kd_pos = float(kd_pos if kd_vel is None else kd_vel)
        self.torque_limit_pitch = float(torque_limit_pitch)
        self.torque_limit_roll = float(torque_limit_roll)
        self.relax = float(relax)
        self._tau_pitch = 0.0
        self._tau_roll = 0.0
        self.saturated = False

    def compute(self, com_err_xy, com_vel_xy):
        """返回 ``(tau_hip_pitch, tau_hip_roll)``，单位 N·m。

        ``com_err_xy`` 为质心相对参考位置的水平偏差 (m)，``com_vel_xy`` 为质心水平
        速度 (m/s)。符号约定（与本站模型实测一致）：
          髋 pitch 正力矩 → 质心向 +x（前）移动，故质心偏后 (ex<0) 时取正力矩；
          髋 roll  正力矩 → 质心向 −y（右）移动，故质心偏左 (ey>0) 时取正力矩。
        """
        ex, ey = float(com_err_xy[0]), float(com_err_xy[1])
        vx, vy = float(com_vel_xy[0]), float(com_vel_xy[1])

        target_pitch = -(self.kp_pos * ex + self.kd_pos * vx)
        target_roll = (self.kp_pos * ey + self.kd_pos * vy)

        if self.relax > 0.0:
            self._tau_pitch = self.relax * self._tau_pitch + (1.0 - self.relax) * target_pitch
            self._tau_roll = self.relax * self._tau_roll + (1.0 - self.relax) * target_roll
        else:
            self._tau_pitch, self._tau_roll = target_pitch, target_roll

        self._tau_pitch = float(np.clip(self._tau_pitch,
                                        -self.torque_limit_pitch, self.torque_limit_pitch))
        self._tau_roll = float(np.clip(self._tau_roll,
                                       -self.torque_limit_roll, self.torque_limit_roll))
        self.saturated = (abs(self._tau_pitch) >= self.torque_limit_pitch - 1e-9 or
                          abs(self._tau_roll) >= self.torque_limit_roll - 1e-9)
        return self._tau_pitch, self._tau_roll

    def reset(self):
        self._tau_pitch = 0.0
        self._tau_roll = 0.0
