#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
麦克纳姆轮运动学解算
底盘速度 (vx m/s, vy m/s, wz rad/s) -> 四轮转速 (fl, fr, rl, rr) rad/s

四轮顺序：[front_left, front_right, rear_left, rear_right]
符号对齐原 mecanum_ctrl.py：纯前进时四个轮子都正转
"""

import rospy
import numpy as np


class MecanumKinematics:
    def __init__(self, wheel_radius=0.08, half_length=0.18, half_width=0.15):
        self.r = wheel_radius
        self.L = half_length
        self.W = half_width
        self.l_plus_w = self.L + self.W

        # 逆运动学矩阵（物理正确，除以轮半径）
        # 纯前进 vx>0：四个轮子都正转 ✓
        self.inv_matrix = (1.0 / self.r) * np.array([
            [ 1, -1, -self.l_plus_w],   # FL 左前
            [ 1,  1,  self.l_plus_w],   # FR 右前
            [ 1,  1, -self.l_plus_w],   # RL 左后
            [ 1, -1,  self.l_plus_w],   # RR 右后
        ])

        rospy.loginfo("麦克纳姆轮运动学初始化 r=%.3f L=%.3f W=%.3f",
                      self.r, self.L, self.W)

    def inverse(self, vx, vy, wz):
        """底盘速度(m/s, m/s, rad/s) -> 四轮转速(rad/s)"""
        v = np.array([vx, vy, wz])
        wheels = self.inv_matrix @ v
        return tuple(wheels)


if __name__ == '__main__':
    rospy.init_node('kinematics_test')
    kin = MecanumKinematics()

    print("纯前进 vx=0.05 m/s (低速):")
    fl, fr, rl, rr = kin.inverse(0.05, 0, 0)
    print("  FL=%.2f FR=%.2f RL=%.2f RR=%.2f rad/s" % (fl, fr, rl, rr))
    print("  (对应原代码 vx=0.05 的量级)")
