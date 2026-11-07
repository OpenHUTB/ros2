"""基于 LIPM 的 ZMP 步态规划。

设计思路：给定平滑的质心轨迹（侧向余弦摆动 + 前进匀速），由 LIPM 关系

    p = c - (z_c / g) * a

反算 ZMP，再逐点判定 ZMP 是否落在支撑多边形内。由于侧向摆动的角频率
被限制在 0.5 Hz 量级、前进方向加速度为零，ZMP 始终位于脚掌矩形并集内，
属于准静态 ZMP 步行。
"""

import math

from .footstep import (LEFT, RIGHT, foot_rect, plan_footsteps,
                       zmp_inside_support)
from .lipm import zmp_from_com

# 关节幅度（rad）
HIP_AMPLITUDE = 0.25
KNEE_AMPLITUDE = 0.50
ANKLE_AMPLITUDE = 0.60


def joint_targets(phase):
    """由步态相位生成 6 个关节目标角。

    顺序为左髋、左膝、左踝、右髋、右膝、右踝。左右髋反相；膝关节只做
    单向屈曲（非负）且与髋相位错开；踝关节抵消髋与膝的转角。
    """
    value = math.sin(phase)
    left_hip = HIP_AMPLITUDE * value
    right_hip = -HIP_AMPLITUDE * value
    left_knee = KNEE_AMPLITUDE * max(0.0, value)
    right_knee = KNEE_AMPLITUDE * max(0.0, -value)
    left_ankle = max(-ANKLE_AMPLITUDE,
                     min(ANKLE_AMPLITUDE, -left_hip - left_knee))
    right_ankle = max(-ANKLE_AMPLITUDE,
                      min(ANKLE_AMPLITUDE, -right_hip - right_knee))
    return [left_hip, left_knee, left_ankle,
            right_hip, right_knee, right_ankle]


class GaitPlan:
    """准静态 ZMP 步态的规划结果。

    属性：
        t           : 采样时间（s）
        com         : 质心水平位置，(x, y) 序列
        zmp         : 由质心反算的 ZMP，(x, y) 序列
        in_support  : 每个采样点 ZMP 是否落在支撑多边形内
        duration    : 规划总时长（s）
    """

    def __init__(self,
                 com_height=0.65,
                 num_steps=6,
                 step_length=0.10,
                 step_width=0.06,
                 t_step=1.0,
                 t_swing=0.40,
                 foot_length=0.14,
                 foot_width=0.12,
                 dt=0.01):
        self.com_height = com_height
        self.step_length = step_length
        self.step_width = step_width
        self.t_step = t_step
        self.t_swing = t_swing
        self.foot_length = foot_length
        self.foot_width = foot_width
        self.dt = dt
        self.steps = plan_footsteps(num_steps, step_length, step_width,
                                    t_step - t_swing, t_swing, t_step)
        self.duration = num_steps * t_step
        self.t = []
        self.com = []
        self.zmp = []
        self.in_support = []
        self._sample()
        self._check_support()

    def com_at(self, t):
        """给定时刻的质心水平位置 (x, y)。

        前进方向匀速（加速度为 0，ZMP 在 x 上等于质心）；侧向按步态周期
        做余弦摆动，使质心在摆动腿离地时位于支撑脚上方。
        """
        sway_center = self.t_swing + (self.t_step - self.t_swing) / 2.0
        omega_sway = math.pi / self.t_step
        com_x = self.step_length * (t / self.t_step) - self.step_length / 2.0
        com_y = -self.step_width * math.cos(omega_sway * (t - sway_center))
        return com_x, com_y

    def _sample(self):
        """采样质心轨迹，并用二阶差分反算 ZMP。

        内部点用中心差分，首尾点用三点单侧公式（二阶精度），避免端点
        把速度误当作加速度而放大 1/dt 倍。
        """
        count = int(round(self.duration / self.dt)) + 1
        for i in range(count):
            t = i * self.dt
            self.t.append(t)
            self.com.append(self.com_at(t))
        for i in range(count):
            if i == 0:
                acc = self._one_sided_acc(self.com[0], self.com[1],
                                          self.com[2], self.com[3])
            elif i == count - 1:
                acc = self._one_sided_acc(self.com[-1], self.com[-2],
                                          self.com[-3], self.com[-4])
            else:
                prev_c, next_c = self.com[i - 1], self.com[i + 1]
                acc = ((next_c[0] - 2.0 * self.com[i][0] + prev_c[0])
                       / self.dt ** 2,
                       (next_c[1] - 2.0 * self.com[i][1] + prev_c[1])
                       / self.dt ** 2)
            self.zmp.append(zmp_from_com(self.com[i], acc,
                                         self.com_height))

    def _one_sided_acc(self, c0, c1, c2, c3):
        """三点单侧二阶差分：(2c0 - 5c1 + 4c2 - c3) / dt^2。"""
        return ((2.0 * c0[0] - 5.0 * c1[0] + 4.0 * c2[0] - c3[0])
                / self.dt ** 2,
                (2.0 * c0[1] - 5.0 * c1[1] + 4.0 * c2[1] - c3[1])
                / self.dt ** 2)

    def _landed_rects(self, t):
        """返回时刻 t 已落地脚掌的矩形列表。"""
        pos = {LEFT: (0.0, self.step_width),
               RIGHT: (0.0, -self.step_width)}
        for step in self.steps:
            if t >= step.swing_end:
                pos[step.foot] = (step.x, step.y)
        return [foot_rect(p[0], p[1], self.foot_length, self.foot_width)
                for p in pos.values()]

    def _check_support(self):
        """逐点判定 ZMP 是否位于支撑多边形内。"""
        self.in_support = [
            zmp_inside_support(zmp, self._landed_rects(t))
            for zmp, t in zip(self.zmp, self.t)]
