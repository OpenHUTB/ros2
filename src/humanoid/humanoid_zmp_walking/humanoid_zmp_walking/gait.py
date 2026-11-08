"""基于 LIPM 的 ZMP 步态规划。

设计思路：给定平滑的质心轨迹（侧向余弦摆动 + 前进匀速），由 LIPM 关系

    p = c - (z_c / g) * a

反算 ZMP，再逐点判定 ZMP 是否落在支撑多边形内。支撑多边形只包含已落地
的脚掌：摆动脚在摆动区间内被剔除，因此单脚支撑时段只有一只脚的矩形。

关节运动与落脚时序对齐：每条腿在各自的摆动窗口内抬起，窗口长度等于
t_swing，支撑相关节角为零。
"""

import math

from .footstep import (LEFT, RIGHT, foot_rect, plan_footsteps,
                       zmp_inside_support)
from .lipm import zmp_from_com

# 关节幅度（rad）
HIP_AMPLITUDE = 0.25
KNEE_AMPLITUDE = 0.50
ANKLE_AMPLITUDE = 0.60


def leg_targets(progress):
    """单腿的 3 个关节角；progress 为摆动进度，支撑相传 None。

    摆动期内髋、膝按 sin(pi * progress) 抬起，首末时刻为 0；膝关节只做
    单向屈曲（非负）；踝关节抵消髋与膝的转角，使脚掌尽量保持水平。
    """
    if progress is None:
        return [0.0, 0.0, 0.0]
    lift = math.sin(math.pi * progress)
    hip = HIP_AMPLITUDE * lift
    knee = KNEE_AMPLITUDE * lift
    ankle = max(-ANKLE_AMPLITUDE, min(ANKLE_AMPLITUDE, -hip - knee))
    return [hip, knee, ankle]


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
        做余弦摆动，摆动中心取单脚支撑区间的中点，使质心在摆动腿离地
        时位于支撑脚上方。
        """
        sway_center = (self.t_step - self.t_swing) + self.t_swing / 2.0
        omega_sway = math.pi / self.t_step
        com_x = self.step_length * (t / self.t_step) - self.step_length / 2.0
        com_y = -self.step_width * math.cos(omega_sway * (t - sway_center))
        return com_x, com_y

    def swing_progress(self, t, foot):
        """返回 t 时刻该脚的摆动进度 [0, 1]；处于支撑相时返回 None。"""
        for step in self.steps:
            if step.foot == foot and step.swing_start <= t < step.swing_end:
                return ((t - step.swing_start)
                        / (step.swing_end - step.swing_start))
        return None

    def joint_targets(self, t):
        """返回 t 时刻的 6 个关节目标角。

        顺序为左髋、左膝、左踝、右髋、右膝、右踝。每条腿以各自的摆动
        窗口驱动，因此抬腿窗口与落脚时序一致，支撑腿保持直立。
        """
        left = leg_targets(self.swing_progress(t, LEFT))
        right = leg_targets(self.swing_progress(t, RIGHT))
        return left + right

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
        """返回时刻 t 已落地脚掌的矩形列表。

        每只脚记录当前落点：尚未开始摆动时保留初始位置，摆动区间内整只
        脚被剔除，落地后更新为新落点。
        """
        rects = []
        for foot, initial_y in ((LEFT, self.step_width),
                                (RIGHT, -self.step_width)):
            foot_x, foot_y = 0.0, initial_y
            in_swing = False
            for step in self.steps:
                if step.foot != foot:
                    continue
                if t < step.swing_start:
                    break
                if t < step.swing_end:
                    in_swing = True
                    break
                foot_x, foot_y = step.x, step.y
            if not in_swing:
                rects.append(foot_rect(foot_x, foot_y,
                                       self.foot_length, self.foot_width))
        return rects

    def _check_support(self):
        """逐点判定 ZMP 是否位于支撑多边形内。

        单脚支撑时段必须只剩支撑脚的矩形，否则判定方式本身有误，直接
        抛出异常。
        """
        self.in_support = []
        for zmp, t in zip(self.zmp, self.t):
            rects = self._landed_rects(t)
            in_single = any(step.swing_start <= t < step.swing_end
                            for step in self.steps)
            if in_single and len(rects) != 1:
                raise ValueError(
                    't={:.2f} s 处于单脚支撑区间，但支撑域包含 {} 个矩形'
                    .format(t, len(rects)))
            self.in_support.append(zmp_inside_support(zmp, rects))
