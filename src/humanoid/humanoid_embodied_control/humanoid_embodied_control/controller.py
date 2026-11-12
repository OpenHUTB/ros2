# -*- coding: utf-8 -*-
"""闭环控制器：把感知、规划、步态、平衡粘合在一起。

本模块是**包内可复用**的控制层，sim / offline 两条链路以及 ROS 2 节点
都从这里导入 :class:`WalkingController`。之所以放在 Python 包内而不是
``scripts/main.py``，是因为 ament_python 安装后 ``scripts/`` 不会被收录进
site-packages，包内节点无法反向导入包外脚本；控制器属于可复用库代码，
应当随包安装。

与后端相关的只有三个调用：速度指令、关节目标角、平衡量下发，由后端适配。
"""

from __future__ import annotations

import math

from humanoid_embodied_control import gait
from humanoid_embodied_control.balance import BalanceController
from humanoid_embodied_control.geometry import clamp, wrap_pi
from humanoid_embodied_control.state_machine import State, TaskStateMachine

DEFAULT_ARM_SWING = 0.25


class WalkingController:
    """把感知、规划、步态、平衡粘合在一起的控制回路。

    设计约束：**sim 与 offline 必须走同一份代码**，否则离线自检就失去意义。
    与后端相关的只有三个调用：速度指令、关节目标角、平衡量下发，由后端适配。
    """

    def __init__(self, goal=(4.5, 0.0), stride=0.45, lift=0.55, period=0.9,
                 speed_max=0.45, lookahead=0.85, obstacle_slow=1.2,
                 obstacle_stop=0.65, avoid_clear=1.0,
                 balance=None, start_pose=None, ankle_kp=0.0):
        self.goal = (float(goal[0]), float(goal[1]))
        self.stride = float(stride)
        self.lift = float(lift)
        self.period = float(period)
        self.speed_max = float(speed_max)
        self.lookahead = float(lookahead)
        # 踝策略默认关闭：实测在本模型上开任何非零 kp（即使只作用在支撑腿）
        # 都会在摆动相造成发散，kb=3200 的根力矩环才是主力稳定手段。
        # ankle_kp>0 仅供对比实验使用。
        self.ankle_kp = float(ankle_kp)
        self.balance = balance or BalanceController()
        self.sm = TaskStateMachine(obstacle_slow=obstacle_slow,
                                   obstacle_stop=obstacle_stop,
                                   avoid_clear=avoid_clear)
        self.phase = 0.0
        self.pose = gait.stand_pose() if start_pose is None else dict(start_pose)
        self.last_cmd = 0.0
        self.last_free = 1.0

    # ------------------------------------------------------------------
    def command(self, t, x, y, yaw, front_clear, goal_distance=None):
        """根据本体状态与感知结果算出速度标量指令（m/s，沿当前朝向）。

        返回 ``(speed, state, turn_bias)``：
        speed    前进速度（可为 0）
        state    当前任务状态
        turn_bias 期望转向（非 0 时由调用方叠加到左侧/右侧步幅上）
        """
        if goal_distance is None:
            goal_distance = math.hypot(self.goal[0] - x, self.goal[1] - y)

        fallen = False
        state = self.sm.update(t, goal_distance, front_clear, fallen=fallen)
        self.sm.tick_avoid(0.0)

        if state in (State.REACHED, State.BLOCKED, State.FALLEN):
            self.last_cmd = 0.0
            return 0.0, state, 0.0

        heading_err = wrap_pi(math.atan2(self.goal[1] - y, self.goal[0] - x) - yaw)
        scale = self.sm.speed_scale(front_clear)
        # 航向误差大连带着降低前进速度（先转正再走），避免"边走边偏"越走越远
        align = max(0.15, math.cos(clamp(abs(heading_err), 0.0, math.pi / 2)))
        speed = self.speed_max * scale * align
        # AVOID 期间不再额外减速：步态相位对速度标量敏感，0.2 s 内
        # 从 0.2 降到 0.06 会让相位推进骤降，单腿支撑时间变长 → 摔倒。
        # 绕行动作由 turn 偏置完成，速度保持不变。
        self.last_cmd = speed
        return speed, state, heading_err

    # ------------------------------------------------------------------
    def gait_target(self, dt, speed, turn_bias=0.0, quat=None, gyro=None):
        """推进步态相位，返回 12 维**关节目标角**（弧度）。

        ``quat``/``gyro`` 给定时会叠加踝/髋平衡补偿——仿真与 offline 两条链路
        必须走同一份补偿，否则离线自检就没有意义。
        """
        rate = gait.phase_rate(max(speed, 1e-6), self.stride, self.period)
        if speed <= 1e-3:
            # 速度接近零时相位停在"双脚并拢"附近，避免原地踏步
            target = gait.blend_pose(self.pose, gait.stand_pose(), min(1.0, 6.0 * dt))
        else:
            self.phase += rate * dt
            target = gait.walk_pose(self.phase, self.stride, self.lift,
                                    arm_swing=DEFAULT_ARM_SWING, lean=0.0)
        if turn_bias:
            # turn_bias 缩放得很小：本模型没有偏航舵，转向只能靠"左右腿不同
            # 摆幅"实现，增益过大立刻造成侧向滑移摔倒（实测 0.25 即发散）。
            target["hip_l_pitch"] = clamp(target["hip_l_pitch"] + 0.05 * turn_bias, -1.0, 0.5)
            target["hip_r_pitch"] = clamp(target["hip_r_pitch"] - 0.05 * turn_bias, -1.0, 0.5)
        if quat is not None and gyro is not None:
            self.balance.apply_ankle_strategy(target, quat, gyro,
                                              kp=self.ankle_kp, kd=self.ankle_kp * 0.18)
        self.pose = target
        return gait.to_vector(target)

    # ------------------------------------------------------------------
    def balance_forces(self, quat, gyro, position, velocity, yaw=None):
        """根节点 6 个广义力。给定 ``yaw`` 时开启偏航保持（指向目标）。"""
        if yaw is not None:
            want_yaw = math.atan2(self.goal[1] - position[1], self.goal[0] - position[0])
        else:
            want_yaw = None
        return self.balance.compute_balance(quat, gyro, position, velocity,
                                            yaw_target=want_yaw, yaw=yaw)

    def joint_command(self, target_vec):
        """12 维关节目标角 → 平滑后的下发指令（弧度，直接写 ctrl[6:18]）。"""
        return self.balance.joint_command(target_vec)

    def joint_torques(self, target_vec, angles, velocities):
        """仅用于指标/日志统计的等效伺服力矩（不参与下发）。"""
        return self.balance.compute_joint_torques(target_vec, angles, velocities)
