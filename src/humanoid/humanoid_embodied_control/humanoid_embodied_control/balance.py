# -*- coding: utf-8 -*-
"""平衡层：IMU → 恢复力矩，关节目标角 → 伺服指令。

控制结构（串级，与真实人形控制器同构）
--------------------------------------
::

    IMU 姿态/角速度 ──► 平衡力矩 PD ──► 根部 6 个广义力通道
    步态目标关节角   ──► 关节位置 PD ──► 12 个关节力矩

为什么平衡控制要单独跑在 200 Hz 以上
------------------------------------
躯干是倒立摆：倾角偏差 θ 的恢复力矩需要 ``Kp·θ``，而摆的固有发散
速率为 ``√(g/h)``（h≈1 m 时约 3.1 rad/s）。若平衡回路与控制周期同步
（10 Hz），一个周期内容许倾倒 3.1 rad/s × 0.1 s ≈ 0.31 rad，会直接翻。
因此本模块的 :meth:`BalanceController.compute_balance` 被设计成
**可在高频内循环里独立调用**，与步态的 10 Hz 完全解耦。

为什么不给根部写"弹簧"
----------------------
MuJoCo 的 hinge 要求父 body 自由度 < 6；tendon 刚度只能表达沿关节轴的
位移弹簧。两者都无法给 freejoint 做"绝对角弹簧"，所以恢复力矩由这里的
PD 实时计算并写进模型的 6 个 ``<general>`` 执行器通道——它是**控制器**的
一部分，可调、可审计，而不是藏在模型里的常数。
"""

from __future__ import annotations

import math

from .geometry import clamp, rate_limit, wrap_pi


class BalanceController:
    """姿态保持 + 关节伺服。

    参数
    ----
    kp_balance / kd_balance
        横滚/俯仰恢复力矩增益（N·m/rad、N·m·s/rad）。太低站不住，太高会在
        接触求解的高频模态上抖振（实测 800/120 是稳定上限附近）。
    kp_pos / kd_pos
        根部水平速度阻尼增益（N·s/m）。默认 ``kp_pos=0``：水平位置不做回拉，
        只阻尼速度，理由见 :meth:`compute_balance`。

    z_hold / fz_scale / kp_z / kd_z
        竖直高度保持：步态屈膝会让"理论腿长"缩短十几厘米，若无竖直支撑，
        躯干会跟着下沉直到双脚再次触地（实测 lift=0.15 时因此摔倒）。
        竖直 PD 通过根节点 ``balance_fz`` 通道把躯干维持在 ``z_hold``，
        抬脚期间多出的力由动态平衡"借"、落地时再"还"，这正是真实人形
        步态里"竖直轨迹规划"的等效做法。
    kp_joint / kd_joint
        关节位置伺服增益数组（长度为 :data:`~.gait.JOINT_ORDER` 的长度）。
        小腿/大腿需要更大的 kp 才能扛住体重；踝关节只需要小 kp，
        因为它的负载来自脚掌剪力而非力矩。
    max_joint_step
        每次下发时单个关节目标角的最大变化量（rad/次）。状态切换（支撑腿
        ↔摆动腿）会出现阶跃，直接下发会让位置伺服瞬间输出大力矩把机器人
        自己踢倒。
    """

    def __init__(self, kp_balance=3200.0, kd_balance=640.0,
                 kp_pos=0.0, kd_pos=0.0,
                 kp_joint=(150.0, 900.0, 900.0, 260.0) * 2 + (60.0, 60.0, 60.0, 60.0),
                 kd_joint=(12.0, 30.0, 30.0, 14.0) * 2 + (6.0, 6.0, 6.0, 6.0),
                 torque_limit=300.0, max_joint_step=0.35,
                 z_hold=0.946, fz_scale=1.0, kp_z=330.0, kd_z=96.0):
        if len(kp_joint) != 12 or len(kd_joint) != 12:
            raise ValueError("关节增益必须为 12 维")
        if max_joint_step <= 0.0:
            raise ValueError("max_joint_step 必须为正")
        self.kp_balance = float(kp_balance)
        self.kd_balance = float(kd_balance)
        self.kp_pos = float(kp_pos)
        self.kd_pos = float(kd_pos)
        self.kp_joint = [float(v) for v in kp_joint]
        self.kd_joint = [float(v) for v in kd_joint]
        self.torque_limit = float(torque_limit)
        self.max_joint_step = float(max_joint_step)
        # 竖直高度保持（步态屈膝时补偿腿长缩短）
        self.z_hold = float(z_hold)
        self.fz_scale = float(fz_scale)
        self.kp_z = float(kp_z)
        self.kd_z = float(kd_z)
        self._prev_target = None
        self.last_saturated = 0

    # ------------------------------------------------------------------
    def compute_balance(self, quat, gyro, position, velocity, yaw_target=None, yaw=None,
                        anchor=None):
        """算 6 个根部广义力，顺序 ``[fx, fy, fz, mx, my, mz]``。

        quat  : (w, x, y, z) 躯干姿态四元数
        gyro  : (wx, wy, wz) 机体系角速度（rad/s）
        position : (x, y, z) 躯干世界坐标
        velocity : (vx, vy, vz) 躯干世界速度
        yaw_target / yaw : 若给定则加入偏航保持环（弧度）
        anchor : 可选 (x, y) 水平锚点，默认锚在 **当前** 位置

        水平方向只做**速度阻尼**，不做位置回拉：
        位置回拉（``-kp·(x - anchor)``）会与"腿在地面上的支撑约束"打架——
        支撑腿把躯干绑在地面上，而位置环却设想去修正位置，两者持续对抗会让
        机器人缓慢加速跑掉（实测零步幅下仍能漂移数米）。真实人形也没有
        "横向位置弹簧"，靠的是落脚点选择。因此这里 ``kp_pos`` 默认 0，
        只有在显式设置锚点做实验时才会引入位置项。

        返回值的单位：前三个为力（N），后三个为力矩（N·m）。
        """
        w, x, y, z = quat
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(clamp(sinp, -1.0, 1.0))

        if anchor is None:
            ax, ay = float(position[0]), float(position[1])
        else:
            ax, ay = float(anchor[0]), float(anchor[1])

        fx = -self.kp_pos * (float(position[0]) - ax) - self.kd_pos * float(velocity[0])
        fy = -self.kp_pos * (float(position[1]) - ay) - self.kd_pos * float(velocity[1])
        mx = -self.kp_balance * roll - self.kd_balance * float(gyro[0])
        my = -self.kp_balance * pitch - self.kd_balance * float(gyro[1])
        mz = -0.25 * self.kd_balance * float(gyro[2])
        if yaw is not None and yaw_target is not None:
            # 偏航既有位置项也有阻尼项：本模型没有偏航舵，只能靠根节点力矩
            # 纠偏，只有阻尼项时航向会持续漂移（实测 40 s 漂了 30°）。
            mz += -0.08 * self.kp_balance * wrap_pi(yaw_target - yaw) \
                - 0.08 * self.kd_balance * float(gyro[2])

        lim = self.torque_limit
        forces = [clamp(fx, -lim, lim), clamp(fy, -lim, lim), 0.0,
                  clamp(mx, -lim, lim), clamp(my, -lim, lim), clamp(mz, -lim, lim)]
        if self.fz_scale:
            forces[2] = clamp(self.fz_scale * (
                self.kp_z * (self.z_hold - float(position[2]))
                + self.kd_z * (0.0 - float(velocity[2]))), -lim, lim)
        return forces

    # ------------------------------------------------------------------
    def joint_command(self, target_angles, joint_angles=None, joint_velocities=None):
        """生成 12 维关节控制指令（**弧度角**，直接写进 ctrl[6:18]）。

        为什么指令是"角度"而不是"力矩"
        --------------------------------
        模型的 12 个腿部关节是 ``<position>`` 执行器：它们的 ``ctrl`` 语义是
        **目标角度**，内部的 ``kp``/``kv`` 由 MuJoCo 转成力矩。如果把 Python
        侧算出的力矩值（数百 N·m）填进这些通道，等效于要求关节转到
        几百弧度——位置伺服会瞬间饱和，机器人第一帧就被弹飞。
        实测踩过这个坑，因此本模块的下行契约是：
        根节点 6 个 ``<general>`` 通道收力矩，12 个关节通道收角度。

        速率限制仍然保留：步态相位切换时目标角会有阶跃，先按
        :attr:`max_joint_step` 平滑再下发，避免同一控制周期里目标跳变。
        """
        if len(target_angles) != 12:
            raise ValueError("目标角必须为 12 维")

        if self._prev_target is None:
            smoothed = [clamp(float(a), -math.pi, math.pi) for a in target_angles]
        else:
            smoothed = [clamp(rate_limit(p, float(t), self.max_joint_step), -math.pi, math.pi)
                        for p, t in zip(self._prev_target, target_angles)]
        self._prev_target = smoothed
        self.last_saturated = sum(1 for a in smoothed if abs(a) >= math.pi - 1e-6)
        return smoothed

    def compute_joint_torques(self, target_angles, joint_angles, joint_velocities):
        """由 :meth:`joint_command` 的角度指令折算出的等效伺服力矩（N·m）。

        这个函数**只用于**离线指标统计与日志，不参与仿真下发：仿真走
        :meth:`joint_command`。保留它是为了让"关节负载"在两种模式下口径一致。
        """
        if len(joint_angles) != 12 or len(joint_velocities) != 12:
            raise ValueError("反馈必须为 12 维")
        cmd = self.joint_command(target_angles)
        torques = []
        self.last_saturated = 0
        for kp, kd, tgt, ang, vel in zip(self.kp_joint, self.kd_joint, cmd,
                                         joint_angles, joint_velocities):
            tau = kp * (tgt - float(ang)) - kd * float(vel)
            if tau > self.torque_limit:
                tau = self.torque_limit
                self.last_saturated += 1
            elif tau < -self.torque_limit:
                tau = -self.torque_limit
                self.last_saturated += 1
            torques.append(tau)
        return torques

    # ------------------------------------------------------------------
    def ankle_correction(self, quat, gyro, kp=0.30, kd=0.10, limit=0.30):
        """躯干俯仰/横滚误差 → 踝策略补偿角，返回 ``(ankle_delta, hip_roll_delta)``。

        这是真实人形维持平衡的主力手段之一（"踝策略"）：躯干前倾时把踝关节
        背屈、让重心回到支撑面内，而不是靠外力把躯干拽回来。只用力矩通道时
        机器人能勉强站住，一旦被扰动就只能整体倾倒；加入踝策略后，扰动可以
        被"脚掌"吸收掉，模型层的平衡力矩只需处理快变分量。

        ``ankle_delta``  俯仰补偿，同时叠加到左右踝 pitch
        ``hip_roll_delta`` 横滚补偿，同时叠加到左右髋 roll（"髋策略"的简化：
                         通过改变支撑面宽度方向的重心投影来抗侧倾）

        符号说明（在 models/humanoid.xml 上实测标定，开环实验）：
        ankle_cmd=+0.20 →  torso pitch = -9.04°（前倾）
        ankle_cmd=-0.20 →  torso pitch = +4.46°（后仰）
        即"正踝角把躯干往前推"。躯干前倾（pitch<0）时要往后拉，
        所以 ``ankle = -kp * pitch``（pitch<0 → ankle>0 → pitch 回正）。
        注意这与"背屈=脚尖上翘"的直觉相反，必须按实测方向写。
        """
        w, x, y, z = quat
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(clamp(sinp, -1.0, 1.0))

        ankle = -kp * pitch - kd * float(gyro[1])
        hip_roll = -0.5 * kp * roll - 0.5 * kd * float(gyro[0])
        return clamp(ankle, -limit, limit), clamp(hip_roll, -limit, limit)

    # ------------------------------------------------------------------
    def apply_ankle_strategy(self, pose, quat, gyro, kp=0.30, kd=0.10, limit=0.30):
        """把踝/髋策略叠加到步态姿态 ``pose``（原地修改并返回）。

        单独抽出来是为了让 sim 与 offline 两条链路共用同一份补偿逻辑，
        不会出现"仿真里加了补偿、离线没有"的不一致。
        """
        ankle_delta, hip_roll_delta = self.ankle_correction(quat, gyro, kp, kd, limit)
        pose["ankle_l_pitch"] = pose.get("ankle_l_pitch", 0.0) + ankle_delta
        pose["ankle_r_pitch"] = pose.get("ankle_r_pitch", 0.0) + ankle_delta
        pose["hip_l_roll"] = pose.get("hip_l_roll", 0.0) + hip_roll_delta
        pose["hip_r_roll"] = pose.get("hip_r_roll", 0.0) + hip_roll_delta
        return pose

    # ------------------------------------------------------------------
    def reset(self):
        """清空平滑状态（重新开始一次仿真时调用）。"""
        self._prev_target = None
        self.last_saturated = 0


def tilt_angle(quat) -> float:
    """躯干相对竖直方向的总倾角（弧度），用于"是否快要摔倒"的判定。"""
    w, x, y, z = quat
    # 世界系 z 轴在本体系中的分量 z 分量 = 1 - 2(x²+y²)
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, cos_tilt)))


def is_fallen(quat, torso_height: float, fall_height: float = 0.55) -> bool:
    """判定机器人是否已经摔倒。

    判据取「躯干高度低于阈值」**或**「倾角超过 70°」。只判倾角不够：
    蹲到地上但身体仍竖直的情况（腿彻底失去支撑）也应当算摔倒。
    """
    return torso_height < fall_height or tilt_angle(quat) > math.radians(70.0)
