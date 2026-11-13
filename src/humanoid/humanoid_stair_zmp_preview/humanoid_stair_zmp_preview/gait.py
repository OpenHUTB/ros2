# -*- coding: utf-8 -*-
"""步态生成：落脚点序列 -> ZMP 参考 -> CoM 轨迹 -> 关节角（解析 IK）。

平面模型（矢状面），角度约定：
    髋角 θ 使大腿从竖直向前（+x）摆；
    膝角 φ 为屈曲量（0 = 伸直）；
    脚踝角 ψ 让脚底保持水平。

IK 用两连杆解析解，并用正向运动学反验（见 :func:`foot_from_angles`）——
测试里断言往返误差 < 1 cm，避免「符号写反了但看着像能走」这种最难查的错。
"""

import math

import numpy as np

import humanoid_stair_zmp_preview.config as config
import humanoid_stair_zmp_preview.zmp as zmp


def footholds(count=config.STEP_COUNT, step_height=config.STEP_HEIGHT,
              step_depth=config.STEP_DEPTH, direction="up"):
    """返回 [(x, z, leg), ...]：每只脚的落脚点。

    第一级台阶前缘在 ``STAIR_X0``；双脚交替，每步前进 ``step_depth``、升高 ``step_height``
    （下台阶取负）。起点前先放一个平地落脚点，避免第一步就要抬腿。
    """
    # 落脚点高度必须与场景踏面一致：上台阶 k*h，下台阶 (n-k)*h（从平台降到地面）
    z0 = 0.0 if direction == "up" else count * step_height
    out = [(-0.10, z0, "r"), (-0.10, z0, "l")]
    for k in range(count + 1):
        x = config.STAIR_X0 + k * step_depth
        z = k * step_height if direction == "up" else (count - k) * step_height
        leg = "l" if k % 2 == 0 else "r"
        out.append((x, z, leg))
    # 收尾：两条腿都要站到最后一个踏面，否则躯干取「双脚较低者」会停错高度
    last_leg = out[-1][2]
    out.append((out[-1][0], out[-1][1], "r" if last_leg == "l" else "l"))
    return out


def zmp_reference(steps, dt=config.DT_ZMP, ramp=0.25):
    """由落脚点序列生成 ZMP 参考（沿 x）。

    单脚支撑期 ZMP 保持在支撑脚下方；换脚时用 ``ramp`` 秒线性过渡，
    而不是阶跃 —— 真实落脚点的 ZMP 是在脚掌内平滑移动的。
    """
    T = config.CYCLE_TIME
    total = T * len(steps) + 1.0
    n = int(round(total / dt))
    ref = np.zeros(n)
    for k in range(n):
        t = k * dt
        idx = min(int(t / T), len(steps) - 1)
        xk = steps[idx][0]
        if idx == 0:
            ref[k] = xk
            continue
        xp = steps[idx - 1][0]
        local = t - idx * T
        a = min(1.0, local / ramp) if ramp > 0 else 1.0
        ref[k] = xp + (xk - xp) * a
    return ref


def foot_trajectory(steps, dt=config.DT_ZMP):
    """每只脚的位置时间序列（摆动相用**梯形剖面**）。

    摆动分三段：① 竖直抬脚 ② 水平移动到目标 x ③ 竖直落到目标 z。
    早期用的是「直线插值 + 正弦抬升」，脚尖会**从侧面扫进台阶立面**——
    实测约一半的帧穿透 >20 mm，而且把抬脚高度从 0.06 加到 0.14 完全没变化
    （说明不是高度不够，是水平方向扫过去了）。梯形剖面把水平位移放在最高点完成。
    """
    T = config.CYCLE_TIME
    n = int(round((T * len(steps) + 1.0) / dt))
    out = {"l": np.zeros((n, 2)), "r": np.zeros((n, 2))}
    cur = {"l": np.array(steps[0][:2], dtype=float),
           "r": np.array(steps[1][:2], dtype=float)}
    for k in range(n):
        t = k * dt
        idx = min(int(t / T), len(steps) - 1)
        _, _, swing_leg = steps[idx]
        local = t - idx * T
        s = local / T
        for leg in ("l", "r"):
            if leg != swing_leg or idx == 0 or s >= config.SWING_RATIO:
                out[leg][k] = cur[leg]
                continue
            p0 = cur[leg].copy()
            p1 = np.array(steps[idx][:2], dtype=float)
            u = s / config.SWING_RATIO              # 0..1
            # 抬脚高度：必须越过台阶高度 + 脚厚余量，而不是固定 0.06
            H = abs(p1[1] - p0[1]) + config.STEP_LIFT

            def smooth(a):
                return 10 * a ** 3 - 15 * a ** 4 + 6 * a ** 5

            if u < 0.3:                              # ① 竖直抬脚
                z = p0[1] + H * smooth(u / 0.3)
                x = p0[0]
            elif u < 0.7:                            # ② 水平移动（最高点）
                a = (u - 0.3) / 0.4
                z = p0[1] + H
                x = p0[0] + (p1[0] - p0[0]) * smooth(a)
            else:                                    # ③ 竖直落下
                a = (u - 0.7) / 0.3
                z = (p0[1] + H) + (p1[1] - (p0[1] + H)) * smooth(a)
                x = p1[0]
            out[leg][k] = np.array([x, z])
            if s >= config.SWING_RATIO * 0.999:
                cur[leg] = p1
        # 摆动结束才更新 cur，避免中途被当成已到位
        for leg in ("l", "r"):
            if leg == swing_leg and idx > 0 and s >= config.SWING_RATIO:
                cur[leg] = np.array(steps[idx][:2], dtype=float)
    return out


def ik_leg(x_hip, z_hip, x_foot, z_foot):
    """两连杆解析 IK，返回 (hip, knee)，脚底水平由踝角单独补偿。"""
    dx = x_foot - x_hip
    dz = z_foot - z_hip
    l1, l2 = config.THIGH, config.SHANK
    d = math.hypot(dx, dz)
    d = min(max(d, 0.10), l1 + l2 - 1e-3)
    # 膝：余弦定理
    cos_knee = (l1 * l1 + l2 * l2 - d * d) / (2 * l1 * l2)
    knee = math.pi - math.acos(max(-1.0, min(1.0, cos_knee)))
    # 大腿相对竖直的角度（正 = 向前）
    alpha = math.atan2(dx, -dz)
    cos_beta = (l1 * l1 + d * d - l2 * l2) / (2 * l1 * d)
    beta = math.acos(max(-1.0, min(1.0, cos_beta)))
    hip = alpha + beta
    return hip, knee


def foot_from_angles(x_hip, z_hip, hip, knee):
    """正向运动学：由髋/膝角算脚踝位置（用于反验 IK）。"""
    l1, l2 = config.THIGH, config.SHANK
    # 大腿末端
    x1 = x_hip + l1 * math.sin(hip)
    z1 = z_hip - l1 * math.cos(hip)
    # 小腿相对大腿再转 -knee
    th2 = hip - knee
    x2 = x1 + l2 * math.sin(th2)
    z2 = z1 - l2 * math.cos(th2)
    return x2, z2


def ankle_angle(hip, knee):
    """让脚底保持水平。

    实测脚的世界俯仰角 = hip + knee + ankle（帧 639 上三者代数和 1.014 rad = 58.1°，
    与 MuJoCo 里量到的脚体倾角 58.0° 吻合）。因此要脚底水平：

        ankle = -(hip + knee)

    早期写成 ``-(hip - knee)``，差了一个 ``2*knee``（约 1.0 rad ≈ 58°），
    脚掌长期是翘着插进地面的。
    """
    return -(hip + knee)


def plan(step_height=config.STEP_HEIGHT, step_depth=config.STEP_DEPTH,
         count=config.STEP_COUNT, direction="up"):
    """生成完整计划：ZMP 参考、CoM 轨迹、双脚轨迹、关节角序列。"""
    steps = footholds(count, step_height, step_depth, direction)
    ref = zmp_reference(steps)
    res = zmp.simulate(ref, z_c=config.ZC, dt=config.DT_ZMP,
                       n_preview=config.N_PREVIEW, q_x=config.Q_X,
                       q_e=config.Q_E, r=config.R_JERK)
    com = res["com"]
    feet = foot_trajectory(steps)
    n = len(com)
    joints = {}
    for leg in ("l", "r"):
        hip = np.zeros(n); knee = np.zeros(n); ank = np.zeros(n)
        for k in range(n):
            fx, fz_sole = feet[leg][k]
            # IK 解出的是**踝**的位置，而落脚点给的是脚底高度：
            # 脚 geom 挂在踝下方，底面比踝低 FOOT_H，所以踝要抬 FOOT_H。
            # 漏掉这一项会让脚底直接扎进台阶（实测穿透 -265 mm 起）。
            fz = fz_sole + config.FOOT_H
            # 髋高 = 落脚点高度 + 标称髋高 - 固定屈膝量。
            # 早期写成 z_hip = HIP_Z + fz（与落脚点同步升降），髋到脚距离恒为 0.80
            # = 大腿+小腿长度，腿永远接近伸直、膝角恒定 0.10，上下台阶在关节空间里
            # 完全没区别。加上 CROUCH 后腿才会真正弯曲。
            # z_hip 用**脚底**高度：fz 已含 FOOT_H（踝高），若再用它算髋高，
            # 就与 IK 目标重复计入一个 FOOT_H，实测误差恒为 40 mm。
            z_hip = config.HIP_Z + fz_sole - config.CROUCH
            h, kn = ik_leg(com[k], z_hip, fx, fz)
            hip[k] = h; knee[k] = kn; ank[k] = ankle_angle(h, kn)
        joints[leg] = dict(hip=hip, knee=knee, ankle=ank)
    return dict(steps=steps, zmp_ref=ref, com=com, zmp=res["zmp"],
                feet=feet, joints=joints, dt=config.DT_ZMP, direction=direction,
                step_height=step_height, step_depth=step_depth)