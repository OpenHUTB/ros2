#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 差速小车 go-to-goal 仿真。

小车从原点 (0, 0) 出发，按分段比例控制驶向目标点 ``(GOAL_X, GOAL_Y)``：

- 航向误差大：原地差速转向，先把车头对准目标；
- 航向误差小：直行并叠加航向比例修正，边走边对准；
- 两档阈值带滞回，避免在切换边界来回抖动（极限环）。

运动学口径
----------
``velocity`` 执行器的 ``ctrl`` 单位是**轮子角速度 (rad/s)**，不是车体线速度。
给定目标车体线速度 ``v`` (m/s) 与车体角速度 ``omega`` (rad/s)，由差速逆解换算：

.. math::
    \\omega_L = \\frac{v}{R} - \\frac{\\omega \\cdot L}{2R},\\quad
    \\omega_R = \\frac{v}{R} + \\frac{\\omega \\cdot L}{2R}

其中 ``R`` 为轮半径，``L`` 为左右轮间距。

用法
----
    python3 main.py              # 打开 MuJoCo viewer 可视化，关闭窗口退出
    python3 main.py --headless   # 无头模式：到达目标点或超时后打印结果退出（无窗口）
"""
import os
import sys
import math
import time
import argparse

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

# ---- 车体与目标参数（单位 m / rad）-----------------------------------------
WHEEL_RADIUS = 0.05     # 轮半径 m
WHEEL_BASE = 0.22       # 左右轮间距 m
GOAL_X = 3.0            # 目标点 x m
GOAL_Y = 2.0            # 目标点 y m
TIMESTEP = 0.005        # 仿真步长 s，与 XML 中 option timestep 一致

# ---- 控制参数 ---------------------------------------------------------------
CRUISE_SPEED = 0.6      # 直行车体线速度 m/s
TURN_OMEGA = 1.5        # 原地转向车体角速度 rad/s
K_ANGULAR = 3.0         # 直行航向比例增益 (rad/s)/rad
ANGLE_HI = 0.5          # 转入原地转向的角度阈值 rad
ANGLE_LO = 0.3          # 转回直行的角度阈值 rad（滞回下限）
ARRIVE_DIST = 0.12      # 到达判据：距目标点小于该距离即停车 m
MAX_STEPS = 20000       # 步数上限兜底（dt=0.005 → 100 s），避免死循环


def wrap_angle(angle):
    """把角度归一化到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def quat_to_yaw(quat):
    """从自由关节四元数 ``[qw, qx, qy, qz]`` 解算绕 z 轴偏航角 yaw。

    注意：``<freejoint/>`` 的 7 个 qpos 是 ``[x, y, z, qw, qx, qy, qz]``，
    ``qpos[5]`` 是四元数 qy，不是 yaw；绕 z 轴转动时 qy 恒为 0，
    必须由四元数分量解算。
    """
    qw, qx, qy, qz = quat
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def diff_drive_vels(v, omega):
    """差速逆解：车体线速度 v、角速度 omega → 左右轮角速度 (rad/s)。"""
    wl = v / WHEEL_RADIUS - omega * WHEEL_BASE / (2 * WHEEL_RADIUS)
    wr = v / WHEEL_RADIUS + omega * WHEEL_BASE / (2 * WHEEL_RADIUS)
    return wl, wr


def control_step(x, y, yaw, mode):
    """单步决策。

    参数
    ----
    x, y, yaw : 当前车体位置与偏航角
    mode : 当前模式（``"turn"`` / ``"drive"``），用于滞回判断

    返回
    ----
    (left, right, mode, arrived, angle_diff, dist)
    """
    dx, dy = GOAL_X - x, GOAL_Y - y
    dist = math.hypot(dx, dy)
    if dist < ARRIVE_DIST:
        return 0.0, 0.0, "arrived", True, 0.0, dist

    angle_diff = wrap_angle(math.atan2(dy, dx) - yaw)

    # 滞回：当前在转向，误差降到 ANGLE_LO 才回直行；当前直行，误差超 ANGLE_HI 才转
    if mode == "turn":
        if abs(angle_diff) <= ANGLE_LO:
            mode = "drive"
    else:
        if abs(angle_diff) > ANGLE_HI:
            mode = "turn"

    if mode == "turn":
        v = 0.0
        omega = math.copysign(TURN_OMEGA, angle_diff)
    else:
        v = CRUISE_SPEED
        omega = K_ANGULAR * angle_diff

    left, right = diff_drive_vels(v, omega)
    return left, right, mode, False, angle_diff, dist


def _resolve_xml_path():
    """定位 models/diff_drive.xml，兼容源码空间、devel 与 install 空间。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "models", "diff_drive.xml"),
        os.path.join(here, "..", "..", "share",
                     "mujoco_diff_drive_go_to_goal", "models", "diff_drive.xml"),
    ]
    ros_pp = os.environ.get("ROS_PACKAGE_PATH")
    if ros_pp:
        for base in ros_pp.split(os.pathsep):
            candidates.append(os.path.join(
                base, "mujoco_diff_drive_go_to_goal", "models", "diff_drive.xml"))
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError("未找到 diff_drive.xml，已搜索: " + "; ".join(candidates))


def load_model():
    model = mujoco.MjModel.from_xml_path(_resolve_xml_path())
    model.opt.timestep = TIMESTEP
    return model


def run(model, data, viewer=None, step_sleep=0.0):
    """跑 go-to-goal 闭环，返回 (是否到达, 步数, 触发时 x, 触发时 y, 触发时距目标距离)。"""
    mode = "drive"
    step = 0
    arrived = False
    while step < MAX_STEPS:
        x, y = data.qpos[0], data.qpos[1]
        yaw = quat_to_yaw(data.qpos[3:7])

        left, right, mode, arrived, angle_diff, dist = control_step(x, y, yaw, mode)
        data.ctrl[0] = left
        data.ctrl[1] = right

        mujoco.mj_step(model, data)
        step += 1

        if viewer is not None:
            viewer.sync()
            if step_sleep > 0:
                time.sleep(step_sleep)

        if arrived:
            break

    fx, fy = data.qpos[0], data.qpos[1]
    final_dist = math.hypot(GOAL_X - fx, GOAL_Y - fy)
    return arrived, step, fx, fy, final_dist


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 差速小车 go-to-goal 仿真")
    parser.add_argument("--headless", action="store_true",
                        help="无头模式：到达目标点或超时后打印结果退出（无窗口）")
    args = parser.parse_args()

    print("=== MuJoCo 差速小车 go-to-goal 仿真 ===")
    print(f"目标点: ({GOAL_X}, {GOAL_Y}) m  到达判据: {ARRIVE_DIST} m")
    model = load_model()
    data = mujoco.MjData(model)

    if args.headless:
        arrived, steps, fx, fy, final_dist = run(model, data, viewer=None)
        sim_time = steps * model.opt.timestep
        status = "已到达" if arrived else "超时未到达"
        print(f"结果: {status}")
        print(f"步数: {steps}  仿真时长: {sim_time:.2f} s")
        print(f"判据触发位置: ({fx:.3f}, {fy:.3f}) m  距目标: {final_dist:.3f} m")
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("viewer 已打开；小车先转向对准目标，再直行逼近。关闭窗口即退出。")
        arrived, steps, fx, fy, final_dist = run(
            model, data, viewer=viewer, step_sleep=TIMESTEP)
        print(f"{'已到达' if arrived else '超时未到达'}，判据触发位置："
              f"({fx:.3f}, {fy:.3f}) m，距目标 {final_dist:.3f} m，"
              f"耗时 {steps * model.opt.timestep:.2f} s")
        # 闭环结束后保持窗口，等待用户手动关闭；小睡让出 CPU，避免忙等占满一个核
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
