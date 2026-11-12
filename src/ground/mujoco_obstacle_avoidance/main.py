#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 差速小车避障仿真。

小车从原点 ``(0, 0)`` 出发，沿 ``+x`` 方向驶向绿色目标点 ``(GOAL_X, GOAL_Y)``；
路径上有两个红色圆柱障碍。前向射线测距提前发现障碍，进入**绕行状态机**：

- ``cruise``：朝目标点比例控制（航向误差大时转向对准，误差小直行并叠加航向修正，滞回防抖）；
- 前向射线测得障碍距离小于 ``AHEAD_DIST`` 后进入 ``turn_left``：左转到朝 ``+y`` 侧；
- ``side``：沿 ``+y`` 直行，横向离开障碍所在车道；
- ``turn_right``：转回朝 ``+x``，回到 ``cruise`` 重新对准目标。

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

import numpy as np  # noqa: E402
import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

# ---- 车体与目标参数（单位 m / rad）-----------------------------------------
WHEEL_RADIUS = 0.05     # 轮半径 m
WHEEL_BASE = 0.22       # 左右轮间距 m
GOAL_X = 3.0            # 目标点 x m
GOAL_Y = 0.0            # 目标点 y m
TIMESTEP = 0.005        # 仿真步长 s，与 XML 中 option timestep 一致

# ---- 巡航（go-to-goal）控制参数 -------------------------------------------
CRUISE_SPEED = 0.6      # 直行车体线速度 m/s
TURN_OMEGA = 1.5        # 转向指令角速度 rad/s（差速逆解下发的目标车体角速度；受前两轮被动支撑与轮胎侧滑影响，车体实测偏航速率低于指令值，转向段亦有前移，并非严格原地）
K_ANGULAR = 3.0         # 直行航向比例增益 (rad/s)/rad
ANGLE_HI = 0.5          # 转入转向段的角度阈值 rad
ANGLE_LO = 0.3          # 转回直行的角度阈值 rad（滞回下限）

# ---- 避障状态机参数 ---------------------------------------------------------
AHEAD_DIST = 0.60       # 前向射线小于该距离判定前方有障 m
SIDE_Y = 0.40           # 绕行段抬到的侧向偏移 m（离开障碍车道）
SIDE_SPEED = 0.5        # 绕行段车体线速度 m/s
RAY_MAX_RANGE = 5.0     # 射线最大量程 m
RAY_OFFSET_X = 0.20     # 射线发射点相对车体质心的前向偏移 m

ARRIVE_DIST = 0.12      # 到达判据：距目标点小于该距离即停车 m
MAX_STEPS = 20000       # 步数上限兜底（dt=0.005 → 100 s），避免死循环

OBSTACLE_PREFIX = "obstacle"   # 参与避障判定的障碍 geom 名前缀


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


def _body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def ray_front(model, data, yaw, body_name="chassis"):
    """前向射线测距，返回前方障碍距离 (m)；未命中返回 ``RAY_MAX_RANGE``。

    射线在水平面内按车身航向发射，发射点在车体质心前方 ``RAY_OFFSET_X``；
    用 ``bodyexclude`` 排除自车整棵 body，避免打到自己的轮子。
    """
    bid = _body_id(model, body_name)
    pnt = np.array(data.xpos[bid], dtype=np.float64)
    pnt[0] += RAY_OFFSET_X * math.cos(yaw)
    pnt[1] += RAY_OFFSET_X * math.sin(yaw)
    direction = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)

    geomgroup = np.ones(6, dtype=np.uint8)
    geomid = np.zeros(1, dtype=np.int32)
    geomid[0] = -1
    dist = mujoco.mj_ray(model, data, pnt, direction, geomgroup, 1, bid, geomid)
    if geomid[0] < 0 or dist is None or not np.isfinite(dist) or dist < 0.0:
        return RAY_MAX_RANGE
    return float(min(dist, RAY_MAX_RANGE))


def _cruise_step(x, y, yaw, mode):
    """巡航段 go-to-goal 决策，返回 (left, right, mode, dist)。"""
    dx, dy = GOAL_X - x, GOAL_Y - y
    dist = math.hypot(dx, dy)
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
    return left, right, mode, dist


def control_step(x, y, yaw, state, front_dist):
    """单步决策。

    参数
    ----
    x, y, yaw : 当前车体位置与偏航角
    state : 状态字典，含 ``mode``（``cruise`` / ``turn_left`` / ``side`` /
        ``turn_right``）与 ``cruise_mode``（巡航内 turn/drive）。
    front_dist : 前向射线测得的障碍距离 m。

    返回
    ----
    (left, right, state, arrived, dist)
    """
    dx, dy = GOAL_X - x, GOAL_Y - y
    dist = math.hypot(dx, dy)
    if dist < ARRIVE_DIST:
        return 0.0, 0.0, state, True, dist

    mode = state["mode"]

    if mode == "cruise":
        if front_dist < AHEAD_DIST:
            # 前方有障：左转到朝 +y 侧绕行
            state["mode"] = "turn_left"
            mode = "turn_left"
        else:
            left, right, c_mode, dist = _cruise_step(x, y, yaw, state["cruise_mode"])
            state["cruise_mode"] = c_mode
            return left, right, state, False, dist

    if mode == "turn_left":
        target = math.pi / 2.0   # 朝 +y
        err = wrap_angle(target - yaw)
        if abs(err) <= ANGLE_LO:
            state["mode"] = "side"
            left, right = diff_drive_vels(SIDE_SPEED, 0.0)
        else:
            left, right = diff_drive_vels(0.0, math.copysign(TURN_OMEGA, err))
        return left, right, state, False, dist

    if mode == "side":
        if y >= SIDE_Y:
            # 横向离开车道，转回朝 +x
            state["mode"] = "turn_right"
            mode = "turn_right"
        else:
            left, right = diff_drive_vels(SIDE_SPEED, 0.0)
            return left, right, state, False, dist

    if mode == "turn_right":
        target = 0.0             # 朝 +x
        err = wrap_angle(target - yaw)
        if abs(err) <= ANGLE_LO:
            state["mode"] = "cruise"
            state["cruise_mode"] = "drive"
            left, right, c_mode, dist = _cruise_step(x, y, yaw, "drive")
            state["cruise_mode"] = c_mode
        else:
            left, right = diff_drive_vels(0.0, math.copysign(TURN_OMEGA, err))
        return left, right, state, False, dist

    return 0.0, 0.0, state, False, dist


def _resolve_xml_path():
    """定位 models/obstacle_avoidance.xml，兼容源码空间与 ROS 2 install 空间。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # 源码空间：main.py 与 models/ 同在包根
        os.path.join(here, "models", "obstacle_avoidance.xml"),
    ]
    # install 空间：main.py 装在 lib/<pkg>/，模型装在 share/<pkg>/models/
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("mujoco_obstacle_avoidance")
        candidates.append(os.path.join(
            share_dir, "models", "obstacle_avoidance.xml"))
    except ImportError:
        # 未 source ROS 2 环境（纯脚本直接运行）时跳过 install 空间定位
        pass
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError("未找到 obstacle_avoidance.xml，已搜索: " + "; ".join(candidates))


def load_model():
    model = mujoco.MjModel.from_xml_path(_resolve_xml_path())
    model.opt.timestep = TIMESTEP
    return model


def run(model, data, viewer=None, step_sleep=0.0):
    """跑避障闭环，返回结果字典。"""
    state = {"mode": "cruise", "cruise_mode": "drive"}
    step = 0
    arrived = False
    avoid_triggered = False
    min_obs_dist = float("inf")

    while step < MAX_STEPS:
        x, y = data.qpos[0], data.qpos[1]
        yaw = quat_to_yaw(data.qpos[3:7])

        front_dist = ray_front(model, data, yaw)
        if state["mode"] == "cruise" and front_dist < AHEAD_DIST:
            avoid_triggered = True

        left, right, state, arrived, dist = control_step(x, y, yaw, state, front_dist)
        data.ctrl[0] = left
        data.ctrl[1] = right

        mujoco.mj_step(model, data)
        step += 1

        # 记录到两个障碍的最小中心距（用于测试与结果核对）
        # 用 data.geom_xpos（世界坐标）而非 model.geom(gid).pos（模型静态局部坐标）：
        # 障碍挂在 worldbody 下两者相等，一旦挪进某个 body，局部坐标会算错。
        for gid in range(model.ngeom):
            name = model.geom(gid).name
            if name.startswith(OBSTACLE_PREFIX):
                ox, oy = data.geom_xpos[gid][0], data.geom_xpos[gid][1]
                min_obs_dist = min(min_obs_dist, math.hypot(x - ox, y - oy))

        if viewer is not None:
            viewer.sync()
            if step_sleep > 0:
                time.sleep(step_sleep)

        if arrived:
            break

    fx, fy = data.qpos[0], data.qpos[1]
    final_dist = math.hypot(GOAL_X - fx, GOAL_Y - fy)
    return {
        "arrived": arrived, "steps": step, "fx": fx, "fy": fy,
        "final_dist": final_dist, "avoid_triggered": avoid_triggered,
        "min_obs_dist": min_obs_dist,
    }


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 差速小车避障仿真")
    parser.add_argument("--headless", action="store_true",
                        help="无头模式：到达目标点或超时后打印结果退出（无窗口）")
    args = parser.parse_args()

    print("=== MuJoCo 差速小车避障仿真 ===")
    print(f"目标点: ({GOAL_X}, {GOAL_Y}) m  到达判据: {ARRIVE_DIST} m")
    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if args.headless:
        res = run(model, data, viewer=None)
        sim_time = res["steps"] * model.opt.timestep
        status = "已到达" if res["arrived"] else "超时未到达"
        print(f"结果: {status}")
        print(f"步数: {res['steps']}  仿真时长: {sim_time:.2f} s")
        print(f"终点: ({res['fx']:.3f}, {res['fy']:.3f}) m  距目标: {res['final_dist']:.3f} m")
        print(f"触发避障: {res['avoid_triggered']}  到障碍最小中心距: {res['min_obs_dist']:.3f} m")
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("viewer 已打开；小车直行遇障后左转绕行，再回到目标方向。关闭窗口即退出。")
        res = run(model, data, viewer=viewer, step_sleep=TIMESTEP)
        print(f"{'已到达' if res['arrived'] else '超时未到达'}："
              f"({res['fx']:.3f}, {res['fy']:.3f}) m，距目标 {res['final_dist']:.3f} m，"
              f"耗时 {res['steps'] * model.opt.timestep:.2f} s，"
              f"避障触发: {res['avoid_triggered']}")
        # 闭环结束后保持窗口，等待用户手动关闭
        while viewer.is_running():
            viewer.sync()


if __name__ == "__main__":
    main()
