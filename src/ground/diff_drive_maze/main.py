#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 差速小车走迷宫仿真。

小车从起点 ``(-2.0, -2.0)`` 出发，沿预设走廊航点依次「原地转向对准 → 直行」，
绕过一道 L 型隔墙（横墙 + 竖墙），到达绿色终点 ``(2.0, 2.0)``。途中与墙壁几何
接触即判定撞墙失败，进入终点附近判定成功。

控制为**闭环**：每段航点按车身 yaw 反馈原地对准航向，直行时叠加航向比例修正，
不像开环硬编码步数那样受轮地打滑与时间累积误差影响。

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
    python3 main.py --headless  # 无头模式：到达终点 / 撞墙 / 超时后打印结果退出
"""
import os
import sys
import math
import time
import argparse

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

# ---- 车体与场景参数（单位 m / rad）-----------------------------------------
WHEEL_RADIUS = 0.05     # 轮半径 m
WHEEL_BASE = 0.22       # 左右轮间距 m
START_X, START_Y = -2.0, -2.0
GOAL_X, GOAL_Y = 2.0, 2.0
# 走廊航点（不含起点、终点）：先向东绕过横墙东端，再向北绕过竖墙北端
WAYPOINTS = [(2.0, -2.0), (2.0, 2.0)]
TIMESTEP = 0.005        # 仿真步长 s，与 XML 中 option timestep 一致

# ---- 控制参数 ---------------------------------------------------------------
CRUISE_SPEED = 0.5      # 直行车体线速度 m/s
TURN_OMEGA = 1.5        # 原地转向车体角速度 rad/s
K_ANGULAR = 3.0         # 直行航向比例增益 (rad/s)/rad
ANGLE_HI = 0.5          # 转入原地转向的角度阈值 rad
ANGLE_LO = 0.3          # 转回直行的角度阈值 rad（滞回下限）
ARRIVE_DIST = 0.12      # 到达判据：距终点小于该距离即停车 m
WP_ARRIVE_DIST = 0.15   # 航点到达判据：距当前航点小于该距离即切换下一段 m
MAX_STEPS = 20000       # 步数上限兜底（dt=0.005 → 100 s），避免死循环

# 墙壁几何名前缀；接触到这些几何即判定撞墙
WALL_PREFIX = "wall"


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


def control_step(x, y, yaw, state):
    """单步决策。

    参数
    ----
    x, y, yaw : 当前车体位置与偏航角
    state : 状态字典 ``{"wp": 当前航点下标, "mode": "turn"/"drive"}``

    返回
    ----
    (left, right, state, arrived, dist_to_goal)
    """
    dist_goal = math.hypot(GOAL_X - x, GOAL_Y - y)
    if dist_goal < ARRIVE_DIST:
        return 0.0, 0.0, state, True, dist_goal

    gx, gy = WAYPOINTS[state["wp"]]
    dx, dy = gx - x, gy - y
    dist_wp = math.hypot(dx, dy)

    # 到达当前航点且未到末段 → 切换到下一段航点，重新进入原地转向
    if dist_wp < WP_ARRIVE_DIST and state["wp"] < len(WAYPOINTS) - 1:
        state["wp"] += 1
        state["mode"] = "turn"
        gx, gy = WAYPOINTS[state["wp"]]
        dx, dy = gx - x, gy - y

    angle_diff = wrap_angle(math.atan2(dy, dx) - yaw)

    # 滞回：当前在转向，误差降到 ANGLE_LO 才回直行；当前直行，误差超 ANGLE_HI 才转
    mode = state["mode"]
    if mode == "turn":
        if abs(angle_diff) <= ANGLE_LO:
            mode = "drive"
    else:
        if abs(angle_diff) > ANGLE_HI:
            mode = "turn"
    state["mode"] = mode

    if mode == "turn":
        v = 0.0
        omega = math.copysign(TURN_OMEGA, angle_diff)
    else:
        v = CRUISE_SPEED
        omega = K_ANGULAR * angle_diff

    left, right = diff_drive_vels(v, omega)
    return left, right, state, False, dist_goal


def _resolve_xml_path():
    """定位 models/diff_drive_maze.xml，兼容源码空间与 install 空间。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "models", "diff_drive_maze.xml"),
        os.path.join(here, "..", "..", "share",
                     "diff_drive_maze", "models", "diff_drive_maze.xml"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("diff_drive_maze")
        candidates.append(os.path.join(share, "models", "diff_drive_maze.xml"))
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError("未找到 diff_drive_maze.xml，已搜索: " + "; ".join(candidates))


def load_model():
    model = mujoco.MjModel.from_xml_path(_resolve_xml_path())
    model.opt.timestep = TIMESTEP
    return model


def _touch_wall(model, data):
    """扫描当前接触对，任一接触几何名以 wall 开头即判定撞墙。"""
    for c in range(data.ncon):
        g1 = model.geom(data.contact[c].geom1).name
        g2 = model.geom(data.contact[c].geom2).name
        if g1.startswith(WALL_PREFIX) or g2.startswith(WALL_PREFIX):
            return True
    return False


def run(model, data, viewer=None, step_sleep=0.0):
    """跑走迷宫闭环，返回结果字典。

    返回字段：``arrived``（是否到达终点）、``collided``（是否撞墙）、``steps``、
    ``final_x``、``final_y``、``final_dist``（距终点距离）。
    """
    state = {"wp": 0, "mode": "turn"}
    step = 0
    arrived = False
    collided = False

    while step < MAX_STEPS:
        x, y = data.qpos[0], data.qpos[1]
        yaw = quat_to_yaw(data.qpos[3:7])

        left, right, state, arrived, dist_goal = control_step(x, y, yaw, state)
        data.ctrl[0] = left
        data.ctrl[1] = right

        mujoco.mj_step(model, data)
        step += 1

        if _touch_wall(model, data):
            collided = True
            break

        if viewer is not None:
            viewer.sync()
            if step_sleep > 0:
                time.sleep(step_sleep)

        if arrived:
            break

    fx, fy = data.qpos[0], data.qpos[1]
    final_dist = math.hypot(GOAL_X - fx, GOAL_Y - fy)
    return {
        "arrived": arrived,
        "collided": collided,
        "steps": step,
        "final_x": fx,
        "final_y": fy,
        "final_dist": final_dist,
    }


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 差速小车走迷宫仿真")
    parser.add_argument("--headless", action="store_true",
                        help="无头模式：到达终点 / 撞墙 / 超时后打印结果退出（无窗口）")
    args = parser.parse_args()

    print("=== MuJoCo 差速小车走迷宫仿真 ===")
    print(f"起点: ({START_X}, {START_Y}) m  终点: ({GOAL_X}, {GOAL_Y}) m  "
          f"到达判据: {ARRIVE_DIST} m")
    model = load_model()
    data = mujoco.MjData(model)

    if args.headless:
        res = run(model, data, viewer=None)
        sim_time = res["steps"] * model.opt.timestep
        if res["collided"]:
            status = "撞墙失败"
        elif res["arrived"]:
            status = "到达终点"
        else:
            status = "超时未到达"
        print(f"结果: {status}")
        print(f"步数: {res['steps']}  仿真时长: {sim_time:.2f} s")
        print(f"终止位置: ({res['final_x']:.3f}, {res['final_y']:.3f}) m  "
              f"距终点: {res['final_dist']:.3f} m")
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("viewer 已打开；小车先转向对准航点，再直行绕过 L 型隔墙。关闭窗口即退出。")
        res = run(model, data, viewer=viewer, step_sleep=TIMESTEP)
        if res["collided"]:
            verdict = "撞墙失败"
        elif res["arrived"]:
            verdict = "到达终点"
        else:
            verdict = "超时未到达"
        print(f"{verdict}，终止位置：({res['final_x']:.3f}, {res['final_y']:.3f}) m，"
              f"距终点 {res['final_dist']:.3f} m，"
              f"耗时 {res['steps'] * model.opt.timestep:.2f} s")
        # 闭环结束后保持窗口，等待用户手动关闭；小睡让出 CPU，避免忙等占满一个核
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
