#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 差速小车沿墙巡线仿真（矩形闭环走廊，四轮底盘）。

小车从起点 ``(-1.45, -1.45)`` 出发，沿预设矩形航点逆时针绕闭环走廊一圈，
回到起点附近。控制为**航点闭环 + 右侧射线贴墙微调 + 前向防撞限速**：

- 每段航点先按车身 yaw 反馈转向对准航向，滞回放行后直行并叠加航向比例修正；
  到航点半径内切下一段，末航点回到起点；
- 右侧射线测贴墙距离，直行段叠加侧距比例修正把车保持在目标偏移上；
- 前向射线距墙过近时限速保护；接触墙壁几何即判撞墙失败。

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
    python3 main.py --headless   # 无头模式：绕完一圈 / 撞墙 / 超时后打印结果退出
"""
import os
import math
import time
import argparse

os.environ.setdefault("MUJOCO_GL", "glfw")

import numpy as np  # noqa: E402
import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

# ---- 车体参数（单位 m / rad）---------------------------------------------
WHEEL_RADIUS = 0.05     # 轮半径 m
WHEEL_BASE = 0.22       # 左右轮间距 m
TIMESTEP = 0.005        # 仿真步长 s，与 XML 中 option timestep 一致

# ---- 航点（逆时针绕矩形一圈，末航点回到起点）------------------------------
START_X, START_Y = -1.45, -1.45
WAYPOINTS = [
    (1.45, -1.45),
    (1.45, 1.45),
    (-1.45, 1.45),
    (-1.45, -1.45),
]

# ---- 控制参数 -------------------------------------------------------------
CRUISE_SPEED = 0.45     # 直行车体线速度 m/s
TURN_OMEGA = 1.5        # 转向指令角速度 rad/s（差速逆解下发的目标车体角速度；受轮胎侧滑影响，车体实测偏航速率低于指令值）
K_ANGULAR = 3.0         # 直行航向比例增益 (rad/s)/rad
ANGLE_HI = 0.5          # 转入转向段的角度阈值 rad
ANGLE_LO = 0.3          # 转回直行的角度阈值 rad（滞回下限）
WP_ARRIVE_DIST = 0.18    # 中间航点到达半径 m
END_ARRIVE_DIST = 0.10  # 末航点（回起点）到达半径 m，收紧避免提前判停
D_DES = 0.28            # 目标侧距 m：射线原点到墙；车体中心到墙约 0.43 m（+RAY_OFFSET_X）
KP_SIDE = 2.8           # 侧距比例增益
SAFE_FRONT = 0.30       # 前向防撞限速阈值 m
RAY_MAX_RANGE = 5.0     # 射线最大量程 m
RAY_OFFSET_X = 0.15     # 射线原点沿射线方向外移 m：前向射线在质心前 0.15，右侧射线在质心右侧 0.15
FILTER_ALPHA = 0.5      # 侧距一阶低通滤波系数
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


def control_step(x, y, yaw, state, front_dist, side_dist):
    """单步决策。

    参数
    ----
    x, y, yaw : 当前车体位置与偏航角
    state : 状态字典 ``{"wp": 当前航点下标, "mode": "turn"/"drive",
        "side_filt": 滤波后侧距}``
    front_dist : 前向射线测得的墙距 m（未命中为 RAY_MAX_RANGE）
    side_dist : 右侧射线测得的贴墙距离 m（未命中为 RAY_MAX_RANGE）

    返回
    ----
    (left, right, state, arrived, dist_start)
    """
    gx, gy = WAYPOINTS[state["wp"]]
    dx, dy = gx - x, gy - y
    dist_wp = math.hypot(dx, dy)
    is_end = state["wp"] == len(WAYPOINTS) - 1

    # 末航点回到起点附近即停车，两轮速度清零
    if is_end and dist_wp < END_ARRIVE_DIST:
        return 0.0, 0.0, state, True, dist_wp

    # 到达当前航点且未到末段 → 切换下一段航点，重新进入转向
    if dist_wp < WP_ARRIVE_DIST and not is_end:
        state["wp"] += 1
        state["mode"] = "turn"
        gx, gy = WAYPOINTS[state["wp"]]
        dx, dy = gx - x, gy - y
        dist_wp = math.hypot(dx, dy)
        is_end = state["wp"] == len(WAYPOINTS) - 1

    # 侧距一阶低通滤波；未命中保持上一帧滤波值
    if side_dist < RAY_MAX_RANGE:
        state["side_filt"] = FILTER_ALPHA * side_dist + \
            (1 - FILTER_ALPHA) * state["side_filt"]
    side_filt = state["side_filt"]

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
        err = D_DES - side_filt if side_filt < RAY_MAX_RANGE else 0.0
        err = max(-0.15, min(0.15, err))
        omega = K_ANGULAR * angle_diff + KP_SIDE * err
        if front_dist < SAFE_FRONT:
            v = 0.15

    arrived = is_end and dist_wp < END_ARRIVE_DIST
    left, right = diff_drive_vels(v, omega)
    return left, right, state, arrived, dist_wp


def _body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def _ray_distance(model, data, yaw, rel_angle, bid):
    """相对车身方位 rel_angle 发水平射线；未命中返回 RAY_MAX_RANGE。

    射线原点 = 质心沿该射线方向外移 RAY_OFFSET_X，不是统一的向前偏移：
    前向射线（0 度）在质心前 0.15 m；右侧射线（-90 度）在质心右侧 0.15 m。
    因此侧向读数 d 表示射线原点到墙，车体中心到墙约为 d + RAY_OFFSET_X。
    """
    pnt = np.array(data.xpos[bid], dtype=np.float64)
    a = yaw + rel_angle
    pnt[0] += RAY_OFFSET_X * math.cos(a)
    pnt[1] += RAY_OFFSET_X * math.sin(a)
    direction = np.array([math.cos(a), math.sin(a), 0.0])
    geomgroup = np.ones(6, dtype=np.uint8)
    geomid = np.zeros(1, dtype=np.int32)
    geomid[0] = -1
    dist = mujoco.mj_ray(model, data, pnt, direction, geomgroup, 1, bid, geomid)
    if geomid[0] < 0 or dist is None or not np.isfinite(dist) or dist < 0:
        return RAY_MAX_RANGE
    return float(min(dist, RAY_MAX_RANGE))


def _touch_wall(model, data):
    """扫描当前接触对，任一接触几何名以 wall 开头即判定撞墙。"""
    for c in range(data.ncon):
        g1 = model.geom(data.contact[c].geom1).name
        g2 = model.geom(data.contact[c].geom2).name
        if g1.startswith(WALL_PREFIX) or g2.startswith(WALL_PREFIX):
            return True
    return False


def _resolve_xml_path():
    """定位 models/wall_following.xml，兼容源码空间与 install 空间。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "models", "wall_following.xml"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("mujoco_wall_following")
        candidates.append(os.path.join(share, "models", "wall_following.xml"))
    except Exception:
        # 未 source ROS 2 环境（纯脚本直接运行）时跳过 install 空间定位
        pass
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError("未找到 wall_following.xml，已搜索: " + "; ".join(candidates))


def load_model():
    model = mujoco.MjModel.from_xml_path(_resolve_xml_path())
    model.opt.timestep = TIMESTEP
    return model


def run(model, data, viewer=None, step_sleep=0.0):
    """跑沿墙巡线闭环，返回结果字典。

    返回字段：``arrived``（是否绕回起点）、``collided``（是否撞墙）、``steps``、
    ``final_x``、``final_y``、``dist_start``（距起点距离）、``min_wall``、
    ``front_min``、``side_mean``/``side_std``/``side_min``（有效贴墙段侧距统计）。
    """
    bid = _body_id(model, "chassis")
    state = {"wp": 0, "mode": "turn", "side_filt": RAY_MAX_RANGE}
    step = 0
    arrived = False
    collided = False
    min_wall = RAY_MAX_RANGE
    front_min = RAY_MAX_RANGE
    side_valid = []

    while step < MAX_STEPS:
        x, y = data.qpos[0], data.qpos[1]
        yaw = quat_to_yaw(data.qpos[3:7])

        front_dist = _ray_distance(model, data, yaw, 0.0, bid)
        side_dist = _ray_distance(model, data, yaw, -math.pi / 2, bid)
        front_min = min(front_min, front_dist)
        if side_dist < RAY_MAX_RANGE:
            min_wall = min(min_wall, side_dist)

        left, right, state, arrived, _ = control_step(
            x, y, yaw, state, front_dist, side_dist)
        data.ctrl[0] = left
        data.ctrl[1] = right

        mujoco.mj_step(model, data)
        step += 1

        if _touch_wall(model, data):
            collided = True
            break

        # 有效贴墙样本才纳入精度统计（剔除未命中与转角过渡离群）
        if state["mode"] == "drive" and state["side_filt"] < 1.0:
            side_valid.append(state["side_filt"])

        if viewer is not None:
            viewer.sync()
            if step_sleep > 0:
                time.sleep(step_sleep)

        if arrived:
            break

    fx, fy = data.qpos[0], data.qpos[1]
    side = np.array(side_valid)
    return {
        "arrived": arrived,
        "collided": collided,
        "steps": step,
        "final_x": fx,
        "final_y": fy,
        "dist_start": math.hypot(fx - START_X, fy - START_Y),
        "min_wall": min_wall,
        "front_min": front_min,
        "side_mean": float(side.mean()) if side.size else float("nan"),
        "side_std": float(side.std()) if side.size else float("nan"),
        "side_min": float(side.min()) if side.size else float("nan"),
    }


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 差速小车沿墙巡线仿真")
    parser.add_argument("--headless", action="store_true",
                        help="无头模式：绕完一圈 / 撞墙 / 超时后打印结果退出（无窗口）")
    args = parser.parse_args()

    print("=== MuJoCo 差速小车沿墙巡线仿真 ===")
    print(f"起点: ({START_X}, {START_Y}) m  航点: {WAYPOINTS}")
    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if args.headless:
        res = run(model, data, viewer=None)
        _print_result(model, res)
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("viewer 已打开；小车沿矩形走廊逆时针绕一圈。关闭窗口即退出。")
        res = run(model, data, viewer=viewer, step_sleep=TIMESTEP)
        _print_result(model, res)
        # 闭环结束后保持窗口，等待用户手动关闭；小睡让出 CPU，避免忙等占满一个核
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)


def _print_result(model, res):
    sim_time = res["steps"] * model.opt.timestep
    if res["collided"]:
        status = "撞墙失败"
    elif res["arrived"]:
        status = "绕回起点"
    else:
        status = "超时未到达"
    print(f"结果: {status}")
    print(f"步数: {res['steps']}  仿真时长: {sim_time:.2f} s")
    print(f"终止位置: ({res['final_x']:.3f}, {res['final_y']:.3f}) m  "
          f"距起点: {res['dist_start']:.3f} m")
    print(f"侧距(有效段): 均值 {res['side_mean']:.3f}  std {res['side_std']:.3f}  "
          f"最小 {res['side_min']:.3f} m")
    print(f"最小侧方墙距: {res['min_wall']:.3f} m  最小前向距: {res['front_min']:.3f} m")


if __name__ == "__main__":
    main()
