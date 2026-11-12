#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 平面 2R 双自由度机械臂画正方形（运动学回放）。

设计说明
--------
本脚本是**运动学回放**，不是动力学仿真：
  - 每个目标点先用解析 IK 算出两个关节角，直接写入 ``data.qpos``；
  - 调用 ``mj_forward`` 重算正运动学与渲染几何后由 viewer 同步；
  - 全程**不调用 ``mj_step``**，不做动力学积分，位置伺服 kp / 阻尼
    只写在 XML 里作模型定义，本脚本不经过执行器力矩闭环。
这样做的原因：2R 臂在小负载下直接摆角，轨迹几何精度由 IK 解析解保证，
避免 PD 控制在短杆、低惯量下出现的收敛误差。

正方形四个顶点（单位 m，工作平面 z≈0.06）：
  (0.40,-0.05) -> (0.40,0.05) -> (0.50,0.05) -> (0.50,-0.05) -> 回到起点。
每条边 50 个笛卡尔直线插值点，末端沿正方形轮廓循环运动。

用法
----
    python3 main.py              # 打开 MuJoCo viewer 可视化，关闭窗口退出
    python3 main.py --headless   # 无头模式：跑 1 圈后打印误差并退出（无窗口）
    python3 main.py --headless --laps 3   # 无头模式跑 3 圈
"""
import os
import sys
import time
import math
import argparse

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco
import mujoco.viewer

# 杆长（与 XML 中 capsule fromto 一致）
L1 = 0.40   # 大臂长度 m
L2 = 0.25   # 小臂长度 m
TIMESTEP = 0.002

# 正方形四个顶点（m），按运动顺序排列
TARGET_CORNERS = [
    (0.40, -0.05),
    (0.40,  0.05),
    (0.50,  0.05),
    (0.50, -0.05),
]


def ik(x, y):
    """平面 2R 解析逆运动学（肘向上构型）。返回 (q1, q2) 弧度。"""
    cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    sin_q2 = math.sqrt(1.0 - cos_q2 * cos_q2)   # 肘向上取正
    q2 = math.atan2(sin_q2, cos_q2)
    q1 = math.atan2(y, x) - math.atan2(L2 * sin_q2, L1 + L2 * cos_q2)
    return q1, q2


def linspace(p0, p1, steps):
    """p0 -> p1 直线插值，返回 steps+1 个点（含两端）。"""
    x0, y0 = p0
    x1, y1 = p1
    return [
        (x0 + (x1 - x0) * i / steps,
         y0 + (y1 - y0) * i / steps)
        for i in range(steps + 1)
    ]


def _resolve_xml_path():
    """定位 two_dof_arm_square.xml，兼容源码空间、devel 空间与 install 空间。

    - 源码 / devel：relay 脚本转发到源码 main.py，``__file__`` 与 xml 同目录；
    - install 空间：main.py 在 ``lib/<pkg>/``，xml 在 ``share/<pkg>/``；
    - 也允许通过 ``ROS_PACKAGE_PATH`` 指向的包目录查找。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "two_dof_arm_square.xml"),
        os.path.join(here, "..", "..", "share", "two_dof_arm_square", "two_dof_arm_square.xml"),
    ]
    ros_pp = os.environ.get("ROS_PACKAGE_PATH")
    if ros_pp:
        for base in ros_pp.split(os.pathsep):
            candidates.append(
                os.path.join(base, "two_dof_arm_square", "two_dof_arm_square.xml"))
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError(
        "未找到 two_dof_arm_square.xml，已搜索: " + "; ".join(candidates))


def load_model():
    xml_path = _resolve_xml_path()
    model = mujoco.MjModel.from_xml_path(xml_path)
    model.opt.timestep = TIMESTEP
    return model


def run_lap(model, data, end_id, viewer=None, step_sleep=0.03):
    """跑一圈正方形轨迹，返回本圈最大位置误差 (m) 与各顶点误差列表。"""
    corners = TARGET_CORNERS + [TARGET_CORNERS[0]]
    max_err = 0.0
    corner_errs = []
    for idx in range(len(corners) - 1):
        p0, p1 = corners[idx], corners[idx + 1]
        edge_points = linspace(p0, p1, 50)
        for i, (tx, ty) in enumerate(edge_points):
            q1t, q2t = ik(tx, ty)
            data.qpos[0] = q1t
            data.qpos[1] = q2t
            mujoco.mj_forward(model, data)
            ee_x = data.xpos[end_id][0]
            ee_y = data.xpos[end_id][1]
            err = math.hypot(ee_x - tx, ee_y - ty)
            if err > max_err:
                max_err = err
            # 每条边的第一个点就是一个目标顶点（四条边起点正好是四个角），记录一次
            if i == 0:
                corner_errs.append(((tx, ty), ee_x, ee_y, err))
            if viewer is not None:
                viewer.sync()
                time.sleep(step_sleep)
    return max_err, corner_errs


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 2R 机械臂画正方形（运动学回放）")
    parser.add_argument("--headless", action="store_true",
                        help="无头模式：不打开 viewer，跑指定圈数后打印误差退出")
    parser.add_argument("--laps", type=int, default=1,
                        help="无头模式跑的圈数（默认 1）")
    args = parser.parse_args()

    print("=== MuJoCo 2R 平面机械臂画正方形（运动学回放） ===")
    print("说明：逐帧用解析 IK 写入 qpos 后 mj_forward，不做 mj_step 动力学积分。")
    model = load_model()
    data = mujoco.MjData(model)
    end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")

    if args.headless:
        # 无头：直接跑 N 圈，打印量化误差
        worst = 0.0
        for lap in range(args.laps):
            max_err, corner_errs = run_lap(model, data, end_id, viewer=None)
            worst = max(worst, max_err)
            print(f"[第 {lap+1}/{args.laps} 圈] 末端最大位置误差: {max_err*1000:.3f} mm")
        print("--- 四个顶点末端实测坐标与偏差（最后一圈） ---")
        for (tx, ty), ex, ey, e in corner_errs:
            print(f"  目标 ({tx:+.3f},{ty:+.3f}) m | 实测 ({ex:+.4f},{ey:+.4f}) m | 偏差 {e*1000:.3f} mm")
        print(f"=== {args.laps} 圈累计最大误差: {worst*1000:.3f} mm ===")
        print("误差定义：末端 end_effector 球心世界坐标与目标插值点的欧氏距离，单位 mm。")
        return

    # 可视化模式
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("viewer 已打开，机械臂沿红色正方形边框循环运动；关闭 MuJoCo 窗口即退出。")
        while viewer.is_running():
            max_err, corner_errs = run_lap(model, data, end_id, viewer=viewer)
            print(f"完成一圈，末端最大位置误差: {max_err*1000:.3f} mm")


if __name__ == "__main__":
    main()
