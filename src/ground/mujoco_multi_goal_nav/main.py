#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 小车多目标导航

小车从原点出发，依次到达三个目标点，每到达一个点打印坐标与误差，全部到达后停车。

控制方式（平面 3 自由度）：
  - x / y 平动：按目标方向给定速度，速度大小随剩余距离衰减，到点判定 dist < STOP_DIST；
  - z 高度、姿态（roll / pitch / yaw）：每步锁定为初始值，避免 free joint 车体倾斜、
    车头随碰撞漂移。

用法：
    python3 main.py                        # 带可视化窗口运行
    python3 main.py --nogui                # 无窗口，仅跑物理（服务器 / 无显示环境）
    python3 main.py --nogui --duration 40  # 指定最大仿真时长（秒）
    python3 main.py --nogui --csv traj.csv # 另外把轨迹落成 CSV
"""
import argparse
import math
import os
import sys
import time

try:
    import mujoco
except ImportError:  # 依赖缺失时给出可执行的修复提示，而不是裸 ImportError 堆栈
    sys.exit(
        "缺少 MuJoCo，节点无法启动。请在当前 Python 环境安装依赖后重试：\n"
        "    pip3 install -r requirements.txt      # 或 pip3 install mujoco\n"
        "自检：python3 -c \"import mujoco; print(mujoco.__version__)\"\n"
        "注意：通过 roslaunch 启动时节点运行在 devel 空间的 lib/<pkg>/main.py，"
        "用的是同一个 Python 环境。"
    )

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = "mujoco_multi_goal_nav"

# 目标点（世界坐标，单位 m），与 scene.xml 里的三个标记一一对应
GOALS = [(1.5, 1.0), (-1.5, 1.0), (-1.5, -1.0)]

SPEED = 0.3          # 最高线速度 m/s
SLOW_RADIUS = 0.3    # 剩余距离小于该值时按比例降速，避免冲过目标点
STOP_DIST = 0.02     # 到达判定阈值 m
Z0 = 0.05            # 车体中心高度 m（与 scene.xml 中 car 的 pos 一致）
PRINT_INTERVAL = 0.5 # 位置打印间隔 s


def resolve_scene_path():
    """定位 scene.xml：环境变量 → 脚本同目录 → 上级 share/<pkg>/。"""
    env = os.environ.get("MUJOCO_MULTI_GOAL_SCENE")
    if env and os.path.isfile(env):
        return os.path.abspath(env)
    candidates = [
        os.path.join(HERE, "scene.xml"),
        os.path.join(HERE, "..", "share", PKG, "scene.xml"),
        os.path.join(HERE, "..", "..", "share", PKG, "scene.xml"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    return os.path.join(HERE, "scene.xml")


def lock_pose(data):
    """锁定车体高度与姿态（roll / pitch / yaw），只保留平面平动。"""
    data.qpos[2] = Z0
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    data.qvel[2] = 0.0
    data.qvel[3:6] = 0.0


def run(args):
    scene = resolve_scene_path()
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print("场景文件：%s" % scene)
    print("运行环境：MuJoCo %s / Python %s" % (mujoco.__version__, sys.version.split()[0]))
    print("目标点：%s" % ", ".join("(%g, %g)" % g for g in GOALS))
    print("运行参数：最高速度 %.2f m/s，到达阈值 %.2f m，时长上限 %.1f s"
          % (SPEED, STOP_DIST, args.duration))

    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w")
        csv_file.write("time,x,y,goal_index\n")

    viewer_ctx = None
    viewer = None
    if not args.nogui:
        import mujoco.viewer as mj_viewer   # 显式加载子模块，避免与顶层 mujoco 名字冲突
        viewer_ctx = mj_viewer.launch_passive(model, data)
        viewer = viewer_ctx.__enter__()

    goal_index = 0
    arrived = []
    last_print = -1.0

    try:
        while data.time < args.duration:
            if viewer is not None and not viewer.is_running():
                break
            if goal_index >= len(GOALS):
                data.qvel[0] = 0.0
                data.qvel[1] = 0.0
                print("所有目标点已到达，停车")
                break

            target_x, target_y = GOALS[goal_index]
            dx = target_x - data.qpos[0]
            dy = target_y - data.qpos[1]
            dist = math.hypot(dx, dy)

            if dist < STOP_DIST:
                print("到达目标点%d：x=%g, y=%g（实测 x=%.3f, y=%.3f，误差 %.4f m，累计用时 %.2f s）"
                      % (goal_index + 1, target_x, target_y,
                         data.qpos[0], data.qpos[1], dist, data.time))
                arrived.append((goal_index + 1, data.time, dist))
                goal_index += 1
                if goal_index >= len(GOALS):
                    continue

            target_x, target_y = GOALS[goal_index]
            dx = target_x - data.qpos[0]
            dy = target_y - data.qpos[1]
            dist = math.hypot(dx, dy)
            if dist > 1e-9:
                speed = min(SPEED, SPEED * dist / SLOW_RADIUS)
                data.qvel[0] = speed * dx / dist
                data.qvel[1] = speed * dy / dist
            else:
                data.qvel[0] = 0.0
                data.qvel[1] = 0.0

            mujoco.mj_step(model, data)
            lock_pose(data)

            if data.time - last_print >= PRINT_INTERVAL:
                print("[%5.2fs] 前往目标点%d：x=%+.3f y=%+.3f，剩余 %.3f m"
                      % (data.time, goal_index + 1, data.qpos[0], data.qpos[1], dist))
                if csv_file is not None:
                    csv_file.write("%.3f,%.4f,%.4f,%d\n"
                                   % (data.time, data.qpos[0], data.qpos[1], goal_index + 1))
                last_print = data.time

            if viewer is not None:
                viewer.sync()
                time.sleep(0.002)
    finally:
        if csv_file is not None:
            csv_file.close()
        if viewer_ctx is not None:
            viewer_ctx.__exit__(None, None, None)

    print("=== 统计 ===")
    for index, t, err in arrived:
        print("目标点%d：用时 %.2f s，到达误差 %.4f m" % (index, t, err))
    print("最终位置       : x=%+.3f m, y=%+.3f m" % (data.qpos[0], data.qpos[1]))
    print("到达个数       : %d / %d" % (len(arrived), len(GOALS)))
    print("总仿真时长     : %.2f s" % data.time)
    if len(arrived) == len(GOALS):
        print("是否全部到达   : 是")
        return 0
    print("是否全部到达   : 否（未在时长上限内完成）")
    return 1


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 小车多目标导航")
    parser.add_argument("--nogui", action="store_true",
                        help="不创建可视化窗口，只跑物理（无显示环境用）")
    parser.add_argument("--duration", type=float, default=40.0,
                        help="最大仿真时长，单位秒，默认 40")
    parser.add_argument("--csv", default=None,
                        help="把轨迹写成 CSV 的路径（可选，默认不写）")
    args = parser.parse_args()
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
