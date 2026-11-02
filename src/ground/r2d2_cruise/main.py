#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R2D2 固定轮速差速圆周运动示例（PyBullet）。

左轮速度高、右轮速度低，左右轮保持固定速度差，小车沿圆弧持续巡航。
本示例是开环控制，不含轨迹反馈、路径跟踪与避障。

用法::

    # 打开可视化窗口，一直运行到关窗或 Ctrl+C
    python3 src/ground/r2d2_cruise/main.py

    # 无图形环境自检：跑 5 秒，把轨迹写进 CSV
    python3 src/ground/r2d2_cruise/main.py --direct --duration 5 --log run.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
import time

import pybullet as p
import pybullet_data

# 左快右慢：左右轮速度差的符号决定转弯方向，实测为沿圆弧持续右转巡航
LEFT_SPEED = 10.0   # rad/s，左轮目标角速度
RIGHT_SPEED = 2.0   # rad/s，右轮目标角速度
FORCE = 20.0        # N·m，关节最大力矩
TIME_STEP = 1.0 / 240.0

# r2d2.urdf 静止时车体中心约在 0.465 m 高；直接按 0.2 m 生成会插进地面被弹飞，
# 因此按 0.5 m 生成，并先静置 0.5 s 让车落稳再起步。
SPAWN_HEIGHT = 0.5
SETTLE_TIME = 0.5

# 官方 r2d2.urdf 的轮子关节名：right_* 为右侧轮，left_* 为左侧轮
LEFT_JOINT_NAMES = ("left_front_wheel_joint", "left_back_wheel_joint")
RIGHT_JOINT_NAMES = ("right_front_wheel_joint", "right_back_wheel_joint")

CSV_HEADER = ("t", "x", "y", "z", "yaw", "left_vel", "right_vel")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="R2D2 固定轮速差速圆周运动示例")
    parser.add_argument("--direct", action="store_true",
                        help="使用 DIRECT 模式（不弹窗），用于无图形环境自检")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="仿真时长（秒）；0 表示一直运行到关窗或 Ctrl+C")
    parser.add_argument("--log", default=None,
                        help="可选：把轨迹与轮速按 10 Hz 写入 CSV")
    parser.add_argument("--width", type=int, default=960, help="可视化窗口宽度")
    parser.add_argument("--height", type=int, default=720, help="可视化窗口高度")
    return parser.parse_args(argv)


def connect(args):
    """按 --direct 选择 DIRECT（无窗口）或 GUI 连接。"""
    if args.direct:
        return p.connect(p.DIRECT)
    options = "--width=%d --height=%d" % (args.width, args.height)
    return p.connect(p.GUI, options=options)


def find_wheels(robot_id):
    """按关节名分出左右轮，返回 (left_joints, right_joints)。"""
    left, right = [], []
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        if name in LEFT_JOINT_NAMES:
            left.append(i)
        elif name in RIGHT_JOINT_NAMES:
            right.append(i)
    return left, right


def set_wheel_speed(robot_id, joints, speed):
    """给一组车轮设置恒定目标速度。"""
    for j in joints:
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                targetVelocity=speed, force=FORCE)


def base_state(robot_id):
    """返回车体位置与偏航角。"""
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    yaw = p.getEulerFromQuaternion(orn)[2]
    return pos, yaw


def mean_wheel_velocity(robot_id, joints):
    if not joints:
        return 0.0
    speeds = [p.getJointState(robot_id, j)[1] for j in joints]
    return sum(speeds) / len(speeds)


def settle(client, seconds=SETTLE_TIME):
    """起步前先让车落到地面并稳住，避免初始穿插导致的弹跳。"""
    steps = int(round(seconds / TIME_STEP))
    for _ in range(steps):
        if not p.isConnected(client):
            return
        p.stepSimulation()


def run(robot_id, left_joints, right_joints, args, log_path):
    """主循环：固定轮速驱动 + 可选轨迹记录。"""
    writer = None
    fp = None
    if log_path:
        fp = open(log_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(fp)
        writer.writerow(CSV_HEADER)

    t = 0.0
    steps = 0
    try:
        while p.isConnected():
            if args.duration > 0 and t >= args.duration - 1e-9:
                break
            set_wheel_speed(robot_id, left_joints, LEFT_SPEED)
            set_wheel_speed(robot_id, right_joints, RIGHT_SPEED)
            p.stepSimulation()
            t += TIME_STEP
            steps += 1
            if steps % 24 == 0:   # 240 Hz / 24 = 10 Hz
                pos, yaw = base_state(robot_id)
                if writer is not None:
                    writer.writerow([round(t, 4), round(pos[0], 4), round(pos[1], 4),
                                     round(pos[2], 4), round(yaw, 4),
                                     round(mean_wheel_velocity(robot_id, left_joints), 4),
                                     round(mean_wheel_velocity(robot_id, right_joints), 4)])
                if not args.direct:
                    # 相机跟随车体，避免小车转出视野
                    p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=55,
                                                 cameraPitch=-32, cameraTargetPosition=pos)
            if not args.direct:
                time.sleep(TIME_STEP)
    except KeyboardInterrupt:
        pass
    finally:
        if fp is not None:
            fp.close()
    return steps, t


def main(argv=None):
    args = parse_args(argv)
    client = connect(args)
    if client < 0:
        print("连接 PyBullet 失败", file=sys.stderr)
        return 1

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)
    if not args.direct:
        # 关掉默认打开的 RGB/深度预览面板，只保留三维视图
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)

    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("r2d2.urdf", [0, 0, SPAWN_HEIGHT])
    left_joints, right_joints = find_wheels(robot_id)

    if not left_joints or not right_joints:
        found = [p.getJointInfo(robot_id, i)[1].decode("utf-8")
                 for i in range(p.getNumJoints(robot_id))]
        print("没找到左右轮关节，当前关节名为：%s" % found, file=sys.stderr)
        p.disconnect(client)
        return 2

    try:
        settle(client)
        if not args.direct:
            # 初始机位：略高处俯视小车，保证一打开就能看到车和地面
            p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=55,
                                         cameraPitch=-32, cameraTargetPosition=[0, 0, 0.4])
        steps, elapsed = run(robot_id, left_joints, right_joints, args, args.log)
        pos, yaw = base_state(robot_id)
        print("仿真结束：%d 步 / %.2f s，车体位置 (%.2f, %.2f, %.2f)，偏航角 %.2f rad"
              % (steps, elapsed, pos[0], pos[1], pos[2], yaw))
        if args.log:
            print("轨迹已写入 %s" % args.log)
    finally:
        p.disconnect(client)
        print("仿真已退出，连接已释放")
    return 0


if __name__ == "__main__":
    sys.exit(main())
