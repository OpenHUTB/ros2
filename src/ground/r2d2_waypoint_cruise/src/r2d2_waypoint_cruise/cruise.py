#!/usr/bin/env python3
"""
R2D2 小车航点巡航仿真模块

功能：R2D2 差速小车沿预设航点路径自动巡航，依次导航到每个航点，
到达后自动切换下一个，循环巡逻。使用 P 控制器实现位置与航向控制。

作者：xjm666god
"""

import pybullet as p
import pybullet_data
import time
import math
import csv
import os


# ============ 配置参数 ============
# 预设航点（世界坐标系下的 x, y 坐标）—— 方形巡逻路径
WAYPOINTS = [
    (0.0, 0.0),    # 起点
    (3.0, 0.0),    # 向东 3m
    (3.0, 3.0),    # 向北 3m
    (0.0, 3.0),    # 向西 3m
    (0.0, 0.0),    # 回到起点
]

ARRIVE_THRESHOLD = 0.3   # 到达航点的距离阈值（米）
MAX_LINEAR_SPEED = 3.0   # 最大线速度 (m/s)
MAX_TURN_SPEED = 3.0     # 最大转向角速度 (rad/s)
WHEEL_BASE = 0.4         # 左右轮间距（米），用于差速解算
WHEEL_RADIUS = 0.1       # 轮子半径（米）

# 控制增益
Kp_DIST = 1.5    # 距离比例增益
Kp_HEADING = 3.0 # 航向角比例增益

# R2D2 运动学标定参数
# PyBullet 自带 r2d2.urdf 模型：yaw=0 时朝向 y 轴负方向，
# 左轮加快时 yaw 增加（逆时针旋转）。
YAW_OFFSET = math.pi / 2  # 标准角度转 R2D2 yaw 坐标系的偏移

# 仿真参数
SIM_DURATION = 60.0  # 最长仿真时间（秒），超时自动停止
LOG_FILE = "/tmp/r2d2_cruise_trajectory.csv"


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def get_robot_pose(robot_id):
    """获取机器人当前位置和朝向"""
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    _, _, yaw = p.getEulerFromQuaternion(orn)
    return pos[0], pos[1], yaw


def compute_control(x, y, yaw, goal_x, goal_y):
    """
    P 控制器：根据当前位置和目标航点，计算左右轮速度

    返回: (left_wheel_speed, right_wheel_speed, dist, heading_err)，
          轮子速度单位 rad/s
    """
    dx = goal_x - x
    dy = goal_y - y
    dist = math.sqrt(dx * dx + dy * dy)

    # 标准坐标系下的目标方向角
    target_yaw_standard = math.atan2(dy, dx)
    # 转换为 R2D2 的 yaw 坐标系
    target_yaw = target_yaw_standard + YAW_OFFSET
    heading_err = normalize_angle(target_yaw - yaw)

    # 线速度：距离越远越快，接近目标时减速
    linear_vel = min(Kp_DIST * dist, MAX_LINEAR_SPEED)

    # 航向偏差较大时，先原地转向再前进
    if abs(heading_err) > math.pi / 6:  # 30度以上先转向
        linear_vel *= 0.2

    # 角速度：比例控制
    angular_vel = Kp_HEADING * heading_err
    angular_vel = max(min(angular_vel, MAX_TURN_SPEED), -MAX_TURN_SPEED)

    # 差速解算（R2D2 标定：左轮加快 -> yaw 增加，逆时针转）
    v_left = linear_vel + angular_vel * WHEEL_BASE / 2.0
    v_right = linear_vel - angular_vel * WHEEL_BASE / 2.0

    # 转换为轮子角速度 (rad/s)
    left_wheel_speed = v_left / WHEEL_RADIUS
    right_wheel_speed = v_right / WHEEL_RADIUS

    return left_wheel_speed, right_wheel_speed, dist, heading_err


def main():
    # 连接仿真
    client_id = p.connect(p.GUI, options="--width=960 --height=720")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    # 加载地面和机器人
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.2])
    p.setRealTimeSimulation(1)

    # 找出四个轮子的关节索引
    left_wheels = []
    right_wheels = []
    for i in range(p.getNumJoints(robot_id)):
        joint_name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        if joint_name in ("left_front_wheel_joint", "left_back_wheel_joint"):
            left_wheels.append(i)
        elif joint_name in ("right_front_wheel_joint", "right_back_wheel_joint"):
            right_wheels.append(i)

    print(f"左轮关节: {left_wheels}, 右轮关节: {right_wheels}")

    # 绘制航点标记（红色小球）
    for wx, wy in WAYPOINTS:
        vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.1,
                                rgbaColor=[1, 0, 0, 0.8])
        p.createMultiBody(baseVisualShapeIndex=vs,
                          basePosition=[wx, wy, 0.1])

    # 设置相机视角
    p.resetDebugVisualizerCamera(
        cameraDistance=6,
        cameraYaw=50,
        cameraPitch=-45,
        cameraTargetPosition=[1.5, 1.5, 0]
    )

    # 轨迹记录
    log_rows = [["time", "x", "y", "yaw", "target_wp_idx", "dist_to_target"]]
    start_time = time.time()
    wp_idx = 1  # 起点就是航点0，直接从航点1开始巡航
    loop_count = 0

    print("=" * 50)
    print("R2D2 航点巡航仿真开始")
    print(f"共 {len(WAYPOINTS)} 个航点，循环巡逻")
    print("=" * 50)

    try:
        while p.isConnected(client_id):
            elapsed = time.time() - start_time
            if elapsed > SIM_DURATION:
                print(f"仿真达到最大时长 {SIM_DURATION}s，自动停止")
                break

            # 获取当前位姿
            x, y, yaw = get_robot_pose(robot_id)

            # 当前目标航点
            goal_x, goal_y = WAYPOINTS[wp_idx]

            # 计算控制量
            left_speed, right_speed, dist, heading_err = compute_control(
                x, y, yaw, goal_x, goal_y
            )

            # 设置轮子速度
            force = 20
            for j in left_wheels:
                p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                        targetVelocity=left_speed, force=force)
            for j in right_wheels:
                p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                        targetVelocity=right_speed, force=force)

            # 判断是否到达航点
            if dist < ARRIVE_THRESHOLD:
                print(f"[{elapsed:.1f}s] 到达航点 {wp_idx} "
                      f"({goal_x:.1f}, {goal_y:.1f})")
                wp_idx += 1
                if wp_idx >= len(WAYPOINTS):
                    wp_idx = 1  # 跳过起点，从航点1继续循环
                    loop_count += 1
                    print(f"--- 完成第 {loop_count} 圈巡逻，重新开始 ---")

            # 记录数据
            log_rows.append([
                f"{elapsed:.3f}",
                f"{x:.4f}",
                f"{y:.4f}",
                f"{yaw:.4f}",
                wp_idx,
                f"{dist:.4f}",
            ])

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n用户中断，正在退出...")
    finally:
        # 保存轨迹日志
        with open(LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(log_rows)
        print(f"轨迹数据已保存到: {LOG_FILE}")
        print(f"共记录 {len(log_rows)-1} 条数据")
        print(f"完成巡逻圈数: {loop_count}")

        p.disconnect(client_id)
        print("仿真已退出，连接已释放")


if __name__ == "__main__":
    main()
