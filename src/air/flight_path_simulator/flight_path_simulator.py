#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
预设航线飞行模拟器
==================
按预设时间表播放一段多旋翼巡航飞行（起飞-巡航-转弯-悬停-降落），
逐帧记录飞行状态数据，并生成轨迹图、高度图、速度图、状态时间线。

与仓库已有 drone_teleop / drone_ros_teleop / uav_keyboard_control 的区别：
  以上三者均为真实键盘/手柄输入的实时遥控模块，本模块为离线开环脚本，
  用于快速生成可复现的飞行数据集与可视化结果，不依赖任何仿真器或输入设备。

输出：
  - flight_log.csv    逐帧飞行数据（位置/速度/状态/时间）
  - trajectory.png    XY 平面飞行轨迹图
  - altitude.png      高度随时间变化曲线
  - velocity.png      三轴速度随时间变化曲线
  - state_timeline.png 飞行状态时间线

用法：
  python3 flight_path_simulator.py                    # 输出到当前目录
  python3 flight_path_simulator.py --outdir results/  # 输出到指定目录
"""

import csv
import math
import os
import argparse
import random
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无头环境用 Agg 后端
import matplotlib.pyplot as plt

# ========== 模拟参数 ==========
CONTROL_RATE = 20.0          # 控制频率 20 Hz
DT = 1.0 / CONTROL_RATE      # 每步时间间隔
TOTAL_TIME = 30.0           # 总飞行时长 30 秒

# 无人机标称速度
CRUISE_SPEED = 2.5           # 巡航水平速度 m/s
VERT_SPEED = 1.5             # 垂直速度 m/s
YAW_RATE = math.radians(30)  # 偏航角速度 rad/s

# 起飞点（NED 坐标系：x 北、y 东、z 地）
START_X = 0.0
START_Y = 0.0
START_Z = 0.0  # 地面 z=0

# 悬停高度（离地高度，正数）。NED 坐标系 z 向下为正，因此 z = -HOVER_ALT
HOVER_ALT = 2.7
HOVER_Z = -HOVER_ALT  # 到达该高度即切换为悬停

# 随机种子，保证可复现
random.seed(42)
np.random.seed(42)


class DroneSimulator:
    """开环运动学积分模型：按速度指令积分位置，叠加高斯噪声。

    不含质量/推力/姿态/速度环滞后，因此不构成完整的动力学模型。
    """

    def __init__(self):
        # 状态：位置 (x, y, z)，速度 (vx, vy, vz)，偏航角 theta
        self.x = START_X
        self.y = START_Y
        self.z = START_Z
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.theta = 0.0  # 偏航角（弧度）

        self.time = 0.0
        self.state_label = "grounded"  # grounded / taking_off / hover / forward / turn / landing

    def step(self, cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate):
        """
        一步模拟：输入机体坐标系下的速度指令和偏航角速度，
        更新世界坐标系下的位置和姿态。
        """
        # 机体坐标系 -> 世界坐标系（偏航旋转）
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        world_vx = cmd_vx * cos_t - cmd_vy * sin_t
        world_vy = cmd_vx * sin_t + cmd_vy * cos_t
        world_vz = cmd_vz

        # 加一点噪声，模拟真实飞控抖动
        world_vx += np.random.normal(0, 0.05)
        world_vy += np.random.normal(0, 0.05)
        world_vz += np.random.normal(0, 0.03)

        # 积分更新位置
        self.x += world_vx * DT
        self.y += world_vy * DT
        self.z += world_vz * DT
        self.theta += cmd_yaw_rate * DT

        # 记录当前速度（世界系）
        self.vx = world_vx
        self.vy = world_vy
        self.vz = world_vz

        self.time += DT


def build_flight_plan():
    """
    生成一段模拟的遥控操作序列（按时间段）：
    0-2s:  起飞爬升
    2-5s:  悬停
    5-12s: 直线前进巡航
    12-15s: 原地右转 90°
    15-22s: 直线前进（新方向）
    22-25s: 悬停
    25-30s: 降落
    """
    plan = []  # (start_time, end_time, cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, label)

    # NED 坐标系：vz 负值 = 上升，vz 正值 = 下降
    plan.append((0.0, 2.0, 0.0, 0.0, -VERT_SPEED, 0.0, "takeoff"))       # 起飞爬升（vz 负 = 上升）
    plan.append((2.0, 5.0, 0.0, 0.0, 0.0, 0.0, "hover"))                 # 悬停
    plan.append((5.0, 12.0, CRUISE_SPEED, 0.0, 0.0, 0.0, "forward"))     # 前进
    plan.append((12.0, 15.0, 0.0, 0.0, 0.0, -YAW_RATE, "turn_right"))    # 右转
    plan.append((15.0, 22.0, CRUISE_SPEED, 0.0, 0.0, 0.0, "forward"))   # 再前进
    plan.append((22.0, 25.0, 0.0, 0.0, 0.0, 0.0, "hover"))             # 悬停
    plan.append((25.0, 30.0, 0.0, 0.0, VERT_SPEED, 0.0, "landing"))     # 降落（vz 正 = 下降）

    return plan


def get_command_at(t, plan):
    """根据时间 t 查询当前指令"""
    for start, end, vx, vy, vz, yaw, label in plan:
        if start <= t < end:
            return vx, vy, vz, yaw, label
    return 0.0, 0.0, 0.0, 0.0, "done"


def run_simulation():
    """运行完整模拟，返回逐帧数据列表"""
    drone = DroneSimulator()
    plan = build_flight_plan()
    records = []

    n_steps = int(TOTAL_TIME / DT)

    for i in range(n_steps):
        t = i * DT
        cmd_vx, cmd_vy, cmd_vz, cmd_yaw, label = get_command_at(t, plan)

        # 起飞阶段：垂直上升，直到到达悬停高度附近再切换到悬停
        if label == "takeoff":
            # NED 下 z 越负高度越高；到达目标高度附近就停止上升
            if drone.z <= HOVER_Z:  # z 到达 -2.7（离地 2.7m）即停止上升
                cmd_vz = 0.0
                label = "hover"

        # 降落阶段：垂直下降到地面
        if label == "landing":
            if drone.z >= -0.2:  # 接近地面（z 接近 0）
                cmd_vz = 0.0
                drone.z = 0.0
                label = "landed"

        drone.step(cmd_vx, cmd_vy, cmd_vz, cmd_yaw)

        # 计算离地高度（NED z 取负）
        altitude = -drone.z

        # 水平速度大小
        horizontal_speed = math.sqrt(drone.vx**2 + drone.vy**2)

        records.append({
            "timestamp": round(t, 3),
            "pos_x": round(drone.x, 4),
            "pos_y": round(drone.y, 4),
            "pos_z": round(drone.z, 4),
            "altitude_m": round(altitude, 4),
            "vel_x": round(drone.vx, 4),
            "vel_y": round(drone.vy, 4),
            "vel_z": round(drone.vz, 4),
            "horizontal_speed": round(horizontal_speed, 4),
            "yaw_deg": round(math.degrees(drone.theta), 2),
            "state": label,
        })

    return records


def save_csv(records, filename):
    """保存飞行数据到 CSV"""
    fieldnames = records[0].keys()
    with open(filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)
    print(f"[输出] {filename}  ({len(records)} 行数据)")


def plot_trajectory(records, filename):
    """绘制 XY 平面飞行轨迹图"""
    xs = [r["pos_x"] for r in records]
    ys = [r["pos_y"] for r in records]
    alts = [r["altitude_m"] for r in records]

    fig, ax = plt.subplots(figsize=(10, 8))

    # 用颜色表示高度
    scatter = ax.scatter(xs, ys, c=alts, cmap='viridis', s=8, alpha=0.7)
    plt.colorbar(scatter, label='Altitude (m)')

    # 起点和终点
    ax.plot(xs[0], ys[0], 'go', markersize=12, label='Takeoff')
    ax.plot(xs[-1], ys[-1], 'rs', markersize=12, label='Landing')

    # 轨迹线
    ax.plot(xs, ys, 'b-', linewidth=1.5, alpha=0.5, label='Flight Path')

    ax.set_xlabel('X (North) [m]')
    ax.set_ylabel('Y (East) [m]')
    ax.set_title('UAV Flight Trajectory (Top View)')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()
    print(f"[输出] {filename}")


def plot_altitude(records, filename):
    """绘制高度随时间变化曲线"""
    times = [r["timestamp"] for r in records]
    alts = [r["altitude_m"] for r in records]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, alts, 'b-', linewidth=2)

    # 标记关键阶段
    ax.axhline(y=2.7, color='g', linestyle='--', alpha=0.5, label='Hover Altitude (~2.7m)')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('UAV Altitude vs Time')
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()
    print(f"[输出] {filename}")


def plot_velocity(records, filename):
    """绘制三轴速度曲线"""
    times = [r["timestamp"] for r in records]
    vxs = [r["vel_x"] for r in records]
    vys = [r["vel_y"] for r in records]
    vzs = [r["vel_z"] for r in records]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(times, vxs, 'r-', linewidth=1.5)
    axes[0].set_ylabel('Vx (m/s)')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Velocity Components vs Time')

    axes[1].plot(times, vys, 'g-', linewidth=1.5)
    axes[1].set_ylabel('Vy (m/s)')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, vzs, 'b-', linewidth=1.5)
    axes[2].set_ylabel('Vz (m/s)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()
    print(f"[输出] {filename}")


def plot_state_timeline(records, filename):
    """绘制飞行状态时间线"""
    times = [r["timestamp"] for r in records]
    states = [r["state"] for r in records]

    # 状态映射为数字
    state_map = {
        "takeoff": 4, "hover": 3, "forward": 2,
        "turn_right": 1, "landing": 0, "landed": 0, "grounded": -1
    }
    state_nums = [state_map.get(s, 0) for s in states]

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(times, state_nums, drawstyle='steps-post', linewidth=2, color='purple')

    ax.set_yticks([-1, 0, 1, 2, 3, 4])
    ax.set_yticklabels(['Grounded', 'Landing', 'Turn', 'Forward', 'Hover', 'Takeoff'])
    ax.set_xlabel('Time (s)')
    ax.set_title('Flight State Timeline')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()
    print(f"[输出] {filename}")


def print_summary(records):
    """打印飞行摘要统计"""
    xs = [r["pos_x"] for r in records]
    ys = [r["pos_y"] for r in records]
    alts = [r["altitude_m"] for r in records]
    speeds = [r["horizontal_speed"] for r in records]

    total_dist = 0.0
    for i in range(1, len(records)):
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        total_dist += math.sqrt(dx*dx + dy*dy)

    print("\n" + "="*50)
    print("       飞行模拟完成 - 摘要统计")
    print("="*50)
    print(f"  总飞行时间：   {records[-1]['timestamp']:.1f} 秒")
    print(f"  采样帧数：     {len(records)} 帧 @ {CONTROL_RATE} Hz")
    print(f"  最大高度：     {max(alts):.2f} 米")
    print(f"  水平最大速度： {max(speeds):.2f} m/s")
    print(f"  总航程：       {total_dist:.2f} 米")
    print(f"  结束位置：     ({xs[-1]:.2f}, {ys[-1]:.2f})")
    print("="*50)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='预设航线飞行模拟器')
    parser.add_argument('--outdir', type=str, default='.',
                        help='输出目录（默认当前目录）')
    args = parser.parse_args()

    outdir = args.outdir
    os.makedirs(outdir, exist_ok=True)

    print("="*50)
    print("  预设航线飞行模拟器")
    print("="*50)
    print(f"  控制频率: {CONTROL_RATE} Hz")
    print(f"  总时长:   {TOTAL_TIME} 秒")
    print(f"  悬停高度: ~2.7 m")
    print(f"  输出目录: {outdir}")
    print()

    # 运行模拟
    records = run_simulation()

    # 输出 CSV
    save_csv(records, os.path.join(outdir, 'flight_log.csv'))

    # 输出图表
    plot_trajectory(records, os.path.join(outdir, 'trajectory.png'))
    plot_altitude(records, os.path.join(outdir, 'altitude.png'))
    plot_velocity(records, os.path.join(outdir, 'velocity.png'))
    plot_state_timeline(records, os.path.join(outdir, 'state_timeline.png'))

    # 打印摘要
    print_summary(records)
