#!/usr/bin/env python3
"""task3_slam_nav 主入口 main.py
基于 LiDAR 扇区特征的神经网络 SLAM 导航：
    感知(LiDAR->16扇区) -> 规划/控制(ExplorerNet MLP) -> 速度指令 -> AirSim

支持：
    1) 直接运行： python3 main.py
    2) roslaunch 启动： roslaunch airsim_controller task3_slam.launch
    3) main.sh / main.bat 一键启动

运行前要求：
    - Windows 端 AirSim（Unreal）已启动并配置 Lidar1（settings.json）
    - 仿真模式为 Multirotor
"""
import time
import sys
import os
import math

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import Config
from lidar_utils import (
    lidar_points_to_numpy,
    points_to_sector_features,
    normalize_features,
    goal_direction_feature,
)
from explorer_net import ExplorerNet


def make_control(v_norm, w_norm, cfg):
    """将网络归一化输出映射为实际速度指令。"""
    v = (v_norm + 1.0) / 2.0 * cfg.V_MAX
    w = w_norm * cfg.W_MAX
    return v, w


def safe_override(v, w, features, cfg):
    """安全兜底层：最近障碍物小于安全距离时强制减速转向，防止碰撞。"""
    d_min = float(features.min())
    if d_min < cfg.SAFE_DIST:
        # 找到最近障碍物所在扇区，向远离方向旋转
        sector = int(features.argmin())
        # 扇区中心角
        angle = -np.pi + (sector + 0.5) * (2 * np.pi / cfg.NUM_SECTORS)
        # 远离障碍物的旋转方向：若障碍在左侧(angle>0)则向右转(w<0)
        w = -cfg.W_MAX * np.sign(angle) if abs(angle) > 1e-3 else cfg.W_MAX
        v = min(v, 0.3 * cfg.V_MAX)
    return v, w


def quat_to_yaw(q):
    """四元数(x,y,z,w)转偏航角 yaw(rad)。"""
    siny_cosp = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
    cosy_cosp = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    cfg = Config()
    print("=" * 56)
    print(" task3_slam_nav : LiDAR 神经网络 SLAM 导航")
    print(f" 扇区数={cfg.NUM_SECTORS}  网络输入={cfg.INPUT_DIM}  "
          f"控制频率={cfg.CTRL_RATE}Hz")
    print("=" * 56)

    # 1. 连接 AirSim
    try:
        import airsim
    except ImportError:
        print("[错误] 未安装 airsim 客户端，请执行: pip install airsim")
        sys.exit(1)

    client = airsim.MultirotorClient(ip=cfg.AIRSIM_IP, port=cfg.AIRSIM_PORT)
    client.confirmConnection()
    client.enableApiControl(True, cfg.VEHICLE_NAME)
    client.armDisarm(True, cfg.VEHICLE_NAME)
    client.takeoffAsync(2.0, cfg.VEHICLE_NAME).join()
    client.hoverAsync().join()
    print("[AirSim] 连接成功，无人机已起飞悬停。")

    # 2. 加载神经网络策略
    net = ExplorerNet(input_dim=cfg.INPUT_DIM, hidden_dim=cfg.HIDDEN_DIM,
                      output_dim=cfg.OUTPUT_DIM, weight_path=cfg.WEIGHT_PATH)
    loaded = net.load(cfg.WEIGHT_PATH)
    print(f"[Net] 权重加载: {'成功' if loaded else '未找到(使用随机初始化+安全兜底)'}")

    # 3. 设定目标点（多场景测试时可从命令行传入）
    start_pos = client.getMultirotorState().kinematics_estimated.position
    start_xy = np.array([start_pos.x_val, start_pos.y_val])
    goal_xy = start_xy + np.array([8.0, 8.0], dtype=np.float64)
    if len(sys.argv) >= 3:
        goal_xy = np.array([float(sys.argv[1]), float(sys.argv[2])])
    print(f"[Goal] 目标点: ({goal_xy[0]:.1f}, {goal_xy[1]:.1f})")

    # 4. 主控制循环
    start_t = time.time()
    reached = False
    min_obs_dist = float("inf")
    step = 0
    period = 1.0 / cfg.CTRL_RATE

    try:
        while True:
            t0 = time.time()

            # ---- 感知：获取 LiDAR 点云 -> 扇区特征 ----
            lidar = client.getLidarData(cfg.LIDAR_NAME, cfg.VEHICLE_NAME)
            pts = lidar_points_to_numpy(lidar, max_range=cfg.MAX_RANGE)
            features = points_to_sector_features(pts, cfg.NUM_SECTORS, cfg.MAX_RANGE)
            feats_norm = normalize_features(features, cfg.SECTOR_MAX)
            min_obs_dist = min(min_obs_dist, float(features.min()))

            # ---- 状态：当前位姿 ----
            state = client.getMultirotorState()
            pos = state.kinematics_estimated.position
            yaw = quat_to_yaw(state.kinematics_estimated.orientation)
            current_xy = np.array([pos.x_val, pos.y_val])

            # 目标方向（机体坐标）
            goal_dir = goal_direction_feature(yaw, goal_xy - current_xy)

            # ---- 规划/控制：神经网络输出速度指令 ----
            v, w = net.predict(feats_norm, goal_dir)
            v, w = make_control(v, w, cfg)
            v, w = safe_override(v, w, features, cfg)

            # ---- 执行（机体坐标系速度：v 沿机头方向，转向后朝新方向飞）----
            client.moveByVelocityBodyFrameAsync(v, 0.0, 0.0, period,
                                                yaw_mode=airsim.YawMode(is_rate=True,
                                                                        yaw_or_rate=w),
                                                vehicle_name=cfg.VEHICLE_NAME).join()

            # ---- 到达判定 ----
            dist_to_goal = np.linalg.norm(goal_xy - current_xy)
            if dist_to_goal < cfg.GOAL_TOL:
                reached = True
                print(f"[到达] 已到达目标点，耗时 {time.time()-start_t:.1f}s")
                break

            step += 1
            if step % 20 == 0:
                print(f"[Step {step}] 最近障碍 {features.min():.2f}m | "
                      f"目标距离 {dist_to_goal:.2f}m | v={v:.2f} w={w:.2f}")

            # 控制频率节流
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    except KeyboardInterrupt:
        print("\n[中断] 用户终止导航。")
    finally:
        client.hoverAsync().join()
        client.landAsync().join()
        client.armDisarm(False, cfg.VEHICLE_NAME)
        client.enableApiControl(False, cfg.VEHICLE_NAME)
        print("[完成] 已降落并释放控制权。")

    # ---- 性能评价输出 ----
    elapsed = time.time() - start_t
    print("\n===== 任务3 性能评价 =====")
    print(f"  目标点        : ({goal_xy[0]:.1f}, {goal_xy[1]:.1f})")
    print(f"  是否到达      : {'是' if reached else '否'}")
    print(f"  用时          : {elapsed:.1f} s")
    print(f"  最近障碍物距离: {min_obs_dist:.2f} m")
    print("==========================")
    return 0 if reached else 1


if __name__ == "__main__":
    sys.exit(main())
