#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
神经网络控制器训练脚本（任务 2）

思路：模仿学习（Imitation Learning）
  1. 在 AirSim 里用一个人工设计的“势场法”专家飞若干回合，记录
     (激光雷达特征 + 目标相对量) -> (速度指令) 的样本对
  2. 用这些样本训练一个三层 MLP
  3. 导出 models/controller.pt，供 trajectory_control.py 在线推理

用法：
    python3 train_control.py --episodes 10 --steps 80
"""

import argparse
import math
import os
import random
import sys
import time

import numpy as np
import torch
import torch.nn as nn

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from uav_features import (  # noqa: E402
    FEATURE_DIM, MAX_RANGE, build_feature_vector, yaw_from_quaternion,
)

_client_path = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_path and _client_path not in sys.path:
    sys.path.insert(0, _client_path)

import airsim  # noqa: E402

# 网络输出对应的物理量程，训练时把动作除以它，推理时再乘回去
ACTION_SCALE = np.array([3.0, 2.0, 1.0], dtype=np.float32)   # vx, vy, yaw_rate

SAFE_DIST = 6.0        # 开始产生斥力的距离（米）
KP_GOAL = 0.45
KP_REPULSE = 4.0
MAX_SPEED = 3.0


class LidarPolicy(nn.Module):
    """16 维输入 -> 3 维动作的小型 MLP。"""

    def __init__(self, in_dim=FEATURE_DIM, hidden=64):
        super(LidarPolicy, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 3),
        )

    def forward(self, x):
        return self.net(x)


# ---------------------------------------------------------------- 专家策略
def expert_action(feature_vec):
    """势场法专家：目标吸引 + 障碍斥力，返回 (vx, vy, yaw_rate)。"""
    sectors = feature_vec[:12]
    dist_norm, sin_b, cos_b, _dz = feature_vec[12:]

    bearing = math.atan2(sin_b, cos_b)
    goal_dist = dist_norm * 20.0

    # 目标吸引力，机体系
    vx = KP_GOAL * goal_dist * cos_b
    vy = KP_GOAL * goal_dist * sin_b

    # 障碍斥力：每个扇区中心角 -30 + (i+0.5)*5 度
    for i, d_norm in enumerate(sectors):
        d = float(d_norm) * MAX_RANGE
        if d >= SAFE_DIST:
            continue
        angle = math.radians(-30.0 + (i + 0.5) * 5.0)
        strength = KP_REPULSE * (1.0 - d / SAFE_DIST)
        vx -= strength * math.cos(angle)
        vy -= strength * math.sin(angle)

    # 限速
    speed = math.hypot(vx, vy)
    if speed > MAX_SPEED:
        vx, vy = vx / speed * MAX_SPEED, vy / speed * MAX_SPEED

    yaw_rate = max(-1.0, min(1.0, 1.2 * bearing))
    return np.array([vx, vy, yaw_rate], dtype=np.float32)


# ---------------------------------------------------------------- 数据采集
def random_pose(rng):
    x = rng.uniform(-25.0, 25.0)
    y = rng.uniform(-25.0, 25.0)
    yaw = rng.uniform(-math.pi, math.pi)
    return x, y, yaw


def place_free(client, rng, altitude, tries=25):
    """随机放置无人机；如果落点撞进几何体里就重新掷，避免采到卡死的样本。"""
    sx, sy, yaw = random_pose(rng)
    for _ in range(tries):
        sx, sy, yaw = random_pose(rng)
        client.simSetVehiclePose(
            airsim.Pose(airsim.Vector3r(sx, sy, altitude),
                        airsim.to_quaternion(0, 0, yaw)), True)
        time.sleep(0.25)
        if not client.simGetCollisionInfo().has_collided:
            return sx, sy, yaw
    return sx, sy, yaw


def collect(client, episodes, steps, rng, altitude=-5.0):
    samples_x, samples_y = [], []
    skipped = 0

    for ep in range(episodes):
        sx, sy, yaw = place_free(client, rng, altitude)
        goal = np.array([rng.uniform(-30, 30), rng.uniform(-30, 30), altitude])

        for _ in range(steps):
            state = client.getMultirotorState()
            pos = state.kinematics_estimated.position
            cur = np.array([pos.x_val, pos.y_val, pos.z_val])
            yaw_now = yaw_from_quaternion(state.kinematics_estimated.orientation)

            lidar = client.getLidarData(lidar_name='lidar_1')
            points = np.array(lidar.point_cloud, dtype=np.float32).reshape(-1, 3) \
                if len(lidar.point_cloud) else np.zeros((0, 3), dtype=np.float32)

            feat = build_feature_vector(points, cur, goal, yaw_now)
            action = expert_action(feat)

            # 撞住/贴墙的样本丢掉：这种状态下指令和实际运动对不上
            if client.simGetCollisionInfo().has_collided:
                skipped += 1
            else:
                samples_x.append(feat)
                samples_y.append(action / ACTION_SCALE)

            client.moveByVelocityZAsync(
                float(action[0]), float(action[1]), altitude, 0.1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True,
                                        yaw_or_rate=math.degrees(float(action[2]))),
            )
            time.sleep(0.1)

        print('  第 %d/%d 回合采集完成，累计样本 %d（丢弃碰撞样本 %d）'
              % (ep + 1, episodes, len(samples_x), skipped))

    return np.array(samples_x, dtype=np.float32), np.array(samples_y, dtype=np.float32)


# ---------------------------------------------------------------- 训练
def train(x, y, epochs, out_path, batch_size=256, lr=1e-3):
    torch.manual_seed(0)
    rng = np.random.RandomState(0)
    order = rng.permutation(len(x))
    x, y = x[order], y[order]

    # 特征标准化：雷达扇区和目标量纲差异大，不归一化很难收敛
    feat_mean = x.mean(axis=0)
    feat_std = x.std(axis=0)
    feat_std[feat_std < 1e-6] = 1.0

    x_t = torch.from_numpy((x - feat_mean) / feat_std)
    y_t = torch.from_numpy(y)

    split = int(len(x_t) * 0.85)
    x_tr, y_tr = x_t[:split], y_t[:split]
    x_val, y_val = x_t[split:], y_t[split:]

    model = LidarPolicy()
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    best_val = float('inf')
    history = []
    n = len(x_tr)
    for epoch in range(1, epochs + 1):
        model.train()
        perm = torch.randperm(n)
        epoch_loss = 0.0
        for start in range(0, n, batch_size):
            idx = perm[start:start + batch_size]
            opt.zero_grad()
            loss = loss_fn(model(x_tr[idx]), y_tr[idx])
            loss.backward()
            opt.step()
            epoch_loss += loss.item() * len(idx)
        epoch_loss /= max(n, 1)

        model.eval()
        with torch.no_grad():
            val = loss_fn(model(x_val), y_val).item()

        history.append((epoch, epoch_loss, val))
        if epoch % 20 == 0 or epoch == 1:
            print('  epoch %3d  训练损失 %.5f  验证损失 %.5f'
                  % (epoch, epoch_loss, val))

        if val < best_val:
            best_val = val
            torch.save({
                'state_dict': model.state_dict(),
                'in_dim': FEATURE_DIM,
                'action_scale': ACTION_SCALE.tolist(),
                'feat_mean': feat_mean.astype(np.float32),
                'feat_std': feat_std.astype(np.float32),
            }, out_path)

    print('最佳验证损失 %.5f，模型已保存到 %s' % (best_val, out_path))
    return history


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int, default=24)
    parser.add_argument('--steps', type=int, default=120)
    parser.add_argument('--epochs', type=int, default=200)
    parser.add_argument('--ip', default=os.environ.get('AIRSIM_IP', '172.25.176.1'))
    parser.add_argument('--vehicle', default='Drone_1')
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    rng = random.Random(0)

    out_path = args.out or os.path.join(_SCRIPT_DIR, '..', 'models', 'controller.pt')
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print('连接 AirSim %s' % args.ip)
    client = airsim.MultirotorClient(ip=args.ip, timeout_value=10)
    client.confirmConnection()
    client.enableApiControl(True, args.vehicle)
    client.armDisarm(True, args.vehicle)

    print('开始采集专家数据：%d 回合 x %d 步' % (args.episodes, args.steps))
    x, y = collect(client, args.episodes, args.steps, rng)
    print('样本总数 %d，特征维度 %d' % (len(x), x.shape[1]))

    print('开始训练')
    train(x, y, args.epochs, out_path)

    client.armDisarm(False, args.vehicle)
    client.enableApiControl(False, args.vehicle)
    print('完成')


if __name__ == '__main__':
    main()
