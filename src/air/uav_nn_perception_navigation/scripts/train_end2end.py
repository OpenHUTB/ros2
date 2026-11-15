#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
端到端视觉控制模型训练脚本（任务 4）

思路：
  输入是机载相机拍到的原始图像，输出是速度指令，中间没有任何人工规则。
  训练数据用势场法专家在同一仿真环境里飞出来（目标点始终在机头正前方
  10 米处），因此网络只需要学会“看画面决定怎么飞”。

用法：
    python3 train_end2end.py --episodes 20 --steps 120 --epochs 30
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

from train_control import ACTION_SCALE, expert_action, place_free  # noqa: E402
from uav_features import build_feature_vector, yaw_from_quaternion  # noqa: E402
from scene_image import decode_scene_bgr  # noqa: E402

_client_path = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_path and _client_path not in sys.path:
    sys.path.insert(0, _client_path)

import airsim  # noqa: E402

IMG_H, IMG_W = 36, 64          # 下采样后的输入尺寸（原图 144x256，缩小 4 倍）


def downsample(img, out_h=IMG_H, out_w=IMG_W):
    """块平均下采样，输入 HxWx3 的 uint8 图像，输出 out_h x out_w x 3 的 float32。"""
    h, w = img.shape[0], img.shape[1]
    fh, fw = h // out_h, w // out_w
    if fh == 0 or fw == 0:
        return None
    cropped = img[:out_h * fh, :out_w * fw].astype(np.float32)
    return cropped.reshape(out_h, fh, out_w, fw, 3).mean(axis=(1, 3))


class End2EndNet(nn.Module):
    """图像 -> 控制指令的卷积网络。"""

    def __init__(self):
        super(End2EndNet, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 5, stride=2, padding=2), nn.ReLU(),   # 18x32
            nn.Conv2d(16, 32, 5, stride=2, padding=2), nn.ReLU(),  # 9x16
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(),  # 5x8
        )
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 5 * 8, 128), nn.ReLU(),
            nn.Linear(128, 3),
        )

    def forward(self, x):
        return self.head(self.features(x))


def collect(client, episodes, steps, rng, altitude=-4.0):
    images, actions = [], []
    skipped = 0

    for ep in range(episodes):
        place_free(client, rng, altitude)

        for _ in range(steps):
            state = client.getMultirotorState()
            pos = state.kinematics_estimated.position
            cur = np.array([pos.x_val, pos.y_val, pos.z_val])
            yaw_now = yaw_from_quaternion(state.kinematics_estimated.orientation)

            # 目标点固定在机头正前方 10 米 —— 网络只需要“看画面决定怎么飞”
            goal = np.array([cur[0] + 10.0 * math.cos(yaw_now),
                             cur[1] + 10.0 * math.sin(yaw_now),
                             altitude])

            lidar = client.getLidarData(lidar_name='lidar_1')
            points = np.array(lidar.point_cloud, dtype=np.float32).reshape(-1, 3) \
                if len(lidar.point_cloud) else np.zeros((0, 3), dtype=np.float32)
            feat = build_feature_vector(points, cur, goal, yaw_now)
            action = expert_action(feat)

            resp = client.simGetImages(
                [airsim.ImageRequest('front_center', airsim.ImageType.Scene,
                                     False, False)])[0]
            img = decode_scene_bgr(resp.image_data_uint8, resp.height, resp.width)
            small = downsample(img)
            if small is None:
                continue

            if client.simGetCollisionInfo().has_collided:
                skipped += 1
            else:
                images.append(small.astype(np.float32) / 255.0)
                actions.append(action / ACTION_SCALE)

            client.moveByVelocityZAsync(
                float(action[0]), float(action[1]), altitude, 0.1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True,
                                        yaw_or_rate=math.degrees(float(action[2]))),
            )
            time.sleep(0.1)

        print('  第 %d/%d 回合采集完成，累计样本 %d（丢弃碰撞样本 %d）'
              % (ep + 1, episodes, len(images), skipped))

    return (np.array(images, dtype=np.float32).transpose(0, 3, 1, 2),
            np.array(actions, dtype=np.float32))


def train(images, actions, epochs, out_path, batch_size=64, lr=1e-3):
    torch.manual_seed(0)
    x = torch.from_numpy(images)
    y = torch.from_numpy(actions)

    n = len(x)
    split = int(n * 0.85)
    x_tr, y_tr = x[:split], y[:split]
    x_val, y_val = x[split:], y[split:]

    model = End2EndNet()
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    best = float('inf')
    for epoch in range(1, epochs + 1):
        model.train()
        perm = torch.randperm(len(x_tr))
        total = 0.0
        for start in range(0, len(x_tr), batch_size):
            idx = perm[start:start + batch_size]
            opt.zero_grad()
            loss = loss_fn(model(x_tr[idx]), y_tr[idx])
            loss.backward()
            opt.step()
            total += loss.item() * len(idx)
        total /= max(len(x_tr), 1)

        model.eval()
        with torch.no_grad():
            val = loss_fn(model(x_val), y_val).item()

        if epoch % 5 == 0 or epoch == 1:
            print('  epoch %2d  训练损失 %.5f  验证损失 %.5f' % (epoch, total, val))

        if val < best:
            best = val
            torch.save({
                'state_dict': model.state_dict(),
                'action_scale': ACTION_SCALE.tolist(),
                'img_hw': [IMG_H, IMG_W],
            }, out_path)

    print('最佳验证损失 %.5f，模型已保存到 %s' % (best, out_path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int, default=20)
    parser.add_argument('--steps', type=int, default=120)
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--ip', default=os.environ.get('AIRSIM_IP', '172.25.176.1'))
    parser.add_argument('--vehicle', default='Drone_1')
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    out_path = args.out or os.path.abspath(
        os.path.join(_SCRIPT_DIR, '..', 'models', 'end2end.pt'))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    rng = random.Random(42)

    print('连接 AirSim %s' % args.ip)
    client = airsim.MultirotorClient(ip=args.ip, timeout_value=10)
    client.confirmConnection()
    client.enableApiControl(True, args.vehicle)
    client.armDisarm(True, args.vehicle)

    print('采集图像与控制指令对：%d 回合 x %d 步' % (args.episodes, args.steps))
    images, actions = collect(client, args.episodes, args.steps, rng)
    print('样本总数 %d，图像尺寸 %s' % (len(images), images.shape[1:]))

    print('开始训练')
    train(images, actions, args.epochs, out_path)

    client.armDisarm(False, args.vehicle)
    client.enableApiControl(False, args.vehicle)
    print('完成')


if __name__ == '__main__':
    main()
