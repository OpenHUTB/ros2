#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务 4：图像端到端 DQN 训练脚本

和任务 2 共用同一套奖励与回合管理，只把状态从“雷达特征”换成“前置相机图像”：
  状态 = (84x84 灰度图, 7 维辅助量[目标距离/方位正余弦/高度差, 机体系速度])
  动作 = 与任务 2 相同的 6 个离散指令

用法：
    python3 train_dqn_e2e.py --episodes 200 --ip 172.25.176.1
"""

import argparse
import os
import shutil
import sys

import numpy as np
import torch

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from dqn_agent import ImageDQNAgent  # noqa: E402
from lidar_features import goal_state, velocity_state  # noqa: E402
from train_dqn_nav import DQNTrainer, MODEL_DIR  # noqa: E402

import cv2  # noqa: E402
import airsim  # noqa: E402


class End2EndTrainer(DQNTrainer):
    IMG_SIZE = 84
    EXTRA_DIM = 7

    def __init__(self, ip, vehicle, episodes, resume=None, seed=0,
                 max_steps=200):
        # 父类构造时 resume 置空、不加载权重；先把 self.agent 换成图像网络，
        # 再调用 load_checkpoint 按图像结构加载，避免 MLP 网络读图像权重崩掉。
        super(End2EndTrainer, self).__init__(ip, vehicle, episodes, None,
                                             seed, max_steps)
        self.agent = ImageDQNAgent(action_dim=6, extra_dim=self.EXTRA_DIM)
        self.load_checkpoint(resume)
        self.latest_obstacle = float('inf')

    def obstacle_distance(self, points):
        return self.latest_obstacle

    def read_image(self):
        """取前置相机场景图，转灰度并缩放到 84x84，归一化到 0~1。"""
        responses = self.client.simGetImages([
            airsim.ImageRequest('front_center', airsim.ImageType.Scene, False, False),
            airsim.ImageRequest('front_center', airsim.ImageType.DepthPlanar, True, False),
        ], vehicle_name=self.vehicle)
        resp = responses[0]
        depth = responses[1]
        self.latest_obstacle = float('inf')
        if depth and depth.image_data_float:
            distances = np.asarray(depth.image_data_float, dtype=np.float32)
            if distances.size == depth.height * depth.width:
                frame_depth = distances.reshape(depth.height, depth.width)
                center = frame_depth[depth.height // 3:2 * depth.height // 3,
                                     depth.width // 3:2 * depth.width // 3]
                valid = center[np.isfinite(center) & (center > 0.1)]
                if valid.size:
                    self.latest_obstacle = float(valid.min())
        if not resp or not resp.image_data_uint8:
            return np.zeros((self.IMG_SIZE, self.IMG_SIZE), dtype=np.float32)

        buf = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        if buf.size == resp.height * resp.width * 4:
            frame = buf.reshape(resp.height, resp.width, 4)[:, :, :3]
        elif buf.size == resp.height * resp.width * 3:
            frame = buf.reshape(resp.height, resp.width, 3)
        else:
            return np.zeros((self.IMG_SIZE, self.IMG_SIZE), dtype=np.float32)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (self.IMG_SIZE, self.IMG_SIZE),
                          interpolation=cv2.INTER_AREA)
        return (gray.astype(np.float32) / 255.0)

    def build(self, st, points, last_action):
        img = self.read_image()
        extra = np.concatenate([
            goal_state(st['pos'], self.goal, st['yaw']),
            velocity_state(st['vel'], st['yaw']),
        ]).astype(np.float32)
        return img, extra

    def train(self):
        import csv
        # 日志文件与任务 2 分开，避免覆盖；表头与任务 2 对齐（含 near_miss）
        self.log_path = os.path.join(MODEL_DIR, 'train_e2e_log.csv')
        self.log_header = ['episode', 'steps', 'reward', 'distance', 'success',
                           'collision', 'near_miss', 'epsilon', 'loss']
        os.makedirs(MODEL_DIR, exist_ok=True)
        if os.path.exists(self.log_path):
            with open(self.log_path) as f:
                first = f.readline().strip().split(',')
            if first != self.log_header:
                os.replace(self.log_path, self.log_path + '.bak')
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w', newline='') as f:
                csv.writer(f).writerow(self.log_header)

        rewards, successes = [], []
        best_rate = 0.0
        decay_episodes = max(1, int(self.episodes * 0.5))

        for ep in range(1, self.episodes + 1):
            frac = min(1.0, (ep - 1) / float(decay_episodes))
            epsilon = 1.0 + (0.05 - 1.0) * frac
            try:
                # 端到端策略纯用图像，不需要雷达点云：关掉每步 read_lidar
                # 的冗余 RPC，避免 200 回合 × 200 步的无效往返把训练时长翻倍。
                result = self.run_episode(epsilon, use_lidar=False)
            except Exception as exc:  # pragma: no cover
                print('回合 %d 异常: %s' % (ep, exc))
                continue

            rewards.append(result['reward'])
            successes.append(1.0 if result['success'] else 0.0)
            window = min(20, len(successes))
            rate = sum(successes[-window:]) / float(window)
            print('EP %3d | 步数 %3d | 奖励 %7.2f | 距目标 %5.2f m | '
                  '成功 %s | eps %.3f | loss %.4f | 近20成功率 %.0f%%'
                  % (ep, result['steps'], result['reward'], result['distance'],
                     'Y' if result['success'] else 'N',
                     epsilon, result['loss'], rate * 100))

            with open(self.log_path, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [ep, result['steps'], round(result['reward'], 3),
                     round(result['distance'], 3), int(result['success']),
                     int(result['collision']), int(result['near_miss']),
                     round(epsilon, 4), round(result['loss'], 5)])

            # dqn_e2e.pt 永远是可部署的“当前最佳”；最后一回合另存 last
            if ep % 20 == 0:
                self.agent.save(os.path.join(MODEL_DIR, 'dqn_e2e_last.pt'))
            if len(successes) >= 20 and (len(successes) == 20 or rate > best_rate):
                best_rate = rate
                best = os.path.join(MODEL_DIR, 'dqn_e2e_best.pt')
                self.agent.save(best, include_optimizer=False)
                shutil.copyfile(best, os.path.join(MODEL_DIR, 'dqn_e2e.pt'))

        self.agent.save(os.path.join(MODEL_DIR, 'dqn_e2e_last.pt'))
        if os.path.exists(
                os.path.join(MODEL_DIR, 'dqn_e2e_best.pt')):
            # 保证交付权重就是最佳权重，而不是最后一回合覆盖的结果
            best = os.path.join(MODEL_DIR, 'dqn_e2e_best.pt')
            shutil.copyfile(best, os.path.join(MODEL_DIR, 'dqn_e2e.pt'))
        print('训练完成，最佳近 20 回合成功率 %.0f%%' % (best_rate * 100))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='172.25.176.1')
    parser.add_argument('--vehicle', default='Drone_1')
    parser.add_argument('--episodes', type=int, default=200)
    parser.add_argument('--resume', default=None)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-steps', type=int, default=200)
    args = parser.parse_args()

    trainer = End2EndTrainer(args.ip, args.vehicle, args.episodes,
                             args.resume, args.seed, args.max_steps)
    trainer.train()


if __name__ == '__main__':
    main()
