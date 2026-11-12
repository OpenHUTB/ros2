#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务 2：雷达感知 + DQN 自主导航训练脚本

训练循环完全在 AirSim Blocks 场景里进行，不依赖 ROS：
  1. 每个回合随机摆放到起点（高度 4 m，块区内部），随机生成目标点
  2. 读取激光雷达点云，压成 32 维状态
  3. 按 epsilon-greedy 选动作，换算成世界系速度下发
  4. 用“接近目标的进度 + 朝向对齐 + 避障惩罚 + 碰撞/到达终止”计算奖励
  5. 存入经验池，抽样更新 Dueling Double DQN

用法：
    python3 train_dqn_nav.py --episodes 200 --ip 172.25.176.1
    python3 train_dqn_nav.py --resume models/dqn_nav.pt
"""

import argparse
import csv
import math
import os
import random
import sys
import time

import numpy as np
import torch

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from dqn_agent import DQNAgent  # noqa: E402
from dqn_network import ACTIONS, NUM_ACTIONS  # noqa: E402
from lidar_features import (  # noqa: E402
    MAX_RANGE, build_state, min_obstacle_distance, yaw_from_quaternion,
)

_client_dir = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_dir and _client_dir not in sys.path:
    sys.path.insert(0, _client_dir)

import airsim  # noqa: E402

ALTITUDE = 4.0          # 巡航高度（米），Blocks 场景方块高 5 m
DT = 0.1                # 决策周期（秒）
WORLD_LIMIT = 45.0      # 飞出这个半径视为出界
GOAL_TOL = 1.5          # 到达目标的距离阈值（米）
MAX_STEPS = 200         # 单回合最大步数（20 秒）
MODEL_DIR = os.path.join(os.path.dirname(_SCRIPT_DIR), 'models')


class DQNTrainer(object):
    def __init__(self, ip, vehicle, episodes, resume=None, seed=0,
                 max_steps=MAX_STEPS, free_points=None):
        random.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)

        self.client = airsim.MultirotorClient(ip=ip, timeout_value=15)
        self.client.confirmConnection()
        self.client.enableApiControl(True, vehicle)
        self.client.armDisarm(True, vehicle)
        self.vehicle = vehicle

        self.agent = DQNAgent(state_dim=32, action_dim=NUM_ACTIONS)
        # 断点加载延后到 load_checkpoint()：子类（端到端）要先把 self.agent
        # 替换成图像网络再加载，若在这里按雷达网络先 load，state_dict 的 key
        # 对不上会直接崩。
        self.resume = resume

        self.episodes = episodes
        self.max_steps = max_steps
        self.free_points = self._load_free_points(free_points)
        self.log_path = os.path.join(MODEL_DIR, 'train_nav_log.csv')
        self.log_header = ['episode', 'steps', 'reward', 'distance', 'success',
                           'collision', 'near_miss', 'epsilon', 'loss']
        os.makedirs(MODEL_DIR, exist_ok=True)
        if os.path.exists(self.log_path):
            with open(self.log_path) as f:
                first = f.readline().strip().split(',')
            if first != self.log_header:
                # 旧日志列数不同，先备份，避免追加出坏 CSV
                os.replace(self.log_path, self.log_path + '.bak')
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w', newline='') as f:
                csv.writer(f).writerow(self.log_header)

    def load_checkpoint(self, resume_path):
        """按当前 self.agent 的结构加载断点（含优化器状态与步数）。

        必须在 self.agent 确定为最终网络（雷达 MLP 或端到端图像网络）之后调用。
        """
        if resume_path and os.path.exists(resume_path):
            ckpt = self.agent.load(resume_path)
            self.resume = resume_path
            print('已加载模型 %s' % resume_path)
            return ckpt
        return None

    @staticmethod
    def _load_free_points(path):
        """读取 probe_free_space.py 生成的可通行点表；没有就退回随机采样。"""
        if not path:
            path = os.path.join(os.path.dirname(_SCRIPT_DIR), 'maps',
                                'free_points.json')
        if not os.path.exists(path):
            return []
        try:
            import json
            with open(path) as f:
                data = json.load(f)
            return [(float(p[0]), float(p[1])) for p in data.get('free', [])]
        except Exception:
            return []

    def _sample_start_goal(self):
        if not self.free_points:
            return (random.uniform(-8.0, 8.0), random.uniform(-8.0, 8.0)), \
                   (random.uniform(-25.0, 25.0), random.uniform(-25.0, 25.0))
        start = random.choice(self.free_points)
        far = [p for p in self.free_points
               if 8.0 <= math.hypot(p[0] - start[0], p[1] - start[1]) <= 30.0]
        goal = random.choice(far) if far else random.choice(self.free_points)
        return start, goal

    # ---------- 仿真器交互 ----------
    def reset_episode(self):
        start, goal_xy = self._sample_start_goal()
        x, y = start
        yaw = random.uniform(-math.pi, math.pi)
        pose = airsim.Pose(
            airsim.Vector3r(x, y, -ALTITUDE),
            airsim.to_quaternion(0.0, 0.0, yaw))
        self.client.simSetVehiclePose(pose, ignore_collision=True,
                                      vehicle_name=self.vehicle)
        # 摆好位姿后必须立刻起飞，否则无人机会自由落体
        self.client.enableApiControl(True, self.vehicle)
        self.client.armDisarm(True, self.vehicle)
        self.client.takeoffAsync(timeout_sec=10,
                                 vehicle_name=self.vehicle).join()
        self.client.moveToZAsync(-ALTITUDE, 1.5,
                                 vehicle_name=self.vehicle).join()

        # 记录重置时刻的碰撞时间戳：上一回合的碰撞记录会残留，必须用时间戳区分
        self.collision_baseline = self.client.simGetCollisionInfo(
            vehicle_name=self.vehicle).time_stamp
        self.goal = (goal_xy[0], goal_xy[1], -ALTITUDE)
        return self.read_state()

    def read_state(self):
        state = self.client.getMultirotorState(vehicle_name=self.vehicle)
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        yaw = yaw_from_quaternion(state.kinematics_estimated.orientation)
        return {
            'pos': (pos.x_val, pos.y_val, pos.z_val),
            'vel': (vel.x_val, vel.y_val, vel.z_val),
            'yaw': yaw,
        }

    def read_lidar(self):
        data = self.client.getLidarData(lidar_name='lidar_1',
                                        vehicle_name=self.vehicle)
        arr = np.array(data.point_cloud, dtype=np.float32)
        return arr.reshape(-1, 3) if arr.size else np.zeros((0, 3), np.float32)

    def has_collided(self):
        info = self.client.simGetCollisionInfo(vehicle_name=self.vehicle)
        return bool(info.has_collided and
                    info.time_stamp > getattr(self, 'collision_baseline', 0))

    def send_action(self, action_idx):
        vx_body, yaw_rate = ACTIONS[action_idx]
        st = self.read_state()
        yaw = st['yaw']
        c, s = math.cos(yaw), math.sin(yaw)
        vx = vx_body * c
        vy = vx_body * s
        # ROS 偏航正方向为左，AirSim 为顺时针，取负号
        self.client.moveByVelocityZAsync(
            vx, vy, -ALTITUDE, DT,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True,
                                    yaw_or_rate=-math.degrees(yaw_rate)),
            vehicle_name=self.vehicle)
        time.sleep(DT)

    # ---------- 状态与奖励 ----------
    def build(self, st, points, last_action):
        return build_state(points, st['pos'], self.goal, st['yaw'],
                           st['vel'], last_action)

    def obstacle_distance(self, points):
        return min_obstacle_distance(points, MAX_RANGE)

    @staticmethod
    def potential(pos, goal):
        return math.hypot(goal[0] - pos[0], goal[1] - pos[1])

    def run_episode(self, epsilon, use_lidar=True):
        st = self.reset_episode()
        points = self.read_lidar() if use_lidar else np.zeros((0, 3), np.float32)
        last_action = 0
        state = self.build(st, points, last_action)
        prev_dist = self.potential(st['pos'], self.goal)
        prev_bearing = self.bearing_to_goal(st)

        total_reward = 0.0
        loss = 0.0
        success = False
        collided = False

        for step in range(self.max_steps):
            action = self.agent.act(state, epsilon)
            self.send_action(action)

            st2 = self.read_state()
            points2 = (self.read_lidar() if use_lidar
                       else np.zeros((0, 3), np.float32))
            next_state = self.build(st2, points2, action)

            dist = self.potential(st2['pos'], self.goal)
            bearing = self.bearing_to_goal(st2)

            # 进度项：距离缩短给正奖励；对齐项：机头朝向目标给少量正奖励
            reward = 6.0 * (prev_dist - dist)
            reward += 0.4 * (math.cos(bearing) - math.cos(prev_bearing))
            reward -= 0.03

            # 近障惩罚：1.2 m 内随距离线性加重，逼策略提前转向而不是贴脸直行
            obstacle = self.obstacle_distance(points2)
            if obstacle < 1.2:
                reward -= 1.2 * (1.2 - obstacle) / 1.2

            collided = self.has_collided()
            near_miss = obstacle < 0.5
            done = False
            if collided:
                reward -= 30.0
                done = True
            elif dist < GOAL_TOL:
                reward += 60.0
                success = True
                done = True
            elif near_miss:
                # 训练里没有部署时的安全层，0.5 m 以内按事故处理，
                # 否则策略会学到“贴墙继续前进也没事”
                reward -= 20.0
                done = True
            elif max(abs(st2['pos'][0]), abs(st2['pos'][1])) > WORLD_LIMIT:
                reward -= 20.0
                done = True
            elif step == self.max_steps - 1:
                done = True

            self.agent.remember(state, action, reward, next_state, done)
            total_reward += reward
            state = next_state
            prev_dist, prev_bearing = dist, bearing

            if self.agent.can_learn():
                loss = self.agent.learn()

            if done:
                break

        return {
            'steps': step + 1,
            'reward': total_reward,
            'distance': prev_dist,
            'success': success,
            'collision': collided,
            'near_miss': near_miss,
            'loss': loss,
        }

    def bearing_to_goal(self, st):
        dx = self.goal[0] - st['pos'][0]
        dy = self.goal[1] - st['pos'][1]
        b = math.atan2(dy, dx) - st['yaw']
        return math.atan2(math.sin(b), math.cos(b))

    # ---------- 主循环 ----------
    def train(self):
        rewards, successes = [], []
        epsilon_start, epsilon_end = 1.0, 0.05
        decay_episodes = max(1, int(self.episodes * 0.5))
        best_rate = 0.0

        for ep in range(1, self.episodes + 1):
            frac = min(1.0, (ep - 1) / float(decay_episodes))
            epsilon = epsilon_start + (epsilon_end - epsilon_start) * frac

            try:
                result = self.run_episode(epsilon)
            except Exception as exc:  # pragma: no cover - 仿真侧异常
                print('回合 %d 异常: %s' % (ep, exc))
                time.sleep(2.0)
                continue

            rewards.append(result['reward'])
            successes.append(1.0 if result['success'] else 0.0)
            window = min(20, len(successes))
            rate = sum(successes[-window:]) / float(window)

            print('EP %3d | 步数 %3d | 奖励 %7.2f | 距目标 %5.2f m | '
                  '成功 %s | 碰撞 %s | eps %.3f | loss %.4f | 近20成功率 %.0f%%'
                  % (ep, result['steps'], result['reward'], result['distance'],
                     'Y' if result['success'] else 'N',
                     'Y' if result['collision'] else 'N',
                     epsilon, result['loss'], rate * 100))

            with open(self.log_path, 'a', newline='') as f:
                csv.writer(f).writerow([ep, result['steps'], round(result['reward'], 3),
                                        round(result['distance'], 3),
                                        int(result['success']),
                                        int(result['collision']),
                                        int(result['near_miss']),
                                        round(epsilon, 4),
                                        round(result['loss'], 5)])

            # dqn_nav.pt 永远是可部署的“当前最佳”；最后一回合另存 last
            if ep % 20 == 0:
                self.agent.save(os.path.join(MODEL_DIR, 'dqn_nav_last.pt'))
            if len(successes) >= 20 and rate > best_rate:
                best_rate = rate
                self.agent.save(os.path.join(MODEL_DIR, 'dqn_nav_best.pt'))
                self.agent.save(os.path.join(MODEL_DIR, 'dqn_nav.pt'))

        self.agent.save(os.path.join(MODEL_DIR, 'dqn_nav_last.pt'))
        if best_rate > 0.0 and os.path.exists(
                os.path.join(MODEL_DIR, 'dqn_nav_best.pt')):
            # 保证交付权重就是最佳权重
            best = os.path.join(MODEL_DIR, 'dqn_nav_best.pt')
            with open(best, 'rb') as src, open(
                    os.path.join(MODEL_DIR, 'dqn_nav.pt'), 'wb') as dst:
                dst.write(src.read())
        print('训练完成，最佳近 20 回合成功率 %.0f%%' % (best_rate * 100))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='172.25.176.1')
    parser.add_argument('--vehicle', default='Drone_1')
    parser.add_argument('--episodes', type=int, default=200)
    parser.add_argument('--resume', default=None)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-steps', type=int, default=MAX_STEPS)
    parser.add_argument('--free-points', default=None)
    args = parser.parse_args()

    trainer = DQNTrainer(args.ip, args.vehicle, args.episodes, args.resume,
                         args.seed, args.max_steps, args.free_points)
    trainer.load_checkpoint(args.resume)
    trainer.train()


if __name__ == '__main__':
    main()
