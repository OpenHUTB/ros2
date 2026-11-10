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
import copy
import json
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

# 被 import 的模块在 devel 空间里不在 sys.path 上（CMake 用的是 install(FILES)），
# 通过 rospkg 找到源码包目录补进来，rosrun 走 relay 脚本时也能正常 import
try:
    import rospkg as _rospkg
    _pkg_scripts = os.path.join(
        _rospkg.RosPack().get_path('uav_nn_perception_navigation'), 'scripts')
    if os.path.isdir(_pkg_scripts) and _pkg_scripts not in sys.path:
        sys.path.insert(0, _pkg_scripts)
except Exception:
    pass

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


def expert_batch(x_raw):
    """对一批原始特征算专家的物理量动作（未除以 ACTION_SCALE）。"""
    return np.stack([expert_action(f) for f in x_raw]).astype(np.float32)


def _deviation_stats(net_phys, expert_phys):
    """网络输出与专家动作的偏差统计（都是物理单位）。"""
    diff = net_phys - expert_phys
    err = np.linalg.norm(diff, axis=1)
    net_speed = np.hypot(net_phys[:, 0], net_phys[:, 1])
    exp_speed = np.hypot(expert_phys[:, 0], expert_phys[:, 1])
    heading_err = np.abs(np.arctan2(
        np.sin(net_phys[:, 2] - expert_phys[:, 2]),
        np.cos(net_phys[:, 2] - expert_phys[:, 2])))
    return {
        'dim_mae': {
            'vx': float(np.abs(diff[:, 0]).mean()),
            'vy': float(np.abs(diff[:, 1]).mean()),
            'yaw_rate': float(np.abs(diff[:, 2]).mean()),
        },
        'dim_rmse': {
            'vx': float(np.sqrt((diff[:, 0] ** 2).mean())),
            'vy': float(np.sqrt((diff[:, 1] ** 2).mean())),
            'yaw_rate': float(np.sqrt((diff[:, 2] ** 2).mean())),
        },
        'action_mae': float(np.abs(diff).mean()),
        'action_rmse': float(np.sqrt((diff ** 2).mean())),
        'norm_error_mean': float(err.mean()),
        'norm_error_p95': float(np.percentile(err, 95)),
        'norm_error_max': float(err.max()),
        'speed_mae': float(np.abs(net_speed - exp_speed).mean()),
        'speed_clip_ratio': float((np.abs(net_speed) > MAX_SPEED + 1e-6).mean()),
        'heading_error_mean_deg': float(np.degrees(heading_err).mean()),
    }


def perturb_features(x_raw, rng, sector_noise=0.10, bearing_deg=5.0,
                     dist_noise=0.10):
    """构造分布外状态：扇区距离抖动、目标方位角旋转、目标距离缩放。"""
    out = np.array(x_raw, dtype=np.float32, copy=True)
    n = len(out)
    if n == 0:
        return out

    out[:, :12] *= (1.0 + rng.uniform(-sector_noise, sector_noise,
                                      size=(n, 12))).astype(np.float32)
    np.clip(out[:, :12], 0.0, 1.0, out=out[:, :12])

    delta = rng.uniform(-math.radians(bearing_deg),
                        math.radians(bearing_deg), n).astype(np.float32)
    sin_b = out[:, 13].copy()
    cos_b = out[:, 14].copy()
    out[:, 13] = sin_b * np.cos(delta) + cos_b * np.sin(delta)
    out[:, 14] = cos_b * np.cos(delta) - sin_b * np.sin(delta)

    out[:, 12] *= (1.0 + rng.uniform(-dist_noise, dist_noise, n)).astype(np.float32)
    np.clip(out[:, 12], 0.0, 1.0, out=out[:, 12])
    return out


def evaluate_expert_deviation(model, x_val_raw, feat_mean, feat_std,
                              action_scale, seed=0):
    """验证集上的开环偏差：网络 vs 势场法专家，含分布外扰动版本。"""
    rng = np.random.RandomState(seed)
    model.eval()

    def _net(raw):
        norm = (raw - feat_mean) / feat_std
        with torch.no_grad():
            out = model(torch.from_numpy(norm.astype(np.float32))).numpy()
        return out * action_scale

    expert = expert_batch(x_val_raw)
    in_dist = _deviation_stats(_net(x_val_raw), expert)

    x_ood = perturb_features(x_val_raw, rng)
    ood = _deviation_stats(_net(x_ood), expert_batch(x_ood))
    return {'in_distribution': in_dist, 'ood_perturbed': ood,
            'ood_perturbation': {'sector_noise': 0.10,
                                 'bearing_deg': 5.0,
                                 'goal_dist_noise': 0.10}}


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

            # 专家动作是机体系速度，moveByVelocityZ 收世界系速度，先旋转再发
            psi = float(yaw_now)
            world_vx = float(action[0]) * math.cos(psi) - float(action[1]) * math.sin(psi)
            world_vy = float(action[0]) * math.sin(psi) + float(action[1]) * math.cos(psi)
            client.moveByVelocityZAsync(
                world_vx, world_vy, altitude, 0.1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True,
                                        yaw_or_rate=math.degrees(float(action[2]))),
            )
            time.sleep(0.1)

        print('  第 %d/%d 回合采集完成，累计样本 %d（丢弃碰撞样本 %d）'
              % (ep + 1, episodes, len(samples_x), skipped))

    return np.array(samples_x, dtype=np.float32), np.array(samples_y, dtype=np.float32)


# ---------------------------------------------------------------- 训练
def train(x, y, epochs, out_path, batch_size=256, lr=1e-3, metrics_path=None):
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
    x_val_raw = x[split:]          # 原始特征，算专家偏差要用未标准化的值

    model = LidarPolicy()
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    best_val = float('inf')
    best_state = None
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
            best_state = copy.deepcopy(model.state_dict())
            torch.save({
                'state_dict': model.state_dict(),
                'in_dim': FEATURE_DIM,
                'action_scale': ACTION_SCALE.tolist(),
                'feat_mean': feat_mean.astype(np.float32),
                'feat_std': feat_std.astype(np.float32),
            }, out_path)

    if best_state is not None:
        model.load_state_dict(best_state)

    print('最佳验证损失 %.5f，模型已保存到 %s' % (best_val, out_path))

    metrics = evaluate_expert_deviation(model, x_val_raw, feat_mean, feat_std,
                                        ACTION_SCALE)
    metrics.update({
        'val_loss': float(best_val),
        'samples': {'total': int(len(x)), 'train': int(split),
                    'val': int(len(x_val_raw))},
        'epochs': int(epochs),
        'action_scale': ACTION_SCALE.tolist(),
    })
    if metrics_path:
        with open(metrics_path, 'w', encoding='utf-8') as fh:
            json.dump(metrics, fh, ensure_ascii=False, indent=2)
        print('专家偏差统计已写入 %s' % metrics_path)
    _print_deviation(metrics)
    return history, metrics


def _print_deviation(metrics):
    for name, label in (('in_distribution', '验证集(同分布)'),
                        ('ood_perturbed', '扰动状态(分布外)')):
        stats = metrics[name]
        print('  %s: 动作 MAE %.3f, 范数误差 均值 %.3f / P95 %.3f / 最大 %.3f, '
              '速度偏差 %.3f m/s, 航向偏差 %.1f°'
              % (label, stats['action_mae'], stats['norm_error_mean'],
                 stats['norm_error_p95'], stats['norm_error_max'],
                 stats['speed_mae'], stats['heading_error_mean_deg']))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int, default=24)
    parser.add_argument('--steps', type=int, default=120)
    parser.add_argument('--epochs', type=int, default=200)
    parser.add_argument('--ip', default=os.environ.get('AIRSIM_IP', '172.25.176.1'))
    parser.add_argument('--vehicle', default='Drone_1')
    parser.add_argument('--out', default=None)
    # 只做偏差评估：用现成权重 + 新采集的样本算专家偏差，不重新训练
    parser.add_argument('--report-model', default=None)
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

    metrics_path = os.path.join(os.path.dirname(out_path), 'train_metrics.json')

    if args.report_model:
        print('只做偏差评估：%s' % args.report_model)
        checkpoint = torch.load(args.report_model, map_location='cpu')
        model = LidarPolicy(in_dim=checkpoint.get('in_dim', FEATURE_DIM))
        model.load_state_dict(checkpoint['state_dict'])
        model.eval()
        scale = np.array(checkpoint.get('action_scale', ACTION_SCALE.tolist()),
                         dtype=np.float32)
        metrics = evaluate_expert_deviation(
            model, x, checkpoint['feat_mean'], checkpoint['feat_std'], scale)
        metrics.update({'samples': {'total': int(len(x)), 'train': 0,
                                    'val': int(len(x))},
                        'model': os.path.basename(args.report_model)})
        with open(metrics_path, 'w', encoding='utf-8') as fh:
            json.dump(metrics, fh, ensure_ascii=False, indent=2)
        _print_deviation(metrics)
        print('偏差统计已写入 %s' % metrics_path)
    else:
        print('开始训练')
        train(x, y, args.epochs, out_path, metrics_path=metrics_path)

    client.armDisarm(False, args.vehicle)
    client.enableApiControl(False, args.vehicle)
    print('完成')


if __name__ == '__main__':
    main()
