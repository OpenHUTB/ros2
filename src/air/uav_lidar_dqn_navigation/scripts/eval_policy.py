#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线评估 DQN 策略（不依赖 ROS / AirSim）

两个用途：

1. 汇总 `models/train_nav_log.csv` 的训练结果（整体 / 末 50 / 末 20 成功率）；
2. 构造“正前方若干米有墙”的状态，打印网络输出的 Q 值与动作，
   同时打印动作屏蔽后的结果——用来复现审核时做的贴墙探测。

用法：
    python3 scripts/eval_policy.py
    python3 scripts/eval_policy.py --model models/dqn_nav.pt --log models/train_nav_log.csv
    python3 scripts/eval_policy.py --self-test     # 只跑几何 / 安全层断言，不需要权重
"""

import argparse
import csv
import os
import pickle
import sys

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from lidar_features import MAX_RANGE, build_state, sector_distances  # noqa: E402

from safety_shield import action_mask, select_action  # noqa: E402


def package_dir():
    source = os.path.dirname(_SCRIPT_DIR)
    if os.path.isdir(os.path.join(source, 'models')):
        return source
    try:
        import rospkg
        return rospkg.RosPack().get_path('uav_lidar_dqn_navigation')
    except Exception:
        return source


def wall_points(distance, half_width=5.0):
    """正前方 distance 米处的一堵横向墙（机体系，x 前 y 右）。"""
    ys = np.linspace(-half_width, half_width, 201)
    return np.stack([
        np.full_like(ys, distance),
        ys,
        np.zeros_like(ys),
    ], axis=1).astype(np.float32)


def state_for_wall(distance, goal=(10.0, 0.0, -4.0)):
    points = wall_points(distance) if distance else np.zeros((0, 3), np.float32)
    return build_state(points, (0.0, 0.0, -4.0), goal, 0.0, (0.0, 0.0, 0.0), 0)


def summarize_log(path):
    if not os.path.exists(path):
        return None
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    if not rows:
        return None

    def rate(sub):
        return 100.0 * sum(r['success'] for r in sub) / max(1, len(sub))

    return {
        'episodes': len(rows),
        'overall': rate(rows),
        'last50': rate(rows[-50:]),
        'last20': rate(rows[-20:]),
        'collisions': int(sum(r.get('collision', 0) for r in rows)),
        'near_miss': int(sum(r.get('near_miss', 0) for r in rows)),
    }


def load_net(model_path):
    import torch
    from dqn_network import DuelingQNet

    net = DuelingQNet()
    try:
        ckpt = torch.load(model_path, map_location='cpu', weights_only=True)
    except pickle.UnpicklingError as exc:
        raise RuntimeError('安全加载检查点失败；本次已验证 PyTorch 2.15.0 可读取随包权重。'
                           '请使用兼容版本或提供可安全加载的权重，不会自动退回不安全加载。') from exc
    net.load_state_dict(ckpt['model'])
    net.eval()
    return net, torch


def probe(model_path, log_path):
    from dqn_network import ACTION_NAMES

    summary = summarize_log(log_path)
    if summary:
        print('训练日志: %s' % log_path)
        print('  回合 %d | 整体 %.1f%% | 末 50 %.1f%% | 末 20 %.1f%% | '
              '碰撞 %d | 近失 %d'
              % (summary['episodes'], summary['overall'], summary['last50'],
                 summary['last20'], summary['collisions'], summary['near_miss']))
    else:
        print('训练日志不存在: %s' % log_path)

    if not os.path.exists(model_path):
        print('\n权重不存在: %s' % model_path)
        return 1

    net, torch = load_net(model_path)
    print('\n模型: %s' % model_path)
    print('%-8s %-14s %-14s %s' % ('障碍', '网络动作', '安全层后', 'Q(6 动作)'))
    for distance in (None, 5.0, 2.0, 1.0, 0.8, 0.4):
        state = state_for_wall(distance)
        with torch.no_grad():
            q = net(torch.as_tensor(state).unsqueeze(0))[0].numpy()
        raw = ACTION_NAMES[int(q.argmax())]
        shielded = ACTION_NAMES[int(select_action(q, state[:24]))]
        label = '无遮挡' if distance is None else '%.1f m' % distance
        print('%-8s %-14s %-14s [%s]'
              % (label, raw, shielded,
                 ', '.join('%.2f' % v for v in q)))
    return 0


def self_test():
    """几何与安全层的离线断言，不加载权重。"""
    clear = sector_distances(np.zeros((0, 3), np.float32))
    near = sector_distances(wall_points(0.4))
    assert float(np.min(clear)) > 0.9, '空旷场景应全部判为通畅'
    assert float(np.min(near)) < 0.05, '0.4 m 墙应让正前扇区接近 0'
    assert float(np.max(near[8:16])) < float(np.min(clear)), '近墙应明显区别于空旷'

    assert not action_mask(near)[1], '近墙时直行应被安全层屏蔽'
    assert action_mask(clear)[1], '空旷时直行应被允许'
    q = np.array([0.0, 10.0, 9.0, 8.0, 2.5, 2.0], dtype=np.float32)
    action = int(select_action(q, near))
    assert action != 1, '网络偏向直行时安全层仍应拦下'
    print('self-test ok（几何与动作屏蔽已验证）')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.path.join(
        package_dir(), 'models', 'dqn_nav.pt'))
    parser.add_argument('--log', default=os.path.join(
        package_dir(), 'models', 'train_nav_log.csv'))
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()

    if args.self_test:
        return self_test()
    return probe(args.model, args.log)


if __name__ == '__main__':
    sys.exit(main())
