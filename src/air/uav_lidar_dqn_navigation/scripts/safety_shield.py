#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DQN 动作安全层（部署用）

训练好的 DQN 主要学的是“朝目标飞”，当障碍进入训练分布之外的近距范围时，
Q 值在动作之间的区分度会变小（实测正前方 0.4 m 时 forward 仍是最大值）。
部署时先用 24 扇区距离把明显会撞的动作屏蔽掉，再在剩余动作里取 Q 最大：
这是一种简单的 action shield，不改变网络本身。

扇区索引约定（见 lidar_features.sector_distances）：
    0  .. 7   -90° ~ -30°（负角，机体左侧）
    8  .. 15  -30° ~ +30°（正前方）
    16 .. 23  +30° ~ +90°（正角，机体右侧）
"""

import numpy as np

NUM_SECTORS = 24
MAX_RANGE = 40.0

FRONT = slice(8, 16)
LEFT = slice(0, 8)
RIGHT = slice(16, 24)

# 动作编号与 dqn_network.ACTIONS 一致
HOVER, FORWARD, FORWARD_LEFT, FORWARD_RIGHT, TURN_LEFT, TURN_RIGHT = range(6)


def action_mask(sectors, front_m=1.0, side_m=0.9):
    """返回 6 个动作的布尔掩码，True 表示允许执行。"""
    d = np.asarray(sectors, dtype=np.float32).reshape(-1) * MAX_RANGE
    if d.size != NUM_SECTORS:
        return np.ones(6, dtype=bool)

    front = float(np.min(d[FRONT]))
    left = float(np.min(d[LEFT]))
    right = float(np.min(d[RIGHT]))

    mask = np.ones(6, dtype=bool)
    if front < front_m:
        mask[FORWARD] = False
        mask[FORWARD_LEFT] = False
        mask[FORWARD_RIGHT] = False
    else:
        if left < side_m:
            mask[FORWARD_LEFT] = False
        if right < side_m:
            mask[FORWARD_RIGHT] = False
    return mask


def select_action(q_values, sectors, front_m=1.0, side_m=0.9):
    """在安全动作集合里取 Q 最大的动作；悬停与原地转向永远允许。"""
    q = np.asarray(q_values, dtype=np.float32).reshape(-1).copy()
    mask = action_mask(sectors, front_m=front_m, side_m=side_m)
    q[~mask] = -np.inf
    return int(np.argmax(q))


def front_distance_m(sectors):
    """Return the nearest obstacle in the forward 60-degree field, in metres."""
    d = np.asarray(sectors, dtype=np.float32).reshape(-1)
    if d.size != NUM_SECTORS or not np.all(np.isfinite(d)):
        return 0.0  # Missing/invalid perception must not authorize forward flight.
    return float(np.min(d[FRONT])) * MAX_RANGE


def recovery_turn(sectors):
    """Turn toward the more open half of the forward lidar field."""
    d = np.asarray(sectors, dtype=np.float32).reshape(-1)
    if d.size != NUM_SECTORS or not np.all(np.isfinite(d)):
        return HOVER
    left_clearance = float(np.min(d[LEFT]))
    right_clearance = float(np.min(d[RIGHT]))
    return TURN_LEFT if left_clearance >= right_clearance else TURN_RIGHT
