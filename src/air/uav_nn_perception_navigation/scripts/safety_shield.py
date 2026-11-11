#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
神经网络控制器外层的安全兜底层（任务 2）

网络仍然是控制量的唯一来源，本层不做任何"自动驾驶"决策，只在四种情况下
限制或接管输出：

1. 动作非法：出现 NaN / Inf；
2. 输入过期：雷达特征或里程计超过 STALE_TIMEOUT 没有更新；
3. 输出越界：线速度、偏航角速率超过训练时的物理量程；
4. 近距障碍：12 个扇区的最近距离进入 FRONT/SIDE_BLOCK_DIST 以内。

另外对持续左右摆动的偏航指令做阻尼（OscillationDetector），避免网络在
分布外状态下输出高频抖振。

模块只依赖 numpy，不依赖 rospy / airsim，便于单独做单元测试。
"""

import numpy as np

# 与训练脚本 train_control.py 的量程保持一致
MAX_SPEED = 3.0          # m/s，水平线速度上限
MAX_YAW_RATE = 1.0       # rad/s，偏航角速率上限
MAX_VERTICAL = 1.5       # m/s，垂直速度上限

# 障碍距离阈值（米）
FRONT_BLOCK_DIST = 1.2   # 正前方小于此距离：禁止继续前进
SIDE_BLOCK_DIST = 0.9    # 单侧小于此距离：禁止朝该侧平移
HARD_HOVER_DIST = 0.5    # 正前方小于此距离：直接悬停

STALE_TIMEOUT = 0.5      # s，输入超过此时长未更新视为失效

# 12 个扇区覆盖 ±30°，每个 5°；正角在机体系右侧
FRONT_SECTORS = slice(4, 8)    # -7.5° ~ +7.5°
LEFT_SECTORS = slice(0, 4)     # -30° ~ -5°，正角向右，所以负角是左侧
RIGHT_SECTORS = slice(8, 12)   # +5° ~ +30°

# 抖动检测
OSC_WINDOW = 8           # 观察最近 N 次偏航指令
OSC_MIN_FLIPS = 5        # 符号翻转次数达到此值判定为抖动
OSC_MIN_AMPLITUDE = 0.3  # rad/s，翻转幅度太小不算抖动
OSC_DAMP = 0.5           # 抖动时偏航指令乘以该系数


class ShieldResult(object):
    """安全层输出：限幅后的动作 + 是否介入 + 原因。"""

    __slots__ = ('vx', 'vy', 'yaw_rate', 'active', 'reason')

    def __init__(self, vx=0.0, vy=0.0, yaw_rate=0.0, active=False, reason=''):
        self.vx = float(vx)
        self.vy = float(vy)
        self.yaw_rate = float(yaw_rate)
        self.active = bool(active)
        self.reason = reason

    def as_tuple(self):
        return self.vx, self.vy, self.yaw_rate

    def __repr__(self):
        return ('ShieldResult(vx=%.2f, vy=%.2f, yaw=%.2f, active=%s, reason=%s)'
                % (self.vx, self.vy, self.yaw_rate, self.active, self.reason))


def is_finite(*values):
    """所有标量都是有限实数。"""
    for value in values:
        if value is None:
            return False
        try:
            if not np.isfinite(float(value)):
                return False
        except (TypeError, ValueError):
            return False
    return True


def sector_distances(sectors, max_range=40.0):
    """把归一化的扇区特征还原成米制距离（长度不足时补 max_range）。"""
    if sectors is None:
        return np.full(12, float(max_range), dtype=np.float32)
    arr = np.asarray(sectors, dtype=np.float32).reshape(-1)
    if arr.size < 12:
        arr = np.concatenate([arr, np.ones(12 - arr.size, dtype=np.float32)])
    arr = arr[:12]
    # 特征里 1.0 表示"没有障碍或超量程"
    arr = np.where(np.isfinite(arr), arr, 1.0)
    return np.clip(arr, 0.0, 1.0) * float(max_range)


def clip_action(vx, vy, yaw_rate,
                max_speed=MAX_SPEED, max_yaw_rate=MAX_YAW_RATE):
    """按训练时的物理量程做限幅，返回 (vx, vy, yaw_rate, clipped)。"""
    speed = float(np.hypot(vx, vy))
    clipped = False
    if speed > max_speed and speed > 1e-6:
        scale = max_speed / speed
        vx, vy = vx * scale, vy * scale
        clipped = True
    if yaw_rate > max_yaw_rate:
        yaw_rate, clipped = max_yaw_rate, True
    elif yaw_rate < -max_yaw_rate:
        yaw_rate, clipped = -max_yaw_rate, True
    return float(vx), float(vy), float(yaw_rate), clipped


def apply_shield(vx, vy, yaw_rate, sectors, max_range=40.0,
                 front_block=FRONT_BLOCK_DIST, side_block=SIDE_BLOCK_DIST,
                 hard_hover=HARD_HOVER_DIST):
    """对网络输出做合法性检查、量程限幅与近距障碍屏蔽。

    sectors 为 12 维归一化扇区特征（与 uav_features.extract_sector_features
    的输出一致，单位向量 1.0 表示无障碍）。
    """
    # 1) 合法性：NaN / Inf 一律悬停
    if not is_finite(vx, vy, yaw_rate):
        return ShieldResult(0.0, 0.0, 0.0, True, 'non_finite')

    # 2) 量程限幅
    vx, vy, yaw_rate, clipped = clip_action(vx, vy, yaw_rate)
    reason = 'clip' if clipped else ''

    dists = sector_distances(sectors, max_range)
    front_min = float(np.min(dists[FRONT_SECTORS]))
    left_min = float(np.min(dists[LEFT_SECTORS]))
    right_min = float(np.min(dists[RIGHT_SECTORS]))

    # 3) 正前方极近：直接悬停（优先级最高）
    if front_min < hard_hover:
        return ShieldResult(0.0, 0.0, 0.0, True, 'hard_hover')

    # 4) 正前方过近：禁止前进，保留平移与转向
    if front_min < front_block and vx > 0.0:
        vx = 0.0
        reason = 'front_block'

    # 5) 单侧过近：禁止朝该侧平移（机体系 +y 向右）
    if left_min < side_block and vy < 0.0:
        vy = 0.0
        reason = 'left_block'
    if right_min < side_block and vy > 0.0:
        vy = 0.0
        reason = 'right_block'

    return ShieldResult(vx, vy, yaw_rate, bool(reason), reason)


def is_stale(now, last_lidar, last_odom, timeout=STALE_TIMEOUT):
    """任一传感器时间戳缺失或超时，判定输入失效。"""
    if last_lidar is None or last_odom is None:
        return True
    return (now - last_lidar) > timeout or (now - last_odom) > timeout


class OscillationDetector(object):
    """基于最近 N 次偏航指令的符号翻转次数检测左右摆动。"""

    def __init__(self, window=OSC_WINDOW, min_flips=OSC_MIN_FLIPS,
                 min_amplitude=OSC_MIN_AMPLITUDE, damp=OSC_DAMP):
        self.window = int(window)
        self.min_flips = int(min_flips)
        self.min_amplitude = float(min_amplitude)
        self.damp = float(damp)
        self.history = []

    def update(self, yaw_rate):
        value = float(yaw_rate) if is_finite(yaw_rate) else 0.0
        if abs(value) >= self.min_amplitude:
            self.history.append(1 if value > 0 else -1)
        else:
            self.history.append(0)
        if len(self.history) > self.window:
            del self.history[:-self.window]

        flips = 0
        for prev, cur in zip(self.history, self.history[1:]):
            if prev and cur and prev != cur:
                flips += 1

        if flips >= self.min_flips:
            return value * self.damp
        return value
