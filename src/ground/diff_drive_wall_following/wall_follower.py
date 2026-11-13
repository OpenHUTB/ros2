#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""差速小车沿右侧墙壁行驶的控制器（纯 Python，不依赖 ROS）。

控制器只接收几路激光测距值，输出差速小车的线速度 v 和角速度 w：

    front        正前方最近距离
    side         正右方最近距离（跟随的目标墙）
    front_right  右前方 45° 左右的距离
    back_right   右后方 45° 左右的距离
    left         正左方最近距离（预留，用于窄通道判断）

沿墙策略：
1. 正常跟随时，用“右侧距离误差”把车拉回目标距离；
2. 用“右前 / 右后距离差”估计车身与墙的夹角，夹角偏了就转向修正；
3. 正前方太近说明到了墙角，原地左转，直到前方重新变空再继续跟随。

角速度约定：逆时针为正（左转为正）。
"""
from __future__ import print_function

import math

STATE_FOLLOW = 'FOLLOW'
STATE_TURN = 'TURN'

STATE_NAMES = {
    STATE_FOLLOW: '沿墙',
    STATE_TURN: '转角',
}


def _clamp(value, limit):
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


class WallFollower(object):
    """右侧沿墙控制器。"""

    def __init__(self, target_distance=0.5, k_distance=1.6, k_align=1.2,
                 max_angular=0.9, base_speed=0.35, turn_speed=0.8,
                 front_block_distance=0.75, front_clear_distance=1.1,
                 min_speed_factor=0.35, lost_range=3.0):
        self.target_distance = float(target_distance)
        self.k_distance = float(k_distance)
        self.k_align = float(k_align)
        self.max_angular = float(max_angular)
        self.base_speed = float(base_speed)
        self.turn_speed = float(turn_speed)
        self.front_block_distance = float(front_block_distance)
        self.front_clear_distance = float(front_clear_distance)
        self.min_speed_factor = float(min_speed_factor)
        self.lost_range = float(lost_range)

        self.state = STATE_FOLLOW

    def reset(self):
        self.state = STATE_FOLLOW

    def update(self, front, side, front_right, back_right, left, dt):
        """返回 (v, w, state)。dt 目前未参与计算，保留以保持接口一致。"""
        if front < self.front_block_distance:
            # 前方是墙：停车并原地左转
            self.state = STATE_TURN
            return 0.0, self.turn_speed, self.state

        if self.state == STATE_TURN:
            if front < self.front_clear_distance:
                return 0.0, self.turn_speed, self.state
            self.state = STATE_FOLLOW

        if not math.isfinite(side) or side >= self.lost_range:
            # 右侧没有可跟随的墙，缓慢右转寻找
            return self.base_speed * self.min_speed_factor, -0.4, self.state

        distance_error = side - self.target_distance      # 正：离墙太远

        # 激光某一路没有有效回波时返回 inf；两路都缺失会得到 inf-inf=NaN。
        # 这里用侧向墙距兜底缺失的一路，仍无法兜底时跳过夹角修正。
        if not math.isfinite(front_right):
            front_right = side
        if not math.isfinite(back_right):
            back_right = side
        if math.isfinite(front_right) and math.isfinite(back_right):
            align_error = front_right - back_right         # 正：车头偏向外侧
        else:
            align_error = 0.0

        w = -(self.k_distance * distance_error + self.k_align * align_error)
        w = _clamp(w, self.max_angular)

        speed_factor = 1.0 - 0.8 * abs(w) / self.max_angular
        if speed_factor < self.min_speed_factor:
            speed_factor = self.min_speed_factor
        v = self.base_speed * speed_factor
        return v, w, self.state
