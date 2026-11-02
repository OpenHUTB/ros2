#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_trajectory_planner/core.py - 航点轨迹规划核心（纯 Python，无 ROS 依赖）

功能：给定一串航点（NED 坐标），按匀速巡航 + 航点悬停的方式生成时间参数化轨迹：
    - 每段以固定巡航速度直线飞行
    - 到达每个航点后悬停 hover_time 秒
输出按时间采样的位置序列，用于发布 ROS Path 或绘制航迹图。
"""

import math


class WaypointTrajectory:

    def __init__(self, waypoints_ned, cruise_speed=5.0, hover_time=2.0,
                 sample_dt=0.1):
        """
        :param waypoints_ned: [(x,y,z), ...] NED 航点序列, 至少 2 个
        :param cruise_speed: 巡航速度 m/s
        :param hover_time: 每个航点悬停秒数
        :param sample_dt: 采样周期秒
        """
        if len(waypoints_ned) < 2:
            raise ValueError('至少需要 2 个航点')
        if cruise_speed <= 0 or hover_time < 0 or sample_dt <= 0:
            raise ValueError('速度必须为正, 悬停时间非负, 采样周期为正')

        self.waypoints = [tuple(map(float, wp)) for wp in waypoints_ned]
        self.cruise_speed = float(cruise_speed)
        self.hover_time = float(hover_time)
        self.sample_dt = float(sample_dt)

        # 每段长度与时长
        self.segment_lengths = [
            self._distance(self.waypoints[i], self.waypoints[i + 1])
            for i in range(len(self.waypoints) - 1)]
        self.segment_times = [l / self.cruise_speed for l in self.segment_lengths]
        self.total_length = sum(self.segment_lengths)
        # 总时长 = 飞行时间 + (航点数-1) 次悬停
        self.total_time = self.total_length / self.cruise_speed \
            + self.hover_time * (len(self.waypoints) - 1)

    @staticmethod
    def _distance(a, b):
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def position_at(self, t: float):
        """返回时刻 t 的 NED 位置 (线性插值 + 悬停)。"""
        if t <= 0:
            return self.waypoints[0]
        if t >= self.total_time:
            return self.waypoints[-1]

        remaining = t
        for i, length in enumerate(self.segment_lengths):
            # 到达航点 i+1 前先悬停 hover_time
            if remaining <= self.hover_time and i > 0:
                return self.waypoints[i]
            if i > 0:
                remaining -= self.hover_time

            if remaining <= self.segment_times[i]:
                ratio = remaining / self.segment_times[i]
                a, b = self.waypoints[i], self.waypoints[i + 1]
                return tuple(a[k] + ratio * (b[k] - a[k]) for k in range(3))
            remaining -= self.segment_times[i]
        return self.waypoints[-1]

    def sample(self):
        """按 sample_dt 采样全程, 返回 [(t, (x,y,z)), ...] (末点恰为总时长)。"""
        samples = []
        n = int(self.total_time / self.sample_dt) + 1
        for k in range(n):
            t = min(k * self.sample_dt, self.total_time)
            samples.append((t, self.position_at(t)))
        # 保证末点恰为终点 (采样网格可能未对齐总时长)
        if samples and samples[-1][0] < self.total_time:
            samples.append((self.total_time, self.waypoints[-1]))
        return samples
