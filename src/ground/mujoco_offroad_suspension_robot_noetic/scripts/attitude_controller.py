#!/usr/bin/env python3
"""越野工况控制律：航向跟踪 + 崎岖路面限速 + 可选倾角差速补偿。

设计原则（全部为可实测量）：
  1. 航向跟踪用滑移转向差速实现，是真实可用的；
  2. 崎岖路面按 |roll|/|pitch| 降速，物理上是有效的（侧倾越大越慢）；
  3. 倾角差速补偿默认关闭，需用同一地形 A/B 对照确认收益后再启用，
     避免"名义上的姿态稳定器"。
"""
import numpy as np


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class OffroadController:
    def __init__(self,
                 wheel_radius=0.145,
                 track_width=0.54,
                 v_nominal=0.95,
                 heading_kp=1.8,
                 yaw_rate_limit=0.85,
                 accel_limit=1.2,
                 roll_soft=0.10,
                 pitch_soft=0.14,
                 tilt_assist_gain=0.0):
        self.r = wheel_radius
        self.track = track_width
        self.v_nominal = v_nominal
        self.heading_kp = heading_kp
        self.yaw_rate_limit = yaw_rate_limit
        self.accel_limit = accel_limit
        self.roll_soft = roll_soft
        self.pitch_soft = pitch_soft
        self.tilt_assist_gain = tilt_assist_gain
        self.v_ref = 0.0

    def command(self, x, y, yaw, roll, pitch, v_body, target, dt):
        tx, ty = target
        dx, dy = tx - x, ty - y
        dist = float(np.hypot(dx, dy))

        desired_yaw = float(np.arctan2(dy, dx))
        heading_err = wrap_angle(desired_yaw - yaw)
        yaw_rate = float(np.clip(self.heading_kp * heading_err,
                                 -self.yaw_rate_limit, self.yaw_rate_limit))

        # 基础目标速度：航向偏差越大越慢
        v_t = self.v_nominal * float(np.clip(np.cos(heading_err), 0.15, 1.0))
        # 崎岖路面限速：侧倾/俯仰超过软阈值就降速（真实有效的"姿态保护"）
        rough = 1.6 * max(0.0, abs(roll) - self.roll_soft) + \
                1.4 * max(0.0, abs(pitch) - self.pitch_soft)
        v_t *= float(np.clip(1.0 - rough, 0.30, 1.0))
        # 接近目标点减速
        if dist < 2.0:
            v_t = min(v_t, 0.30 + 0.35 * dist)
        if dist < 0.72:
            v_t = 0.0
        # 纵向加速度限制，避免起步打滑/急停点头
        dv = float(np.clip(v_t - self.v_ref, -self.accel_limit * dt, self.accel_limit * dt))
        self.v_ref += dv
        v_cmd = self.v_ref

        w_left = (v_cmd - yaw_rate * self.track / 2.0) / self.r
        w_right = (v_cmd + yaw_rate * self.track / 2.0) / self.r

        if self.tilt_assist_gain:
            comp = self.tilt_assist_gain * roll
            w_left += comp
            w_right -= comp

        w_left = float(np.clip(w_left, -9.0, 9.0))
        w_right = float(np.clip(w_right, -9.0, 9.0))

        return {
            "w_left": w_left,
            "w_right": w_right,
            "v_cmd": v_cmd,
            "yaw_rate": yaw_rate,
            "heading_err": heading_err,
            "dist": dist,
            "v_body": v_body,
        }
