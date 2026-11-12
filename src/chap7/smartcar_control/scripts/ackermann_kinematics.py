#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阿克曼转向运动学与 Pure Pursuit 路径跟踪（不依赖 ROS / Gazebo）。

为什么需要
----------
第 7 章的 ``smartcar_control_gazebo.cpp`` 只在键盘遥控时算一次阿克曼解算：
``tan(delta)=t`` → 左右前轮转角、左右后轮线速度。想验证"这套转向几何对不对"
"给定一条路线小车能不能跟住"，只能起 Gazebo 看，改一个参数就要重来一遍。

这个模块把同一套公式抽成纯 Python，并补上两件原来没有的东西：

1. **正运动学**：给定车速和转向角，按自行车模型积分出位姿（``bicycle_step``）；
2. **路径跟踪**：Pure Pursuit 控制器（``PurePursuit``）+ 离线仿真（``simulate``），
   可以直接算横向误差、转向饱和比例这些指标。

所有参数（轴距、轮距、轮半径、最大转角）默认与 C++ 节点里的常量一致，公式也逐项对齐，
所以离线算出来的四轮指令和真车节点是同一套。

只用标准库，Windows / macOS / Ubuntu 都能跑：

    python3 pure_pursuit_demo.py --path circle
"""

from __future__ import annotations

import math

__all__ = [
    "WHEEL_RADIUS",
    "WHEELBASE",
    "TRACK",
    "MAX_STEER",
    "MAX_SPEED",
    "clamp",
    "ackermann_angles",
    "wheel_speeds",
    "curvature",
    "steer_for_curvature",
    "min_turn_radius",
    "bicycle_step",
    "wrap_angle",
    "Path",
    "PurePursuit",
    "simulate",
]

#: 整车参数，与 smartcar_control_gazebo.cpp 完全一致
WHEEL_RADIUS = 0.025    # 车轮半径 (m)
WHEELBASE = 0.1885      # 前后桥轴距 (m)
TRACK = 0.16            # 左右轮距 (m)
MAX_STEER = 0.4         # 最大转向角 (rad)
MAX_SPEED = 1.0         # 最大车速 (m/s)


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def wrap_angle(angle):
    """角度归一化到 (-π, π]。"""
    value = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if value <= -math.pi else value


# ---------------------------------------------------------------------------
# 1. 阿克曼几何
# ---------------------------------------------------------------------------
def ackermann_angles(delta, wheelbase=WHEELBASE, track=TRACK):
    """由虚拟前轮转角解出左右前轮实际转角。

    与 C++ 节点相同的公式::

        left  = atan(L·t / (L − ½·W·t))
        right = atan(L·t / (L + ½·W·t))      t = tan(delta)

    转向内侧的车轮转角更大（转弯半径更小），这正是阿克曼几何要解决的问题：
    如果两轮平行转向，内外轮会互相拖拽。

    返回 ``(left_angle, right_angle)``，单位 rad，左转时为正。
    """
    if wheelbase <= 0 or track < 0:
        raise ValueError("轴距必须为正、轮距不能为负")
    t = math.tan(delta)
    inner = wheelbase - 0.5 * track * t
    outer = wheelbase + 0.5 * track * t
    if inner <= 1e-9 or outer <= 1e-9:
        raise ValueError("转向角过大：内侧轮转角趋近 90°")
    return math.atan(wheelbase * t / inner), math.atan(wheelbase * t / outer)


def wheel_speeds(speed, delta, wheelbase=WHEELBASE, track=TRACK):
    """由车速和虚拟转角解出左右后轮线速度。

    转弯时外侧后轮要走更长的弧，所以更快::

        v_left  = v·(1 − ½·W·t/L)
        v_right = v·(1 + ½·W·t/L)

    返回 ``(left_speed, right_speed)``，单位 m/s。
    """
    if wheelbase <= 0:
        raise ValueError("轴距必须为正")
    t = math.tan(delta)
    factor = 0.5 * track * t / wheelbase
    return speed * (1.0 - factor), speed * (1.0 + factor)


def curvature(delta, wheelbase=WHEELBASE):
    """转角对应的路径曲率 κ = tan(δ)/L。"""
    if wheelbase <= 0:
        raise ValueError("轴距必须为正")
    return math.tan(delta) / wheelbase


def steer_for_curvature(kappa, wheelbase=WHEELBASE, max_steer=MAX_STEER):
    """曲率反解转角，并按最大转角截断。"""
    if wheelbase <= 0:
        raise ValueError("轴距必须为正")
    return clamp(math.atan(kappa * wheelbase), -max_steer, max_steer)


def min_turn_radius(wheelbase=WHEELBASE, max_steer=MAX_STEER):
    """最大转角对应的最小转弯半径 R = L / tan(δ_max)。"""
    if max_steer <= 0 or max_steer >= math.pi / 2:
        raise ValueError("最大转角必须在 (0, π/2) 内")
    return wheelbase / math.tan(max_steer)


def bicycle_step(pose, speed, delta, dt, wheelbase=WHEELBASE):
    """自行车模型积分一步。

    ``pose = (x, y, theta)``，返回新位姿::

        x' = x + v·cos(θ)·dt
        y' = y + v·sin(θ)·dt
        θ' = θ + v·tan(δ)/L·dt
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if wheelbase <= 0:
        raise ValueError("轴距必须为正")
    x, y, theta = pose
    x += speed * math.cos(theta) * dt
    y += speed * math.sin(theta) * dt
    theta = wrap_angle(theta + speed * math.tan(delta) / wheelbase * dt)
    return (x, y, theta)


# ---------------------------------------------------------------------------
# 2. 路径
# ---------------------------------------------------------------------------
class Path(object):
    """折线路径，提供按弧长取点与横向偏差计算。"""

    def __init__(self, points):
        if len(points) < 2:
            raise ValueError("路径至少需要 2 个点")
        self.points = [(float(p[0]), float(p[1])) for p in points]
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(self.points[:-1], self.points[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        if self.cum[-1] <= 0:
            raise ValueError("路径长度为 0")

    @property
    def length(self):
        return self.cum[-1]

    def project(self, pose):
        """把位姿投影到路径上。

        返回 ``(s, cross_track)``：``s`` 是投影点的弧长坐标；``cross_track``
        是有符号横向偏差（米），**路径左侧为正**。
        """
        x, y, _theta = pose
        best = (float("inf"), 0.0, 0.0)
        for i in range(len(self.points) - 1):
            ax, ay = self.points[i]
            bx, by = self.points[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len2 = dx * dx + dy * dy
            if seg_len2 <= 0:
                continue
            t = ((x - ax) * dx + (y - ay) * dy) / seg_len2
            t = clamp(t, 0.0, 1.0)
            px, py = ax + t * dx, ay + t * dy
            dist = math.hypot(x - px, y - py)
            if dist < best[0]:
                seg_len = math.sqrt(seg_len2)
                ux, uy = dx / seg_len, dy / seg_len
                # 二维叉积：位姿在路径左侧时为正
                cross = ux * (y - ay) - uy * (x - ax)
                best = (dist, self.cum[i] + t * seg_len, cross)
        return best[1], best[2]

    def point_at(self, s):
        """取弧长 s 处的点（超出范围时夹到两端）。"""
        s = clamp(s, 0.0, self.length)
        for i in range(len(self.cum) - 1):
            if s <= self.cum[i + 1]:
                seg_len = self.cum[i + 1] - self.cum[i]
                if seg_len <= 0:
                    return self.points[i]
                ratio = (s - self.cum[i]) / seg_len
                ax, ay = self.points[i]
                bx, by = self.points[i + 1]
                return (ax + ratio * (bx - ax), ay + ratio * (by - ay))
        return self.points[-1]


# ---------------------------------------------------------------------------
# 3. Pure Pursuit
# ---------------------------------------------------------------------------
class PurePursuit(object):
    """Pure Pursuit 路径跟踪控制器。

    在路径上取距离当前位置 ``lookahead`` 的前视点，用"车头指向它"对应的
    曲率作为期望曲率::

        κ = 2·sin(α) / L_d          α = 前视点方位角 − 当前航向

    再换算成前轮转角 ``δ = atan(κ·L)`` 并截断到最大转角。前视距离越大越平稳、
    越小越贴线，但过小会在弯道里画龙。
    """

    def __init__(self, path, lookahead=0.5, speed=0.4,
                 wheelbase=WHEELBASE, max_steer=MAX_STEER):
        if lookahead <= 0:
            raise ValueError("前视距离必须为正")
        if speed <= 0:
            raise ValueError("车速必须为正")
        if wheelbase <= 0:
            raise ValueError("轴距必须为正")
        self.path = path
        self.lookahead = float(lookahead)
        self.speed = float(speed)
        self.wheelbase = float(wheelbase)
        self.max_steer = float(max_steer)

    def command(self, pose):
        """返回 ``(speed, steer, target, s)``。

        ``target`` 是前视点，``s`` 是当前投影弧长（用于判断是否到达终点）。
        """
        s, _cross = self.path.project(pose)
        target = self.path.point_at(s + self.lookahead)
        x, y, theta = pose
        alpha = wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        dist = math.hypot(target[0] - x, target[1] - y)
        if dist < 1e-6:
            return self.speed, 0.0, target, s
        kappa = 2.0 * math.sin(alpha) / dist
        steer = clamp(math.atan(kappa * self.wheelbase),
                      -self.max_steer, self.max_steer)
        return self.speed, steer, target, s


# ---------------------------------------------------------------------------
# 4. 离线仿真
# ---------------------------------------------------------------------------
def simulate(path, start_pose=(0.0, 0.0, 0.0), lookahead=0.5, speed=0.4,
             dt=0.01, max_time=60.0, wheelbase=WHEELBASE,
             max_steer=MAX_STEER, finish_tolerance=0.05, warmup=1.0):
    """让小车沿 path 跑一圈，返回轨迹与统计指标。

    返回 dict：
        trajectory    [(t, x, y, theta, steer), …]
        duration      仿真时长 (s)
        distance      实际行驶里程 (m)
        cross_track_rms / cross_track_max   横向偏差（去掉前 warmup 秒起步段；
                                            **样本不足时为 None**，不要当成 0 用）
        samples       参与统计的采样点数（0 表示仿真太短、没有有效样本）
        saturate_ratio  转向被最大转角截断的时间占比
        finished      是否到达路径终点（终点容差 finish_tolerance）
        path_length   路径总长
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")
    controller = PurePursuit(path, lookahead=lookahead, speed=speed,
                             wheelbase=wheelbase, max_steer=max_steer)
    pose = tuple(start_pose)
    t = 0.0
    distance = 0.0
    trajectory = []
    errors = []
    saturated = 0
    steps = 0

    while t < max_time:
        _speed, steer, _target, s = controller.command(pose)
        _s, cross = path.project(pose)
        trajectory.append((t, pose[0], pose[1], pose[2], steer))
        if t > warmup:                   # 跳过起步阶段
            errors.append(abs(cross))
        if abs(abs(steer) - max_steer) < 1e-9:
            saturated += 1
        steps += 1

        new_pose = bicycle_step(pose, speed, steer, dt, wheelbase)
        distance += math.hypot(new_pose[0] - pose[0], new_pose[1] - pose[1])
        pose = new_pose
        t += dt

        if s >= path.length - finish_tolerance:
            break

    _s, cross = path.project(pose)
    if t > warmup:
        errors.append(abs(cross))
    trajectory.append((t, pose[0], pose[1], pose[2], 0.0))

    # 样本不足（仿真时长短于 warmup）时指标无意义，返回 None 而不是 0：
    # 返回 0 会让"车离线 5 m"的短仿真看起来"完美贴合"，把结论带偏。
    if errors:
        rms = math.sqrt(sum(e * e for e in errors) / len(errors))
        cross_max = max(errors)
    else:
        rms = None
        cross_max = None
    return {
        "trajectory": trajectory,
        "duration": t,
        "distance": distance,
        "cross_track_rms": rms,
        "cross_track_max": cross_max,
        "samples": len(errors),
        "saturate_ratio": (saturated / float(steps)) if steps else 0.0,
        "finished": s >= path.length - finish_tolerance,
        "path_length": path.length,
        "final_pose": pose,
    }


# ---------------------------------------------------------------------------
# 5. 常用测试路径
# ---------------------------------------------------------------------------
def circle_path(radius=1.0, samples=180, center=(0.0, 0.0)):
    """圆周路径（逆时针）。"""
    if radius <= 0:
        raise ValueError("半径必须为正")
    cx, cy = center
    return Path([(cx + radius * math.cos(2 * math.pi * i / samples),
                  cy + radius * math.sin(2 * math.pi * i / samples))
                 for i in range(samples + 1)])


def lane_change_path(length=4.0, offset=0.6, samples=120):
    """正弦换道：沿 x 前进 length 米、横向平移 offset 米。"""
    if length <= 0:
        raise ValueError("长度必须为正")
    pts = []
    for i in range(samples + 1):
        x = length * i / samples
        y = 0.5 * offset * (1.0 - math.cos(math.pi * x / length))
        pts.append((x, y))
    return Path(pts)
