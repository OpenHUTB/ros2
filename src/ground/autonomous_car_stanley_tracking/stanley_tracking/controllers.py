#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""横向控制器集合。

- `StanleyController`：前轴横向偏差 + 航向偏差 + 曲率前馈，本模块的主角。
- `AdaptivePurePursuitController`：自适应前视距离的纯跟踪控制器，作为对照基线。
- `LookaheadPIDController`：预瞄点横向偏差 PID，作为对照基线。

三个控制器都只输出"期望横摆角速度 + 期望转向角"，与平台无关；
仿真器再按运动学模型把横摆角速度积分成位姿。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .geometry import ProjectionResult, ReferencePath, wrap_to_pi


@dataclass
class ControlCommand:
    """一个控制周期内的控制量。"""

    steering: float          # 期望前轮转角，单位 rad
    yaw_rate: float          # 期望横摆角速度，单位 rad/s
    speed: float             # 期望纵向速度，单位 m/s
    cross_track_error: float = 0.0  # 本周期用于反馈的横向偏差，单位 m
    heading_error: float = 0.0      # 航向偏差，单位 rad
    feedforward: float = 0.0        # 曲率前馈对应的转角，单位 rad


class BaseController:
    """控制器基类：统一限幅、限速率与复位逻辑。"""

    name = "base"

    def __init__(self, wheelbase: float = 2.7, max_steering: float = math.radians(35.0),
                 steering_rate_limit: float = math.radians(120.0)) -> None:
        self.wheelbase = float(wheelbase)
        self.max_steering = float(max_steering)
        self.steering_rate_limit = float(steering_rate_limit)
        self._steering = 0.0
        self._previous_steering = 0.0

    def reset(self) -> None:
        """每个场景开始前复位内部状态。"""
        self._steering = 0.0
        self._previous_steering = 0.0

    def _saturate_steering(self, steering: float, dt: float) -> float:
        """先按物理极限限幅，再按转向速率限幅。"""
        steering = float(np.clip(steering, -self.max_steering, self.max_steering))
        max_delta = self.steering_rate_limit * dt
        steering = float(np.clip(steering, self._steering - max_delta, self._steering + max_delta))
        self._previous_steering = self._steering
        self._steering = steering
        return steering

    def steering_rate(self, dt: float) -> float:
        """上一周期的转向角变化率，用于评价转向平顺性。"""
        if dt <= 0.0:
            return 0.0
        return (self._steering - self._previous_steering) / dt

    def compute(self, state: np.ndarray, speed: float, path: ReferencePath,
                projection: ProjectionResult, dt: float) -> ControlCommand:
        raise NotImplementedError


class StanleyController(BaseController):
    """Stanley 横向控制器（前轴参考点 + 横摆角速度实现）。

    反馈律（Stanley 1985 的经典形式）::

        delta = psi_e - atan( k * e_fa / (k_soft + v) ) + delta_ff

    符号约定（``delta > 0`` 为左转）：

    * ``e_fa`` 为**前轴中心**到参考路径的有符号横向偏差，左偏为正；
    * ``psi_e = wrap(path_yaw - yaw)`` 为**路径切向相对车头**的偏角，
      即经典 Stanley 写法中的航向偏差。

    这两项都必须取上述符号，原因是几何关系本身带负号：
    车辆在路径左侧（``e_fa > 0``）时需要**右转**（``delta < 0``）才能回到路径；
    车头相对路径左偏（``yaw > path_yaw``，此时 ``psi_e < 0``）时同样需要右转。
    若把两处符号写反（等价于改成"车头相对路径"的定义却不翻符号），
    闭环会变成正反馈：横向偏差单调增大，转角很快打满并绕圈飞出。
    实测数据：``e_fa = +0.05 m`` 时若输出正转角，偏差会在 20 s 内增长到 10 m 以上。

    * ``k`` 为横向增益：越大收敛越快，过大则出现转向振荡；
    * ``k_soft`` 为软化项：低速时防止 ``atan`` 项过猛，避免低速蛇行；
    * ``delta_ff = atan(L * kappa)`` 为曲率前馈：左弯 ``kappa > 0``
      给出正转角（左转），直接补偿路径弯曲带来的稳态偏差。

    差速底盘没有真实前轮，这里按自行车模型把转角折算成横摆角速度::

        omega = v * tan(delta) / L

    该折算在有前轮转角的阿克曼底盘上同样成立，因此同一套控制器
    既可以直接驱动 ``/cmd_vel`` 差速小车，也可以给出 Ackermann 底盘的前轮转角。

    控制点选择（``control_point``）——本项目的一个实测结论：

    * ``'front_axle'``：经典 Stanley 取前轴中心。前轴比车辆实际到达的位置
      超前一个轴距，曲率前馈会在车辆前方 L 处取值，转弯时相当于用"更外侧"
      的弯曲信息；在半径 15 m 的圆弧上稳态偏差约 **0.22 m**。
    * ``'rear_axle'``（默认）：在状态点（后轴中心，运动学模型的原点）上取横向
      偏差与前馈，与模型一致。同一场景稳态偏差降到 **0.008 m** 量级（约 25 倍改善），
      代价是横向偏差的响应略慢（前轴超前量相当于天然的前视阻尼）。

    两种控制点都保留，便于在报告里对比；曲率前馈的取值点也可用
    ``feedforward_point`` 单独指定。
    """

    name = "stanley"

    def __init__(self, k: float = 1.6, k_soft: float = 1.2,
                 wheelbase: float = 2.7, max_steering: float = math.radians(35.0),
                 steering_rate_limit: float = math.radians(120.0),
                 feedforward_gain: float = 1.0, use_feedforward: bool = True,
                 control_point: str = "rear_axle",
                 feedforward_point: str = "") -> None:
        super().__init__(wheelbase=wheelbase, max_steering=max_steering,
                         steering_rate_limit=steering_rate_limit)
        self.k = float(k)
        self.k_soft = float(k_soft)
        self.feedforward_gain = float(feedforward_gain)
        self.use_feedforward = bool(use_feedforward)
        if control_point not in ("rear_axle", "front_axle"):
            raise ValueError("control_point 只能是 'rear_axle' 或 'front_axle'")
        self.control_point = control_point
        # 前馈取值点默认跟随控制点；显式指定可做"前轴偏差 + 后轴前馈"这类组合实验
        self.feedforward_point = feedforward_point or control_point

    def _point_at(self, state: np.ndarray, which: str) -> np.ndarray:
        """按名称取出车体上的参考点坐标。"""
        if which == "front_axle":
            return np.array([state[0] + self.wheelbase * math.cos(state[2]),
                             state[1] + self.wheelbase * math.sin(state[2])])
        return np.array([state[0], state[1]])

    def compute(self, state: np.ndarray, speed: float, path: ReferencePath,
                projection: ProjectionResult, dt: float) -> ControlCommand:
        yaw = float(state[2])

        # 1) 控制点处的横向偏差（左偏为正）。
        #    控制点取后轴时车辆位姿本身就是控制点，无需再投影一次。
        if self.control_point == "rear_axle":
            e_ct = projection.signed_distance
            curvature_s = projection.s
        else:
            control_projection = path.project(self._point_at(state, "front_axle"))
            e_ct = control_projection.signed_distance
            curvature_s = control_projection.s

        # 2) 航向偏差：路径切向相对车头（车头左偏时该值为负，对应右转修正）
        path_yaw = math.atan2(projection.tangent[1], projection.tangent[0])
        heading_error = float(wrap_to_pi(path_yaw - yaw))

        # 3) 反馈项：横向偏差取负号（左偏要右转），低速软化避免 atan 项过猛
        denominator = self.k_soft + max(speed, 0.0)
        feedback = -math.atan(self.k * e_ct / denominator)

        # 4) 前馈项：按指定点处的曲率补偿路径弯曲
        feedforward = 0.0
        if self.use_feedforward:
            if self.feedforward_point == "front_axle":
                curvature_s = path.project(self._point_at(state, "front_axle")).s
            curvature = path.curvature_at_s(curvature_s)
            feedforward = self.feedforward_gain * math.atan(self.wheelbase * curvature)

        steering = heading_error + feedback + feedforward
        steering = self._saturate_steering(steering, dt)
        yaw_rate = 0.0 if abs(speed) < 1e-6 else speed * math.tan(steering) / self.wheelbase
        return ControlCommand(steering=steering, yaw_rate=yaw_rate, speed=speed,
                              cross_track_error=e_ct, heading_error=heading_error,
                              feedforward=feedforward)

    def describe(self) -> str:
        return (f"Stanley(k={self.k}, k_soft={self.k_soft}, "
                f"control_point={self.control_point}, "
                f"feedforward={'on' if self.use_feedforward else 'off'})")


class AdaptivePurePursuitController(BaseController):
    """自适应前视距离的纯跟踪控制器（对照基线）。

    前视距离随速度线性增长 ``L_d = clamp(k*v, L_min, L_max)``；
    目标是前视点，转角由几何关系给出::

        delta = atan( 2 * L * sin(alpha) / L_d )

    其中 ``alpha`` 为车头朝向与前视点方向的夹角。
    """

    name = "pure_pursuit"

    def __init__(self, lookahead_gain: float = 1.0, lookahead_min: float = 2.5,
                 lookahead_max: float = 12.0, wheelbase: float = 2.7,
                 max_steering: float = math.radians(35.0),
                 steering_rate_limit: float = math.radians(120.0)) -> None:
        super().__init__(wheelbase=wheelbase, max_steering=max_steering,
                         steering_rate_limit=steering_rate_limit)
        self.lookahead_gain = float(lookahead_gain)
        self.lookahead_min = float(lookahead_min)
        self.lookahead_max = float(lookahead_max)

    def lookahead_distance(self, speed: float) -> float:
        return float(np.clip(self.lookahead_gain * max(speed, 0.0),
                             self.lookahead_min, self.lookahead_max))

    def compute(self, state: np.ndarray, speed: float, path: ReferencePath,
                projection: ProjectionResult, dt: float) -> ControlCommand:
        x, y, yaw = float(state[0]), float(state[1]), float(state[2])
        lookahead = self.lookahead_distance(speed)
        target = path.point_at_s(projection.s + lookahead)

        dx, dy = target[0] - x, target[1] - y
        alpha = float(wrap_to_pi(math.atan2(dy, dx) - yaw))
        distance = max(float(np.hypot(dx, dy)), 1e-6)
        steering = math.atan(2.0 * self.wheelbase * math.sin(alpha) / distance)
        steering = self._saturate_steering(steering, dt)
        yaw_rate = 0.0 if abs(speed) < 1e-6 else speed * math.tan(steering) / self.wheelbase
        path_yaw = math.atan2(projection.tangent[1], projection.tangent[0])
        return ControlCommand(steering=steering, yaw_rate=yaw_rate, speed=speed,
                              cross_track_error=projection.signed_distance,
                              heading_error=float(wrap_to_pi(path_yaw - yaw)))


class LookaheadPIDController(BaseController):
    """预瞄点横向偏差 PID 控制器（对照基线）。

    取固定前视距离处的路径点，计算它相对**车体坐标系**的横向偏差
    （车体左侧为正），再按 ``delta = kp*e + ki*∫e + kd*de/dt`` 输出转角。
    """

    name = "lookahead_pid"

    def __init__(self, kp: float = 0.35, ki: float = 0.02, kd: float = 0.10,
                 lookahead: float = 6.0, wheelbase: float = 2.7,
                 max_steering: float = math.radians(35.0),
                 steering_rate_limit: float = math.radians(120.0)) -> None:
        super().__init__(wheelbase=wheelbase, max_steering=max_steering,
                         steering_rate_limit=steering_rate_limit)
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.lookahead = float(lookahead)
        self._integral = 0.0
        self._previous_error = 0.0

    def reset(self) -> None:
        super().reset()
        self._integral = 0.0
        self._previous_error = 0.0

    def preview_error(self, state: np.ndarray, path: ReferencePath,
                      projection: ProjectionResult) -> float:
        """预瞄点相对车体的横向偏差：把预瞄点投影到车体左法向。

        ``e = (p_preview - p_vehicle) . n_left``，其中
        ``n_left = (-sin(yaw), cos(yaw))``；预瞄点在车体左侧为正。
        """
        preview = path.point_at_s(projection.s + self.lookahead)
        delta = preview - state[:2]
        left_normal = np.array([-math.sin(state[2]), math.cos(state[2])])
        return float(np.dot(delta, left_normal))

    def compute(self, state: np.ndarray, speed: float, path: ReferencePath,
                projection: ProjectionResult, dt: float) -> ControlCommand:
        error = self.preview_error(state, path, projection)
        derivative = 0.0 if dt <= 0.0 else (error - self._previous_error) / dt
        self._integral += error * dt
        # 抗积分饱和：积分项限幅，避免大偏差下积分累积导致超调
        self._integral = float(np.clip(self._integral, -5.0, 5.0))
        self._previous_error = error

        steering = self.kp * error + self.ki * self._integral + self.kd * derivative
        steering = self._saturate_steering(steering, dt)
        yaw_rate = 0.0 if abs(speed) < 1e-6 else speed * math.tan(steering) / self.wheelbase
        path_yaw = math.atan2(projection.tangent[1], projection.tangent[0])
        return ControlCommand(steering=steering, yaw_rate=yaw_rate, speed=speed,
                              cross_track_error=error,
                              heading_error=float(wrap_to_pi(path_yaw - state[2])))


def build_controller(name: str, **kwargs) -> BaseController:
    """按名称构造控制器，供命令行与测试使用。"""
    mapping = {
        "stanley": StanleyController,
        "pure_pursuit": AdaptivePurePursuitController,
        "lookahead_pid": LookaheadPIDController,
    }
    if name not in mapping:
        raise KeyError(f"未知控制器 '{name}'，可选: {', '.join(mapping)}")
    return mapping[name](**kwargs)
