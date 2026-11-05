# -*- coding: utf-8 -*-
"""距离分级决策状态机（纯函数，不导入 MuJoCo，可脱离仿真环境单测）。

决策只使用"射线测得的本车道最近距离"，不使用场景真值；
真实间距仅由 metrics.py 用于性能评价，两者互不影响。
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class State(str, Enum):
    """行驶状态。取值同时作为日志与 ROS 消息中的状态名。"""

    STALE = "STALE"            # 感知失效：超时没有有效测距 -> 兜底刹停
    CRUISE = "CRUISE"          # 安全距离 -> 按巡航速度前进
    SLOW_DOWN = "SLOW_DOWN"    # 进入减速区 -> 按距离线性降速
    FULL_BRAKE = "FULL_BRAKE"  # 进入急刹区 -> 目标速度归零


@dataclass(frozen=True)
class DecisionParams:
    """决策阈值。默认值与文档"参数表"一一对应。"""

    cruise_speed: float = 2.0        #: 巡航速度 (m/s)
    slow_start_m: float = 6.0        #: 开始减速的距离 (m)
    stop_target_m: float = 1.8       #: 期望停车位置：障碍物前该距离处速度归零
    emergency_m: float = 0.8         #: 急刹距离 (m)，小于该值直接给零速
    stale_timeout_s: float = 0.5     #: 有效测距超时 (s)
    creep_speed: float = 0.12        #: 蠕动阈值 (m/s)：目标速度低于该值即主动刹停

    def __post_init__(self):
        # 急刹区必须比"期望停车位置"更近：车正常会在 stop_target_m 处停住，
        # 只有异常（例如障碍物突然出现）才会进入 emergency_m 以内。
        if not self.emergency_m < self.stop_target_m < self.slow_start_m:
            raise ValueError(
                "阈值必须满足 emergency_m < stop_target_m < slow_start_m，"
                "当前为 emergency={} / stop_target={} / slow_start={}".format(
                    self.emergency_m, self.stop_target_m, self.slow_start_m
                )
            )
        if self.cruise_speed <= 0.0:
            raise ValueError("cruise_speed 必须为正数")
        if not 0.0 <= self.creep_speed < self.cruise_speed:
            raise ValueError("creep_speed 必须满足 0 <= creep_speed < cruise_speed")


@dataclass(frozen=True)
class Decision:
    """一次决策的输出。target_speed 为期望速度，最终速度还要经 controller 的加减速限幅。"""

    state: State
    target_speed: float
    distance: Optional[float]
    reason: str


def decide(min_distance_m: Optional[float],
           since_last_valid_s: float,
           params: DecisionParams = DecisionParams()) -> Decision:
    """由最近距离与感知新鲜度给出状态与目标速度。

    参数:
        min_distance_m:     本车道最近障碍距离 (m)；None 表示本帧没有有效测距
        since_last_valid_s: 距上一次有效测距的时间 (s)
        params:             阈值

    规则（自上而下，先命中先返回）:
        1. 感知失效（无数据或无命中且已超时） -> STALE，目标速度 0；
        2. d <= emergency_m                    -> FULL_BRAKE，目标速度 0；
        3. d <= slow_start_m                   -> SLOW_DOWN，目标速度按 d 线性插值；
           插值结果低于 creep_speed 时 -> FULL_BRAKE，主动刹停（避免长时间蠕动逼近）；
        4. 其余                                -> CRUISE，目标速度 = cruise_speed。
    """
    if min_distance_m is None:
        if since_last_valid_s >= params.stale_timeout_s:
            return Decision(State.STALE, 0.0, None,
                            "无有效测距 {:.2f}s（>= {:.2f}s），兜底刹停".format(
                                since_last_valid_s, params.stale_timeout_s))
        # 刚启动、还没来得及出第一帧：按停车前的保守速度处理
        return Decision(State.FULL_BRAKE, 0.0, None,
                        "尚未获得有效测距，保持静止")

    if min_distance_m <= params.emergency_m:
        return Decision(State.FULL_BRAKE, 0.0, min_distance_m,
                        "距离 {:.2f}m <= 急刹阈值 {:.2f}m".format(
                            min_distance_m, params.emergency_m))

    if min_distance_m <= params.slow_start_m:
        span = params.slow_start_m - params.stop_target_m
        ratio = (min_distance_m - params.stop_target_m) / span
        ratio = min(1.0, max(0.0, ratio))
        target = params.cruise_speed * ratio
        if target < params.creep_speed:
            # 线性剖面在 stop_target_m 附近趋于 0，若不截断会长时间低速蠕动；
            # 实际停车时低于蠕动速度就果断刹停。
            return Decision(State.FULL_BRAKE, 0.0, min_distance_m,
                            "目标速度 {:.3f}m/s 低于蠕动阈值 {:.3f}m/s，主动刹停".format(
                                target, params.creep_speed))
        return Decision(State.SLOW_DOWN, target, min_distance_m,
                        "距离 {:.2f}m 位于减速区 [{:.2f}, {:.2f}]m".format(
                            min_distance_m, params.stop_target_m, params.slow_start_m))

    return Decision(State.CRUISE, params.cruise_speed, min_distance_m,
                    "距离 {:.2f}m >= 减速起点 {:.2f}m".format(
                        min_distance_m, params.slow_start_m))
