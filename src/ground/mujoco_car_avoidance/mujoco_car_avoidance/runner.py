# -*- coding: utf-8 -*-
"""控制回路：感知 -> 决策 -> 速度整形 -> 差速 -> 写回 MuJoCo -> 采集指标。

把回路独立成类，使 --demo / --interactive / --headless / ROS 2 节点四种入口
复用同一套逻辑，保证"测过的就是跑着的"。

手动模式（--interactive 与 ROS 2 cmd_vel）与自主避障共用同一条限速链路：
手动请求的速度最终会被 decision 给出的 target_speed 封顶，
因此"手动开车也撞不到障碍物"，不需要在 teleop 里再写一套保护。
"""

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from . import config
from .controller import RampParams, SpeedRamp, differential_drive
from .decision import Decision, DecisionParams, State, decide
from .metrics import MetricsRecorder


@dataclass
class StepInfo:
    """一帧的完整信息，供终端日志与 ROS 话题发布复用。"""

    t: float
    state: State
    reason: str
    d_sensor: Optional[float]
    ranges: object
    v_target: float
    v_cmd: float
    v_actual: float
    angular: float
    wheel_left: float
    wheel_right: float
    saturated: bool
    x: float
    y: float
    yaw: float
    clearance: float
    collision: bool
    held: bool = False          #: 本帧是否沿用了上一次有效测距（传感器单帧丢失）


class AvoidanceRunner:
    """把 CarEnv 与决策/控制/指标组装成闭环。"""

    def __init__(self, env,
                 params: DecisionParams = DecisionParams(),
                 ramp_params: RampParams = RampParams(),
                 control_hz: float = config.CONTROL_HZ,
                 manual_angular_limit: float = 1.0,
                 settle_seconds: float = config.SETTLE_SECONDS,
                 heading_hold_gain: float = config.HEADING_HOLD_GAIN,
                 heading_hold_limit: float = config.HEADING_HOLD_LIMIT):
        self.env = env
        self.params = params
        self.ramp_params = ramp_params
        self.ramp = SpeedRamp(ramp_params)
        self.control_hz = float(control_hz)
        self.dt = 1.0 / float(control_hz)
        self.manual_angular_limit = float(manual_angular_limit)
        self.settle_seconds = float(settle_seconds)
        self.heading_hold_gain = float(heading_hold_gain)
        self.heading_hold_limit = float(heading_hold_limit)

        self.recorder = MetricsRecorder()
        self._since_valid = float("inf")   # 启动时视为"还没有有效测距"
        self._last_valid_distance = None   # 上一次有效测距，用于单帧丢失时保持
        self._yaw_reference = 0.0          # 直行保持的目标航向
        self.realtime_factor = 0.0

    # ------------------------------------------------------------------ 复位
    def reset(self) -> None:
        self.env.reset()
        self.ramp.reset(0.0)
        self.recorder = MetricsRecorder()
        self._since_valid = float("inf")
        self._last_valid_distance = None
        self._yaw_reference = 0.0

    def settle(self, seconds=None) -> None:
        """起步前静置：零轮速推进一段时间，等接触建立、车身稳定后再进入控制闭环。

        为什么必需：车辆出生时轮胎与地面恰好相切（穿透量 0），MuJoCo 会判定为"未接触"。
        实测 ncon 在 0/3/6 之间跳动、车轮空转（滑移率 55%+），左右摩擦不对称还会让车偏航约 10°，
        最终把路侧干扰物误判为本车道障碍物而提前刹停。
        """
        if seconds is None:
            seconds = self.settle_seconds
        seconds = max(0.0, float(seconds))

        self.ramp.reset(0.0)
        self.env.set_wheel_speeds(0.0, 0.0)
        if seconds > 0.0:
            steps = max(1, int(round(seconds / self.env.timestep)))
            self.env.step(steps)

        # 静置结束后重新开始计"有效测距"，并以当前航向作为直行基准
        self._since_valid = float("inf")
        self._last_valid_distance = None
        self._yaw_reference = self.env.state().yaw if seconds > 0.0 else 0.0

    # ------------------------------------------------------------------ 内部
    def _shape_speed_command(self, decision: Decision, manual_cmd, dt: float):
        """速度整形：返回 (v_cmd, angular)。

        自主模式：目标速度 = 决策输出，经加减速限幅；
                  急刹 / 感知失效不过斜坡，直接给零速（物理减速度由轮胎摩擦决定）。
        手动模式：请求速度先被决策的 target_speed 封顶，再经同一斜坡；
                  因此减速区会压速、急刹区会强制停车。
        """
        emergency = decision.state in (State.FULL_BRAKE, State.STALE)

        if manual_cmd is None:
            if emergency:
                self.ramp.reset(0.0)
                return 0.0, 0.0
            return self.ramp.update(decision.target_speed, dt), 0.0

        v_request = float(manual_cmd[0])
        angular = max(-self.manual_angular_limit,
                      min(self.manual_angular_limit, float(manual_cmd[1])))

        if emergency:
            v_request = 0.0
        elif v_request > 0.0:
            # 前进方向受避障限速封顶；倒车（v_request < 0）不叠加，仅用于演示
            v_request = min(v_request, decision.target_speed)

        return self.ramp.update(v_request, dt), angular

    def _heading_correction(self, yaw: float) -> float:
        """直线行驶航向保持：比例控制，输出角速度修正。

        自主模式下启用：车辆若有左右牵引不对称就会慢慢偏航，
        偏航会让"本车道"跟着旋转，把路侧目标错误地纳入避障决策
        （实测偏航 10° 即触发；偏航符号写反时更会变成正反馈，实测被推到 29°）。

        符号约定：differential_drive 中正角速度 = 左转（左轮慢、右轮快），
        因此 yaw 大于参考航向（车头左偏）时必须输出**负**角速度才能校正。
        """
        if self.heading_hold_gain <= 0.0:
            return 0.0
        error = math.atan2(math.sin(self._yaw_reference - yaw),
                           math.cos(self._yaw_reference - yaw))
        correction = self.heading_hold_gain * error      # 偏差本身已含方向
        return max(-self.heading_hold_limit, min(self.heading_hold_limit, correction))

    # ------------------------------------------------------------------ 单步
    def step_once(self, manual_cmd: Optional[Tuple[float, float]] = None) -> StepInfo:
        """推进一个控制周期，返回本帧信息。

        manual_cmd: (线速度 m/s, 角速度 rad/s)；None 表示完全自主避障。
        """
        dt = self.dt

        # ---------- 1. 感知：只用射线，不用任何场景真值 ----------
        ranges = self.env.scan()
        distance = self.env.lidar.min_distance(ranges)
        held = False
        if distance is None:
            self._since_valid += dt
            # 单帧丢失不应立刻急刹：在超时窗口内沿用上一次有效测距。
            # 实测教训：丢失即急刹会把车轮瞬间拍死，反作用力矩让车头俯仰，
            # 俯仰又让测距更不可靠，形成"丢测距 -> 急刹 -> 抬头 -> 丢更多"的恶性循环。
            if (self._last_valid_distance is not None
                    and self._since_valid < self.params.stale_timeout_s):
                distance = self._last_valid_distance
                held = True
        else:
            self._since_valid = 0.0
            self._last_valid_distance = distance

        # ---------- 2. 决策 ----------
        decision = decide(distance, self._since_valid, self.params)

        # ---------- 3. 速度整形 + 差速运动学 ----------
        v_cmd, angular = self._shape_speed_command(decision, manual_cmd, dt)
        if manual_cmd is None:
            # 自主模式下的直行保持；手动模式不干预驾驶者转向
            angular += self._heading_correction(self.env.state().yaw)
        left, right, saturated = differential_drive(
            v_cmd, angular,
            config.WHEEL_RADIUS_M, config.WHEEL_SEPARATION_M, config.MAX_WHEEL_OMEGA)

        # ---------- 4. 写回 MuJoCo 并推进一个控制周期 ----------
        self.env.set_wheel_speeds(left, right)
        self.env.advance_control_period()

        # ---------- 5. 采集本帧结果 ----------
        state = self.env.state()
        clearance = self.env.truth_clearance()
        collision = self.env.has_obstacle_contact()
        if collision:
            self.recorder.collision = True

        self.recorder.record(state.t, decision.state.value, distance, v_cmd,
                             state.v, state.x, clearance)

        return StepInfo(
            t=state.t, state=decision.state, reason=decision.reason,
            d_sensor=distance, ranges=ranges,
            v_target=decision.target_speed, v_cmd=v_cmd,
            v_actual=state.v, angular=angular,
            wheel_left=state.wheel_left, wheel_right=state.wheel_right,
            saturated=saturated, x=state.x, y=state.y, yaw=state.yaw,
            clearance=clearance, collision=collision, held=held,
        )

    # ------------------------------------------------------------------ 整段
    def run(self, duration: float,
            on_frame: Optional[Callable[[StepInfo], bool]] = None,
            manual_source: Optional[Callable[[], Optional[Tuple[float, float]]]] = None,
            settle_seconds: float = 2.0) -> MetricsRecorder:
        """连续运行 duration 秒（仿真时间）。

        on_frame 返回 False 时提前结束（交互模式里用户关闭窗口）。
        车辆平稳停住超过 settle_seconds 秒后提前结束，避免空跑。
        """
        wall_start = time.perf_counter()
        sim_start = self.env.time
        settled_for = 0.0

        while (self.env.time - sim_start) < duration:
            manual = manual_source() if manual_source is not None else None
            info = self.step_once(manual)

            if on_frame is not None and on_frame(info) is False:
                break

            if abs(info.v_actual) < 0.02 and info.state in (State.SLOW_DOWN, State.FULL_BRAKE):
                settled_for += self.dt
                if settled_for >= settle_seconds:
                    break
            else:
                settled_for = 0.0

        wall_elapsed = max(time.perf_counter() - wall_start, 1e-9)
        sim_elapsed = self.env.time - sim_start
        self.realtime_factor = sim_elapsed / wall_elapsed
        return self.recorder
