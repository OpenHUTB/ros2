"""闭环实验：仿真世界 + 检测器 + 脱困监督器；可关闭脱困作为对照。

评估用仿真真值位姿计算"脱困后是否真的走起来了"，而模块内部只看里程计——
这正是本模块要解决的矛盾：**里程计会骗人**，所以评估口径必须与在线口径分开。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ros2_stuck_recovery.detect import CAUSE_NONE, DetectorConfig, Sample, StuckDetector
from ros2_stuck_recovery.recovery import (
    STATE_ABORT,
    STATE_CRUISE,
    Command,
    RecoveryConfig,
    RecoverySupervisor,
)
from ros2_stuck_recovery.robot import Limits, Twist, body_forward_delta, limit_accel

from .scenarios import SCENARIOS, get
from .stuck_world import Scenario, StuckWorld

DT = 0.05
POST_ESCAPE_TRAVEL = 0.30   # 脱困后至少还要真的走出这么远，才算"恢复任务"


def task_command(sc: Scenario, t: float) -> Twist:
    """任务层指令。free 场景里插一段原地转向，用来检查转弯时会不会误报。"""
    if sc.name == "free" and 4.0 <= t < 6.0:
        return Twist(0.0, 0.60)
    return sc.task


@dataclass
class Episode:
    scenario: str
    seed: int
    use_recovery: bool
    t: np.ndarray
    pose_true: np.ndarray
    pose_odom: np.ndarray
    cmd: np.ndarray
    applied: np.ndarray
    causes: list[str]
    states: list[str]
    slip_ratio: np.ndarray
    triggers: int
    escapes: int
    aborted: bool
    success: bool
    escape_time: float
    post_travel: float
    traveled: float
    stuck_ratio: float
    notes: list[str] = field(default_factory=list)

    @property
    def trigger_rate(self) -> float:
        return self.triggers / max(self.t[-1] - self.t[0], 1e-6)


def is_success(sc: Scenario, escapes: int, post_travel: float, triggers: int, aborted: bool) -> bool:
    """按场景期望判定：该脱困的要脱困，该停车的要停车，正常行驶的不能误报。"""
    if sc.expect == "no_trigger":
        return triggers == 0
    if sc.expect == "abort":
        return aborted
    return escapes >= 1 and post_travel >= POST_ESCAPE_TRAVEL and not aborted


def run_episode(
    name: str,
    seed: int = 0,
    use_recovery: bool = True,
    dt: float = DT,
    config: RecoveryConfig | None = None,
    detector_config: DetectorConfig | None = None,
    noise_scale: float = 1.0,
    sensors: tuple[str, ...] = ("imu", "range"),
) -> Episode:
    sc = get(name)
    world = StuckWorld(sc, seed=seed, noise_scale=noise_scale)
    detector = StuckDetector(detector_config)
    sup = RecoverySupervisor(config) if use_recovery else None
    limits = Limits()

    cmd = Twist()
    odom_prev = world.odom_pose
    true_prev = world.pose
    steps = int(round(sc.duration / dt))

    ts, poses, odoms, cmds, applied = [], [], [], [], []
    causes, states, ratios, notes = [], [], [], []
    traveled = 0.0
    post_travel = 0.0
    escape_time = -1.0
    stuck_steps = 0

    for _ in range(steps):
        obs = world.step(cmd, dt)
        v_body = body_forward_delta(odom_prev, obs.odom_pose) / dt
        odom_prev = obs.odom_pose

        sample = Sample(
            t=obs.t,
            v_cmd=cmd.v,
            w_cmd=cmd.w,
            v_odom=v_body,
            w_odom=obs.w_odom,
            w_imu=obs.w_imu if "imu" in sensors else None,
            range_front=obs.range_front if "range" in sensors else None,
        )
        verdict = detector.update(sample)

        task = task_command(sc, obs.t)
        if sup is None:
            out = Command(task.v, task.w, STATE_CRUISE, "对照：不脱困", False, 0)
        else:
            imu_gap = abs(obs.w_odom - obs.w_imu)
            out = sup.step(obs.t, task, verdict, 0.0, obs.odom_pose, imu_gap)
        cmd = limit_accel(cmd, Twist(out.v, out.w), dt, limits)

        moved = math.hypot(obs.pose[0] - true_prev[0], obs.pose[1] - true_prev[1])
        traveled += moved
        if sup is not None and sup.escapes >= 1 and escape_time < 0.0:
            escape_time = obs.t
        if escape_time >= 0.0:
            post_travel += moved
        true_prev = obs.pose

        if verdict.stuck:
            stuck_steps += 1
        ts.append(obs.t)
        poses.append(obs.pose)
        odoms.append(obs.odom_pose)
        cmds.append((cmd.v, cmd.w))
        applied.append((out.v, out.w))
        causes.append(verdict.cause)
        states.append(out.state)
        ratios.append(verdict.slip_ratio)
        if out.note and (not notes or notes[-1] != out.note):
            notes.append(out.note)

    triggers = sup.triggers if sup is not None else 0
    escapes = sup.escapes if sup is not None else 0
    aborted = bool(sup is not None and sup.state == STATE_ABORT)
    success = is_success(sc, escapes, post_travel, triggers, aborted)

    return Episode(
        scenario=name,
        seed=seed,
        use_recovery=use_recovery,
        t=np.asarray(ts),
        pose_true=np.asarray(poses),
        pose_odom=np.asarray(odoms),
        cmd=np.asarray(cmds),
        applied=np.asarray(applied),
        causes=list(causes),
        states=list(states),
        slip_ratio=np.asarray(ratios),
        triggers=triggers,
        escapes=escapes,
        aborted=aborted,
        success=success,
        escape_time=escape_time,
        post_travel=post_travel,
        traveled=traveled,
        stuck_ratio=stuck_steps / max(steps, 1),
        notes=notes,
    )


def summarise(ep: Episode) -> dict:
    first = next((i for i, c in enumerate(ep.causes) if c != CAUSE_NONE), None)
    return {
        "scenario": ep.scenario,
        "seed": ep.seed,
        "recovery": ep.use_recovery,
        "success": ep.success,
        "triggers": ep.triggers,
        "escapes": ep.escapes,
        "aborted": ep.aborted,
        "first_detect": float(ep.t[first]) if first is not None else float("nan"),
        "escape_time": ep.escape_time,
        "post_travel": ep.post_travel,
        "traveled": ep.traveled,
        "stuck_ratio": ep.stuck_ratio,
        "cause": next((c for c in ep.causes if c != CAUSE_NONE), "none"),
    }


def all_scenarios() -> list[str]:
    return list(SCENARIOS.keys())
