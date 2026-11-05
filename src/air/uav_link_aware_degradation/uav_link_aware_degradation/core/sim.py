#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环：遥控端 → 受损链路 → 机载监督器/策略 → 几何控制器 → 机体。

时间轴分三层：

* 物理/控制 100 Hz（``dt``）；
* 遥控端与遥测 20 Hz（``cmd_rate``）；
* 链路按包投递，带延迟/抖动/成片丢包/计划中断。

遥测和指令各走一条独立链路，接收端只知道“到达时间 + 发送时间戳”，
因此 ``cmd_age = now - send_t`` 已经把单向延迟算进去了。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np

from .link import LinkEmulator, LinkMonitor, LinkProfile
from .mission import Mission
from .policies import RemotePilot, follow, make_policy
from .supervisor import Mode, SupervisorParams


# --------------------------------------------------------------------------- #
# 几何净空
# --------------------------------------------------------------------------- #
DRONE_RADIUS = 0.16


def _object_distance(p: np.ndarray, obj) -> float:
    """点到单个场景物体的表面距离（近似 AABB / 圆柱 / 球）。"""
    c = np.asarray(obj.pos, dtype=float)
    s = obj.size if isinstance(obj.size, (tuple, list)) else (obj.size,)
    if obj.kind == "box":
        half = np.asarray(s[:3], dtype=float)
        d = np.abs(p - c) - half
        outside = float(np.linalg.norm(np.maximum(d, 0.0)))
        if outside > 0:
            return outside
        return -float(np.min(np.maximum(half - np.abs(p - c), -1e9)))
    if obj.kind == "cylinder":
        r, hh = float(s[0]), float(s[1])
        dxy = max(0.0, float(np.linalg.norm((p - c)[:2])) - r)
        dz = abs(float(p[2] - c[2])) - hh
        if dz <= 0:
            return dxy if dxy > 0 else -min(r - float(np.linalg.norm((p - c)[:2])),
                                            hh - abs(float(p[2] - c[2])))
        return float(np.hypot(dxy, dz))
    if obj.kind == "sphere":
        return float(np.linalg.norm(p - c) - float(s[0]))
    return float("inf")


def clearance(pos: np.ndarray, scene, drone_radius: float = DRONE_RADIUS) -> float:
    """到最近障碍（含地面）的表面净空，已扣除机体半径。"""
    best = float(pos[2])
    for obj in getattr(scene, "objects", []):
        best = min(best, _object_distance(np.asarray(pos, float), obj))
    return float(best - drone_radius)


# --------------------------------------------------------------------------- #
# 结果容器
# --------------------------------------------------------------------------- #
@dataclass
class RunResult:
    profile: str
    policy: str
    t: List[float] = field(default_factory=list)
    pos: List[np.ndarray] = field(default_factory=list)
    mode: List[str] = field(default_factory=list)
    age: List[float] = field(default_factory=list)
    pdr: List[float] = field(default_factory=list)
    clearance: List[float] = field(default_factory=list)
    speed: List[float] = field(default_factory=list)

    completed: bool = False
    returned: bool = False
    contact: bool = False
    min_clearance: float = float("inf")
    max_age: float = 0.0
    max_speed: float = 0.0
    switches: int = 0
    time_in_mode: Dict[str, float] = field(default_factory=dict)
    final_home_error: float = float("inf")
    max_path_deviation: float = 0.0
    duration: float = 0.0
    progress: float = 0.0

    def to_dict(self) -> Dict[str, object]:
        return {
            "profile": self.profile,
            "policy": self.policy,
            "completed": int(self.completed),
            "returned": int(self.returned),
            "contact": int(self.contact),
            "min_clearance_m": round(self.min_clearance, 3),
            "max_cmd_age_s": round(self.max_age, 3),
            "max_speed_mps": round(self.max_speed, 3),
            "switches": int(self.switches),
            "final_home_error_m": (round(self.final_home_error, 3)
                                   if np.isfinite(self.final_home_error) else None),
            "max_path_deviation_m": round(self.max_path_deviation, 3),
            "duration_s": round(self.duration, 2),
            "progress": round(self.progress, 3),
            "t_normal": round(self.time_in_mode.get("NORMAL", 0.0), 2),
            "t_degraded": round(self.time_in_mode.get("DEGRADED", 0.0), 2),
            "t_return": round(self.time_in_mode.get("RETURN", 0.0), 2),
            "t_land": round(self.time_in_mode.get("LAND", 0.0), 2),
        }


def _path_deviation(pos: np.ndarray, path: np.ndarray) -> float:
    """点到名义航迹折线的水平距离。"""
    p = pos[:2]
    best = float("inf")
    for a, b in zip(path[:-1], path[1:]):
        ab = b[:2] - a[:2]
        n2 = float(ab @ ab)
        if n2 < 1e-9:
            best = min(best, float(np.linalg.norm(p - a[:2])))
            continue
        u = float(np.clip((p - a[:2]) @ ab / n2, 0.0, 1.0))
        best = min(best, float(np.linalg.norm(p - (a[:2] + u * ab))))
    return best


# --------------------------------------------------------------------------- #
# 闭环
# --------------------------------------------------------------------------- #
def run_case(profile: LinkProfile, policy_name: str, mission: Mission,
             quad, controller, scene, dt: float = 0.01, cmd_rate: float = 20.0,
             duration: float = 45.0, seed: int = 0,
             params: SupervisorParams | None = None,
             record_every: int = 5) -> RunResult:
    """跑一次完整任务，返回轨迹与指标。"""
    p = params or SupervisorParams()
    policy = make_policy(policy_name, p)
    pilot = RemotePilot(mission, cruise_speed=p.nominal_speed)
    up = LinkEmulator(profile, seed=seed)
    down = LinkEmulator(profile, seed=seed + 101)
    up_mon = LinkMonitor(cmd_rate, window=3.0)
    down_mon = LinkMonitor(cmd_rate, window=3.0)

    quad.reset(tuple(mission.home), yaw=0.0)
    controller.yaw_hold = 0.0
    p_d = np.asarray(mission.home, dtype=float).copy()
    v_d = np.zeros(3)

    result = RunResult(profile=profile.name, policy=policy_name)
    last_cmd: np.ndarray | None = None
    belief = np.asarray(mission.home, dtype=float).copy()
    up_queue: List[Tuple[float, np.ndarray, float]] = []
    down_queue: List[Tuple[float, np.ndarray, float]] = []

    cmd_dt = 1.0 / cmd_rate
    next_cmd = 0.0
    steps = int(duration / dt)
    mode_counts: Dict[str, float] = {}

    for k in range(steps):
        now = k * dt

        # ---- 遥控端 + 下行遥测 ------------------------------------------
        if now + 1e-9 >= next_cmd:
            st = quad.state()
            d = down.transmit(now)
            if d.delivered:
                down_queue.append((d.arrival, st.pos.copy(), now))
            v_cmd = pilot.command(belief)
            d = up.transmit(now)
            if d.delivered:
                up_queue.append((d.arrival, v_cmd, now))
            next_cmd += cmd_dt

        # ---- 到包处理 ---------------------------------------------------
        while down_queue and down_queue[0][0] <= now:
            arrival, pos_belief, send_t = down_queue.pop(0)
            belief = pos_belief
            down_mon.push(send_t, arrival)
        while up_queue and up_queue[0][0] <= now:
            arrival, v_cmd, send_t = up_queue.pop(0)
            last_cmd = v_cmd
            up_mon.push(send_t, arrival)

        status = up_mon.update(now)
        mission.advance(quad.state().pos)

        # ---- 决策 -------------------------------------------------------
        decision = policy.decide(now, status, quad.state().pos, mission)
        mode_label = decision.mode.value if hasattr(decision.mode, "value") else str(decision.mode)
        if decision.source == "remote":
            v_cmd = last_cmd if last_cmd is not None else np.zeros(3)
        elif decision.source == "zero":
            v_cmd = np.zeros(3)
        else:
            # 返航点下降阶段必须允许高度低于安全带下限
            alt_min = 0.0 if decision.mode == Mode.LAND else p.alt_min
            v_cmd = follow(quad.state().pos, decision.target, decision.speed_cap,
                           alt_min=alt_min, alt_max=p.alt_max)

        # ---- 位置设定点积分 ---------------------------------------------
        p_d = p_d + v_d * dt
        p_d[2] = float(np.clip(p_d[2], 0.30, 2.8))
        v_d = np.asarray(v_cmd, dtype=float)

        horiz = v_d[:2]
        if np.linalg.norm(horiz) > 0.15:
            yaw_target = float(np.arctan2(horiz[1], horiz[0]))
            # 偏航设定点限速 1.2 rad/s：返航瞬间 180° 反向时直接跳到 -x 会让机体翻滚
            dyaw = float(np.arctan2(np.sin(yaw_target - controller.yaw_hold),
                                    np.cos(yaw_target - controller.yaw_hold)))
            step = 1.2 * dt
            controller.yaw_hold += float(np.clip(dyaw, -step, step))
        thrusts = controller.act(quad, p_d, v_d, np.zeros(3), controller.yaw_hold)
        quad.step_with_control(thrusts, n=1)

        # ---- 记录 -------------------------------------------------------
        if k % record_every == 0:
            st = quad.state()
            clr = clearance(st.pos, scene)
            result.t.append(now)
            result.pos.append(st.pos.copy())
            result.mode.append(mode_label)
            result.age.append(float(status.age))
            result.pdr.append(float(status.pdr))
            result.clearance.append(clr)
            result.speed.append(float(np.linalg.norm(st.vel)))
            mode_counts[mode_label] = mode_counts.get(mode_label, 0.0) + record_every * dt
            result.min_clearance = min(result.min_clearance, clr)
            if clr < 0.10:
                result.contact = True
            result.max_age = max(result.max_age, float(status.age))
            result.max_speed = max(result.max_speed, float(np.linalg.norm(st.vel)))
            result.max_path_deviation = max(
                result.max_path_deviation, _path_deviation(st.pos, mission.nominal_path()))

        # ---- 终止条件 ---------------------------------------------------
        st = quad.state()
        if result.contact:
            break
        if decision.mode == Mode.LAND and st.pos[2] <= p.land_alt + 0.04:
            break
        if mission.returned and decision.mode not in (Mode.RETURN, Mode.LAND):
            break

    st = quad.state()
    result.duration = float(quad.time)
    result.completed = bool(mission.finished)
    result.returned = bool(mission.home_reached)
    result.progress = mission.progress()
    result.final_home_error = float(np.linalg.norm(st.pos - mission.home))
    result.time_in_mode = mode_counts
    result.switches = int(getattr(policy, "switches", 0))
    return result
