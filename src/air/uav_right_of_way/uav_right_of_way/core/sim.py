#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环仿真：基线（无规则直飞）与规则启用两两对比，MuJoCo 真飞。

每架无人机用课程底座 ``uav_mujoco_sim``（同一份 ``quadrotor.xml`` 刚体模型
+ 几何控制器）单独实例化。规则启用时，每架机用**刚体积分出来的真实状态**
做侵入判定（``detect``），命中规则（``rules``）后按动作调整参考速度：

* 右转 30°   → 参考航向叠加 −30° 偏置；
* 降高 10 m  → 参考高度下移 10 m；
* 减速 30%   → 参考速度乘 (1−0.30)；
* 左侧追越  → 对参考点施加目标横向偏置（沿航向左侧 overtake_lateral_m）。

威胁消失并保持 ``clear_hold_s`` 后进入恢复（``recover.RecoveryProfile``，
余弦剖面回到计划航路），恢复期间每个决策周期重新做威胁检测，出现二次
冲突立即中止恢复、重新决策。

参考速度前馈经一阶低通（τ=0.5 s）、姿态阻尼 kw≈−0.18——与参考实现
uav_deconflict ``core/sim.py`` 相同的写法，防止航向阶跃激励姿态振荡。
``--no-sim``（或 MuJoCo 不可用）时退化为纯运动学参考点积分，算法层
（判定/规则/恢复）走完全相同的代码路径。
"""
from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .detect import AircraftState, DetectParams, Threat, predict_threat
from .recover import RecoveryProfile, should_abort
from .rules import (ACTION_DESCEND, ACTION_MAINTAIN, ACTION_OVERTAKE_LEFT,
                    ACTION_SLOW, ACTION_TURN_RIGHT, Decision, RuleParams,
                    decide)
from .scenarios import FlightPlan

#: 参考速度前馈的一阶低通时间常数（s）
REF_TAU = 0.5
#: 姿态角速度阻尼增益（同参考实现 sim.py，抑制机动激励的振荡）
ATT_DAMPING = (-0.18, -0.18, -0.126)

#: 默认决策周期（s，仿真时间）
DECIDE_DT = 0.2
#: 默认威胁消失后保持观察时长（s）
CLEAR_HOLD_S = 3.0
#: 默认恢复最短时长（s）
RECOVERY_MIN_S = 8.0
#: 横向偏置跟踪增益与限幅
LAT_GAIN = 0.8
LAT_VMAX = 4.0
#: 高度环增益与限幅
ALT_GAIN = 0.8
ALT_VMAX = 3.0


def _load_backend():
    """导入课程仿真底座；源码目录直接运行时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()
    except Exception:
        here = Path(__file__).resolve()
        # src/air/uav_right_of_way/uav_right_of_way/core/sim.py → src/air
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()


# --------------------------------------------------------------------------- #
# 运行结果与指标
# --------------------------------------------------------------------------- #
@dataclass
class RunResult:
    """一次闭环运行的记录与指标。"""

    mode: str                                   # "baseline" | "rules"
    times: np.ndarray                           # (K,) 记录时刻
    pos: Dict[str, np.ndarray]                  # uid -> (K,3) 真实位置
    plans: Dict[str, FlightPlan]
    decisions: List[Decision] = field(default_factory=list)
    wall_time: float = 0.0
    steps: int = 0

    def min_separation(self) -> Tuple[float, Tuple[str, str], float]:
        """真飞全程机对间最小三维距离及机对、时刻。"""
        best = math.inf
        pair: Tuple[str, str] = ("", "")
        t_best = 0.0
        names = sorted(self.pos)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = np.linalg.norm(self.pos[a] - self.pos[b], axis=1)
                k = int(np.argmin(d))
                if d[k] < best:
                    best = float(d[k])
                    pair = (a, b)
                    t_best = float(self.times[k])
        return best, pair, t_best

    def deviation_stats(self) -> Dict[str, float]:
        """逐机平均航迹偏离：真实位置到同时刻计划位置的平均距离（m）。"""
        out: Dict[str, float] = {}
        for uid, plan in self.plans.items():
            ref = np.array([plan.position_at(t) for t in self.times])
            err = np.linalg.norm(self.pos[uid] - ref, axis=1)
            out[uid] = float(err.mean())
        return out

    def mean_deviation(self) -> float:
        stats = self.deviation_stats()
        return float(np.mean(list(stats.values()))) if stats else 0.0

    def summary(self, params: DetectParams) -> dict:
        best, pair, t_best = self.min_separation()
        names = sorted(self.pos)
        close_samples = 0
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = np.linalg.norm(self.pos[a] - self.pos[b], axis=1)
                close_samples += int(np.sum(d < params.cpa_threshold_m))
        return dict(
            mode=self.mode,
            min_separation_m=round(best, 2),
            min_pair=list(pair),
            min_sep_time_s=round(t_best, 1),
            decisions=len(self.decisions),
            mean_deviation_m=round(self.mean_deviation(), 2),
            per_agent_deviation_m={k: round(v, 2)
                                   for k, v in self.deviation_stats().items()},
            below_threshold_samples=close_samples,
        )


# --------------------------------------------------------------------------- #
# 机体包装：纯运动学（--no-sim 退化模式）
# --------------------------------------------------------------------------- #
class _KinematicBody:
    """直接跟随参考点，接口与 ``Quadrotor`` 对齐（qpos / state / step）。"""

    def __init__(self, p0: Sequence[float]):
        self._pos = np.asarray(p0, dtype=float).copy()
        self.vel = np.zeros(3)

    @property
    def qpos(self) -> np.ndarray:
        return self._pos

    def state(self):
        """与 ``QuadrotorState`` 最小兼容：只用到 .vel。"""
        return self

    def set_state(self, pos: np.ndarray, vel: np.ndarray) -> None:
        self._pos = np.asarray(pos, dtype=float).copy()
        self.vel = np.asarray(vel, dtype=float).copy()


class _Agent:
    """一架无人机的闭环状态机：计划跟随 / 让路机动 / 余弦恢复。"""

    def __init__(self, plan: FlightPlan, body, ctrl, enable_rules: bool,
                 detect_params: DetectParams, rule_params: RuleParams):
        self.plan = plan
        self.body = body
        self.ctrl = ctrl
        self.enable_rules = enable_rules
        self.dp = detect_params
        self.rp = rule_params

        self.v_filt = np.zeros(3)
        self.decisions: List[Decision] = []

        # 巡航参考运动学
        self.ref_xy = plan.cruise_start[:2].copy()
        self.ref_z = plan.alt
        self._cruise_init = False
        # 机动状态
        self.yaw_off = 0.0            # rad，右转为负
        self.alt_off = 0.0            # m
        self.speed_factor = 1.0
        self.lat_target: Optional[float] = None   # m，沿航向左为正
        # 恢复
        self.rec: Optional[RecoveryProfile] = None
        self.rec_t0 = 0.0
        # 决策 episode：other -> 最近一次确认威胁的时刻
        self.episodes: Dict[str, float] = {}

    # ------------------------------------------------------------------ 状态
    def true_pos(self) -> np.ndarray:
        return np.asarray(self.body.qpos[:3], dtype=float)

    def true_vel(self) -> np.ndarray:
        if isinstance(self.body, _KinematicBody):
            return np.asarray(self.body.vel, dtype=float)
        return np.asarray(self.body.state().vel, dtype=float)

    def own_state(self) -> AircraftState:
        return AircraftState(self.plan.uid, self.true_pos(), self.true_vel(),
                             self.plan.priority, self.plan.label)

    # ------------------------------------------------------------------ 动作
    def apply_action(self, dec: Decision) -> None:
        kind = dec.action.kind
        if kind == ACTION_TURN_RIGHT:
            self.yaw_off = -math.radians(dec.action.value)
            self.lat_target = None
        elif kind == ACTION_DESCEND:
            self.alt_off = -dec.action.value
            self.lat_target = None
        elif kind == ACTION_SLOW:
            self.speed_factor = 1.0 - dec.action.value
            self.lat_target = None
        elif kind == ACTION_OVERTAKE_LEFT:
            self.lat_target = dec.action.value
        elif kind == ACTION_MAINTAIN:
            pass

    def begin_recovery(self, t: float) -> None:
        x_off = self.plan.cross_track(self.ref_xy)
        remain = self.plan.cruise_t1 - t
        dur = max(RECOVERY_MIN_S, 2.5 * abs(x_off) + 4.0)
        dur = min(dur, max(4.0, 0.8 * remain))
        self.rec = RecoveryProfile(lateral0=x_off, alt0=self.alt_off,
                                   speed_factor0=self.speed_factor,
                                   duration=dur)
        self.rec_t0 = t
        self.yaw_off = 0.0
        self.lat_target = None
        self.episodes.clear()

    # ------------------------------------------------------------------ 决策
    def tick_decisions(self, t: float, others: Sequence["_Agent"],
                       clear_hold_s: float) -> None:
        """一个决策周期：威胁检测 → 规则 → 恢复触发（含恢复期监视）。"""
        if not self.plan.in_cruise(t):
            return
        others_active = [a for a in others
                         if a is not self and a.plan.in_cruise(t)]
        own = self.own_state()
        other_states = [a.own_state() for a in others_active]
        by_uid = {s.uid: s for s in other_states}

        threats: Dict[str, Threat] = {}
        for st in other_states:
            th = predict_threat(own, st, self.dp)
            if th is not None:
                threats[st.uid] = th

        # ---- 恢复中：持续监视，出现二次冲突立即中止并重新决策 ------------
        if self.rec is not None:
            th = should_abort(own, other_states, self.dp)
            if th is not None:
                self.rec = None
                self.episodes.clear()
                self._new_decision(t, own, by_uid.get(th.other), th)
            elif t - self.rec_t0 >= self.rec.duration:
                self.rec = None
                self.alt_off = 0.0
                self.speed_factor = 1.0
            return

        # ---- 无威胁：episode 全部超过观察期 → 进入余弦恢复 ----------------
        if not threats:
            if self.episodes and \
                    all(t - t0 >= clear_hold_s for t0 in self.episodes.values()):
                self.begin_recovery(t)
            return

        # ---- 有威胁：为尚无 episode 的机对产出决策 ------------------------
        for other_uid in sorted(threats, key=lambda u: threats[u].cpa_t):
            if other_uid in self.episodes:
                self.episodes[other_uid] = t    # 刷新"仍有威胁"时刻
                continue
            self._new_decision(t, own, by_uid.get(other_uid), threats[other_uid])

    def _new_decision(self, t: float, own: AircraftState, other_state,
                      threat: Threat) -> None:
        if other_state is None:
            return
        dec = decide(own, other_state, threat, t, self.rp)
        self.decisions.append(dec)
        self.episodes[other_state.uid] = t
        self.apply_action(dec)

    # ------------------------------------------------------------------ 参考
    def reference(self, t: float, dt: float) -> Tuple[np.ndarray, np.ndarray,
                                                      float]:
        """当前控制周期的 (参考位置, 参考速度, 参考偏航)。"""
        plan = self.plan
        if t < plan.cruise_t0 or t > plan.cruise_t1:
            # 爬升 / 下降段：原样跟随计划
            self._cruise_init = False
            if t <= plan.cruise_t0:
                self.ref_xy = plan.cruise_start[:2].copy()
                self.ref_z = plan.cruise_start[2]
            return plan.position_at(t), plan.velocity_at(t), 0.0

        if not self._cruise_init:
            self._cruise_init = True
            self.ref_xy = plan.cruise_start[:2].copy()
            self.ref_z = plan.alt

        h = plan.cruise_heading
        h_eff = h + self.yaw_off
        u = np.array([math.cos(h_eff), math.sin(h_eff)])
        n = np.array([-math.sin(h), math.cos(h)])   # 计划航向的左法向

        speed_f = self.speed_factor
        lat_cmd = 0.0
        z_off = self.alt_off
        if self.rec is not None:
            s = self.rec.s(t - self.rec_t0)
            speed_f = self.rec.speed_factor(s)
            x_off = plan.cross_track(self.ref_xy)
            # 余弦剖面：剖面导数前馈 + 对参考点的小反馈
            lat_cmd = self.rec.lateral_rate(s) \
                + 0.5 * (self.rec.lateral(s) - x_off)
            z_off = self.rec.alt(s)
        elif self.lat_target is not None:
            x_off = plan.cross_track(self.ref_xy)
            lat_cmd = (self.lat_target - x_off) * LAT_GAIN

        lat_cmd = float(np.clip(lat_cmd, -LAT_VMAX, LAT_VMAX))
        v_h = plan.speed * speed_f * u + lat_cmd * n
        vz = float(np.clip((plan.alt + z_off - self.ref_z) * ALT_GAIN,
                           -ALT_VMAX, ALT_VMAX))

        self.ref_xy = self.ref_xy + v_h * dt
        self.ref_z = self.ref_z + vz * dt
        v_ref = np.array([v_h[0], v_h[1], vz])
        p_ref = np.array([self.ref_xy[0], self.ref_xy[1],
                          max(self.ref_z, 0.05)])
        return p_ref, v_ref, h


# --------------------------------------------------------------------------- #
# 闭环主循环
# --------------------------------------------------------------------------- #
def run_fleet(plans: Sequence[FlightPlan], enable_rules: bool = True,
              dt: float = 0.01, substeps: int = 4, record_every: int = 50,
              tail: float = 2.0, max_steps: int = 200000, progress: bool = False,
              use_mujoco: bool = True, ref_tau: float = REF_TAU,
              detect_params: Optional[DetectParams] = None,
              rule_params: Optional[RuleParams] = None,
              decide_dt: float = DECIDE_DT,
              clear_hold_s: float = CLEAR_HOLD_S,
              recovery_min_s: float = RECOVERY_MIN_S) -> RunResult:
    """让机队按计划真飞：``enable_rules=False`` 为基线（无规则直飞）。"""
    dp = detect_params or DetectParams()
    rp = rule_params or RuleParams()
    use_mj = use_mujoco
    backend = None
    if use_mj:
        try:
            backend = _load_backend()
        except Exception:
            use_mj = False

    plans = list(plans)
    agents: List[_Agent] = []
    if use_mj and backend is not None:
        Quadrotor, GeometricController, model_dir = backend
        model_path = os.path.join(str(model_dir), "quadrotor.xml")
        for plan in plans:
            quad = Quadrotor(model_path)
            quad.reset((plan.start[0], plan.start[1], 0.05))
            ctrl = GeometricController(quad.params, kw=ATT_DAMPING)
            agents.append(_Agent(plan, quad, ctrl, enable_rules, dp, rp))
    else:
        for plan in plans:
            agents.append(_Agent(plan, _KinematicBody(
                (plan.start[0], plan.start[1], 0.05)), None, enable_rules,
                dp, rp))

    t_end = max(p.end_time for p in plans) + tail
    n_steps = min(max_steps, int(round(t_end / dt)))
    times: List[float] = []
    pos: Dict[str, List[np.ndarray]] = {p.uid: [] for p in plans}
    last_decide: Dict[str, float] = {p.uid: -math.inf for p in plans}
    alpha = min(1.0, dt / ref_tau) if ref_tau and ref_tau > 0 else 1.0

    t0_wall = time.time()
    for step in range(n_steps):
        t = step * dt
        for ag in agents:
            # 决策周期（规则启用时）
            if enable_rules and t - last_decide[ag.plan.uid] >= decide_dt - 1e-9:
                ag.tick_decisions(t, agents, clear_hold_s)
                last_decide[ag.plan.uid] = t

            p_ref, v_ref, hdg = ag.reference(t, dt)
            if alpha < 1.0:
                ag.v_filt += (v_ref - ag.v_filt) * alpha
                v_use = ag.v_filt
            else:
                v_use = v_ref

            if use_mj and backend is not None:
                thrusts = ag.ctrl.act(ag.body, p_ref, v_use, np.zeros(3), hdg)
                ag.body.step_with_control(thrusts, substeps)
            else:
                # 运动学退化：真实状态 = 参考点
                ag.body.set_state(p_ref, v_use)

        if step % record_every == 0:
            times.append(t)
            for ag in agents:
                pos[ag.plan.uid].append(ag.true_pos().copy())

        if progress and step % 5000 == 0:
            print("  sim t=%.1f / %.1f s" % (t, t_end))

    times.append(n_steps * dt)
    for ag in agents:
        pos[ag.plan.uid].append(ag.true_pos().copy())

    arr = {uid: np.asarray(v, dtype=float) for uid, v in pos.items()}
    mode = "rules" if enable_rules else "baseline"
    decisions: List[Decision] = []
    for ag in agents:
        decisions.extend(ag.decisions)
    decisions.sort(key=lambda d: (d.t, d.uid))
    return RunResult(mode=mode, times=np.asarray(times, dtype=float), pos=arr,
                     plans={p.uid: p for p in plans}, decisions=decisions,
                     wall_time=time.time() - t0_wall, steps=n_steps)
