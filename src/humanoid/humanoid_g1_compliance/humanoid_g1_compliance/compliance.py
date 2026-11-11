"""柔顺性实验：外力施加、指标采集与对比。

LAC 的「柔顺」体现在：策略把 5 通道刚度当作观测输入，在同样的外力下，
刚度指令越低，末端让位越大、关节力矩峰值越低、恢复越慢。本模块提供

  * ``Perturbation``   —— 描述一次外力（作用 body、方向、大小、起止时刻）
  * ``ComplianceRunner`` —— 跑一段闭环，记录时间序列
  * ``ComplianceMetrics`` —— 从序列里算让位量、力矩峰值、恢复时间

注意外力施加在**手腕**而不是躯干：躯干受力会直接挑战全身平衡，几十牛的
方波就能把 34 kg 的 G1 推倒，指标会淹没在摔倒里；手腕受力才是论文里
「人手接触、机器人柔顺让位」的场景，且躯干基本不动。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .policy import Command, LacPolicy, StartupFSM
from .sim_core import G1Sim

POLICY_HZ = 50.0


@dataclass
class Perturbation:
    body: str = 'left_wrist_roll_rubber_hand'
    axis: str = 'y'          # x / y / z（世界坐标）
    force: float = 6.0       # N，正负号即方向
    t_start: float = 3.0     # s
    duration: float = 2.0    # s

    def axis_index(self) -> int:
        return {'x': 0, 'y': 1, 'z': 2}[self.axis]


@dataclass
class RunLog:
    """一次闭环运行的完整时间序列。"""

    t: List[float] = field(default_factory=list)
    pelvis: List[np.ndarray] = field(default_factory=list)
    hand: List[np.ndarray] = field(default_factory=list)
    tau: List[np.ndarray] = field(default_factory=list)
    pg_z: List[float] = field(default_factory=list)
    kp: List[np.ndarray] = field(default_factory=list)
    state: List[str] = field(default_factory=list)
    fell: bool = False

    def arrays(self) -> Dict[str, np.ndarray]:
        return {
            't': np.asarray(self.t),
            'pelvis': np.asarray(self.pelvis),
            'hand': np.asarray(self.hand),
            'tau': np.asarray(self.tau),
            'pg_z': np.asarray(self.pg_z),
            'kp': np.asarray(self.kp),
        }


@dataclass
class ComplianceMetrics:
    """一次扰动的柔顺指标。"""

    body: str
    force: float
    k_lin_la: float
    baseline_mm: float = 0.0        # 施力前末端位置基线
    peak_mm: float = 0.0            # 施力期间最大让位
    steady_mm: float = 0.0          # 施力末段（稳态）让位
    settle_mm: float = 0.0          # 撤销 2 s 后残留
    recover_s: float = float('nan')  # 回到峰值 10% 以内所需时间
    tau_peak: float = 0.0           # 关节力矩峰值 N·m
    pelvis_shift_mm: float = 0.0    # 骨盆最大偏移
    fell: bool = False

    def effective_stiffness(self) -> float:
        """等效末端刚度 F/Δx（N/m），用稳态让位计算。"""
        if self.steady_mm <= 1e-6:
            return float('inf')
        return abs(self.force) / (self.steady_mm / 1000.0)

    def as_dict(self) -> dict:
        return {
            'body': self.body,
            'force_N': round(self.force, 3),
            'K_lin_la': round(self.k_lin_la, 1),
            'peak_mm': round(self.peak_mm, 1),
            'steady_mm': round(self.steady_mm, 1),
            'settle_mm': round(self.settle_mm, 1),
            'recover_s': None if np.isnan(self.recover_s) else round(self.recover_s, 2),
            'tau_peak_Nm': round(self.tau_peak, 2),
            'pelvis_shift_mm': round(self.pelvis_shift_mm, 1),
            'k_eff_N_per_m': round(self.effective_stiffness(), 1),
            'fell': self.fell,
        }


class ComplianceRunner:
    """把 G1Sim + LacPolicy + StartupFSM 串成一条可复现的闭环。"""

    def __init__(self, sim: G1Sim, policy: LacPolicy,
                 fsm: Optional[StartupFSM] = None, substeps: int = 4):
        self.sim = sim
        self.policy = policy
        self.fsm = fsm or StartupFSM(direct=True)
        self.substeps = int(substeps)
        self.dt = sim.timestep * self.substeps
        assert abs(self.dt - 1.0 / POLICY_HZ) < 1e-9, (
            f'策略周期 {self.dt:.4f}s 不是 {1 / POLICY_HZ:.2f}s，检查 timestep/substeps')

    def run(self, seconds: float, cmd: Optional[Command] = None,
            perturb: Optional[Perturbation] = None,
            body: str = 'left_wrist_roll_rubber_hand',
            support: Optional[float] = None) -> RunLog:
        """跑 seconds 秒闭环，可选施加一次外力。

        Args:
          support: 若给出（0~1），在骨盆施加竖直向上的比例支撑力，托住
            自重 × support。仅用于软启动演示；``direct`` 模式不需要。
        """
        cmd = (cmd or Command()).clamp()
        self.sim.reset()
        self.policy.reset()
        self.fsm.reset()

        hand_id = self.sim.body_id(body)
        push_id = self.sim.body_id(perturb.body) if perturb else -1
        axis = perturb.axis_index() if perturb else 0

        log = RunLog()
        n = int(round(seconds / self.dt))
        mass_n = self.sim.total_mass() * 9.81

        for k in range(n):
            now = k * self.dt
            st = self.sim.read()
            q_policy = self.policy.infer(st, cmd)
            q_cmd, kp, kd = self.fsm.step(now, st.q, q_policy)

            self.sim.clear_forces()
            if support:
                self.sim.apply_force(self.sim.pelvis_id, (0.0, 0.0, support * mass_n))
            if perturb and perturb.t_start <= now < perturb.t_start + perturb.duration:
                f = np.zeros(3)
                f[axis] = perturb.force
                self.sim.apply_force(push_id, f)

            self.sim.apply_pd(q_cmd, kp, kd)
            self.sim.step(self.substeps)

            st2 = self.sim.read()
            pg_z = float(self.sim.projected_gravity(st2.quat)[2])
            log.t.append(now + self.dt)
            log.pelvis.append(self.sim.pelvis_pos())
            log.hand.append(self.sim.body_pos(hand_id))
            log.tau.append(st2.tau.copy())
            log.pg_z.append(pg_z)
            log.kp.append(kp.copy())
            log.state.append(self.fsm.state)

            if pg_z > -0.5 or self.sim.pelvis_pos()[2] < 0.35:
                log.fell = True
                break

        return log

    # ------------------------------------------------------------------
    @staticmethod
    def metrics(log: RunLog, perturb: Perturbation,
                k_lin_la: float = 250.0) -> ComplianceMetrics:
        """从时间序列里算柔顺指标。"""
        t = np.asarray(log.t)
        hand = np.asarray(log.hand)
        pel = np.asarray(log.pelvis)
        tau = np.asarray(log.tau)
        axis = perturb.axis_index()

        pre = t < perturb.t_start
        during = (t >= perturb.t_start) & (t < perturb.t_start + perturb.duration)
        post = t >= perturb.t_start + perturb.duration

        if pre.sum() < 3:
            base = hand[0, axis]
        else:
            base = float(np.mean(hand[pre][-10:, axis]))
        base_pel = pel[pre][-10:].mean(axis=0) if pre.sum() >= 3 else pel[0]

        m = ComplianceMetrics(body=perturb.body, force=perturb.force,
                              k_lin_la=k_lin_la, baseline_mm=base * 1000.0)
        if during.sum() == 0:
            return m

        disp = (hand[during, axis] - base) * 1000.0
        m.peak_mm = float(np.max(np.abs(disp)))
        # 稳态：施力段最后 0.5 s
        tail = t[during] >= perturb.t_start + perturb.duration - 0.5
        m.steady_mm = float(np.mean(np.abs(disp[tail]))) if tail.any() else m.peak_mm

        if post.sum() > 0:
            m.settle_mm = float(abs(hand[post][-1, axis] - base) * 1000.0)
            # 恢复时间：撤销后首次回到峰值 10% 以内
            thr = max(0.1 * m.peak_mm, 1.0)
            idx = np.where(np.abs((hand[post, axis] - base) * 1000.0) <= thr)[0]
            m.recover_s = float(t[post][idx[0]] - (perturb.t_start + perturb.duration)) \
                if idx.size else float('nan')

        m.tau_peak = float(np.max(np.abs(tau[during])))
        m.pelvis_shift_mm = float(np.max(np.linalg.norm(
            pel[during][:, :2] - base_pel[:2], axis=1)) * 1000.0)
        m.fell = bool(log.fell)
        return m


__all__ = ['Perturbation', 'RunLog', 'ComplianceMetrics', 'ComplianceRunner',
           'POLICY_HZ']
