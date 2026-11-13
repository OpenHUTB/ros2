# -*- coding: utf-8 -*-
"""三项实验：定高弹跳、抗扰恢复、弹簧刚度参数扫描。"""

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

import mujoco_hopper_slip_control.config as config
from mujoco_hopper_slip_control import metrics
from mujoco_hopper_slip_control.sim import HopperSim


@dataclass
class ExperimentResult:
    """一项实验的完整结果。"""

    name: str
    log: Dict[str, np.ndarray]
    hops: List[metrics.HopRecord]
    stats: Dict[str, float]
    extra: Dict[str, float]

    @property
    def ok(self) -> bool:
        return self.stats.get("count", 0) >= config.MIN_HOPS


def run_height(target_height: float = config.TARGET_HEIGHT, duration: float = 12.0,
               target_speed: float = 0.0) -> ExperimentResult:
    """A. 定高弹跳：验证顶点高度收敛到目标值。"""
    sim = HopperSim(target_height=target_height, target_speed=target_speed)
    log = sim.run(duration)
    sim.close()
    hops = metrics.extract_hops(log)
    stats = metrics.steady_stats(hops)
    stats["target_height"] = target_height
    stats["apex_error"] = abs(stats.get("apex_mean", float("nan")) - target_height) \
        if stats.get("count") else float("nan")
    return ExperimentResult("height", log, hops, stats, {"target_height": target_height})


def run_disturbance(target_height: float = config.TARGET_HEIGHT, duration: float = 14.0,
                    dv: float = config.DISTURB_DV,
                    t0: float = config.DISTURB_T) -> ExperimentResult:
    """B. 抗扰恢复：飞行中施加水平速度冲量。

    判据是**竖直方向**的：扰动后顶点是否回到 ``target ± TOL_APEX``。
    横向漂移量单独报告，不作为成功判据（点足 + 自由髋的横向控制权限弱）。
    """
    sim = HopperSim(target_height=target_height)
    log = sim.run(duration, disturb_at=t0, disturb_dv=dv, stop_on_fall=False)
    sim.close()
    hops = metrics.extract_hops(log)
    stats = metrics.steady_stats(hops)
    after_mask = log["t"] >= t0
    after_hops = [h for h in hops if h.t_liftoff > t0]
    apex_rec = metrics.apex_recovery_time(after_hops, t0, target=target_height)

    stats["recovery_time"] = apex_rec
    stats["apex_error"] = abs(stats.get("apex_mean", float("nan")) - target_height) \
        if stats.get("count") else float("nan")
    extra = {
        "disturb_dv": dv,
        "disturb_t": t0,
        "vx_peak": float(np.abs(log["vx"][after_mask]).max()),
        "vx_final": float(log["vx"][-1]),
        "drift": float(log["x"][-1] - log["x"][after_mask][0]),
        "z_min_after": float(log["z"][after_mask].min()),
        "apex_mean_after": float(np.mean([h.z_apex for h in after_hops]))
        if after_hops else float("nan"),
        "survived": float(log["z"][-1] > config.FALL_Z),
        "vx_settle_time": metrics.recovery_time(log, t0),
    }
    return ExperimentResult("disturbance", log, hops, stats, extra)


def run_stiffness_sweep(stiffnesses: Optional[List[float]] = None,
                        target_height: float = config.TARGET_HEIGHT,
                        duration: float = 12.0) -> ExperimentResult:
    """C. 弹簧刚度扫描：刚度如何影响支撑时间、峰值力与注能需求。

    这是 SLIP 模型的核心结论（刚度↑ → 支撑相↓、峰值力↑），现有实现完全没有涉及。
    """
    from dataclasses import replace
    import mujoco_hopper_slip_control.sim as sim_mod

    stiffnesses = stiffnesses or [1100.0, 1600.0, 2200.0, 2800.0, 3400.0]
    rows: List[dict] = []
    original = sim_mod.config.LEG_STIFFNESS
    for k in stiffnesses:
        # 以运行时寄存器改写：MuJoCo 的 joint_stiffness 直接在模型里改
        sim = HopperSim(target_height=target_height)
        jid = sim._mujoco.mj_name2id(sim.model, sim._mujoco.mjtObj.mjOBJ_JOINT, "leg")
        sim.model.jnt_stiffness[jid] = float(k)
        log = sim.run(duration)
        sim.close()
        hops = metrics.extract_hops(log)
        st = metrics.steady_stats(hops)
        rows.append({
            "stiffness": float(k),
            "apex_mean": st.get("apex_mean", float("nan")),
            "stance_mean": st.get("stance_mean", float("nan")),
            "flight_mean": st.get("flight_mean", float("nan")),
            "peak_force": st.get("peak_force_mean", float("nan")),
            "injected_force": st.get("injected_force_mean", float("nan")),
            "duty": st.get("duty", float("nan")),
            "hops": st.get("count", 0),
        })
    sim_mod.config.LEG_STIFFNESS = original

    # 用噪声极小的合成 log 承载表格数据，避免 ExperimentResult 要求真实 log
    fake = {k: np.zeros(1) for k in ("t", "z", "vz", "x", "vx", "q", "qd",
                                     "hip_q", "phase", "f_ex", "leg_cmd", "contact")}
    return ExperimentResult("sweep", fake, [], {"count": len(rows)}, {"rows": rows})
