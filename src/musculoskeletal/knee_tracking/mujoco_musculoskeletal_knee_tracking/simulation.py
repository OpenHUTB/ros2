"""轨迹、指标与离线仿真。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import Config
from .model import KneeSim


def sine_ref(t: float, bias_deg: float, amplitude_deg: float,
             frequency_hz: float) -> float:
    return bias_deg + amplitude_deg * np.sin(2.0 * np.pi * frequency_hz * t)


def step_ref(t: float, start_deg: float, target_deg: float,
             step_time: float) -> float:
    return start_deg if t < step_time else target_deg


@dataclass
class Run:
    t: np.ndarray
    theta_deg: np.ndarray
    ref_deg: np.ndarray
    error_deg: np.ndarray
    flex: np.ndarray
    ext: np.ndarray
    torque: np.ndarray


def run_tracking(sim: KneeSim, cfg: Config, kind: str,
                 duration: float | None = None,
                 initial_deg: float = 30.0) -> Run:
    """跑一段跟踪并返回时间序列。"""
    if kind not in ("sine", "step"):
        raise ValueError("kind 只能是 sine 或 step")
    traj = cfg.trajectory
    duration = cfg.sim.duration if duration is None else float(duration)
    n = int(round(duration / cfg.sim.dt))
    sim.reset(initial_deg)

    t = np.zeros(n + 1)
    theta = np.zeros(n + 1)
    ref = np.zeros(n + 1)
    flex = np.zeros(n + 1)
    ext = np.zeros(n + 1)
    torque = np.zeros(n + 1)

    for k in range(n + 1):
        time_s = k * cfg.sim.dt
        if kind == "sine":
            r = sine_ref(time_s, traj.bias_deg, traj.amplitude_deg,
                         traj.frequency_hz)
        else:
            r = step_ref(time_s, traj.step_start_deg, traj.step_target_deg,
                         traj.step_time)
        st = sim.state()
        t[k] = time_s
        theta[k] = np.degrees(st.theta_rad)
        ref[k] = r
        flex[k] = st.flex
        ext[k] = st.ext
        torque[k] = st.tau_muscle
        if k == n:
            break
        sim.step(np.radians(r))

    return Run(t=t, theta_deg=theta, ref_deg=ref,
               error_deg=theta - ref, flex=flex, ext=ext, torque=torque)


# ---------------------------------------------------------------- metrics


def sine_metrics(run: Run) -> dict[str, float]:
    err = run.error_deg
    return {
        "rmse_deg": float(np.sqrt(np.mean(err**2))),
        "max_abs_error_deg": float(np.max(np.abs(err))),
    }


def step_metrics(run: Run, cfg: Config) -> dict[str, float | None]:
    traj = cfg.trajectory
    amp = abs(traj.step_target_deg - traj.step_start_deg)
    band = cfg.metrics.band_percent / 100.0 * amp
    wide = cfg.metrics.wide_band_deg

    idx = int(np.searchsorted(run.t, traj.step_time - 1e-12))
    t = run.t[idx:] - run.t[idx]
    y = run.theta_deg[idx:]
    target = traj.step_target_deg

    peak = float(np.max(y))
    overshoot = max(0.0, peak - target) / amp * 100.0 if amp > 0 else 0.0

    def first_entry(tol: float) -> float | None:
        inside = np.abs(y - target) <= tol
        for i in range(len(inside)):
            if inside[i] and np.all(inside[i:]):
                return float(t[i])
        return None

    settle = first_entry(band)
    wide_entry = first_entry(wide)
    window = max(1, int(round(cfg.metrics.steady_window_s / cfg.sim.dt)))
    steady = float(np.mean(y[-window:]) - target)

    return {
        "overshoot_percent": float(overshoot),
        "settling_time_s": settle,
        "wide_band_entry_s": wide_entry,
        "steady_state_error_deg": steady,
        "peak_deg": peak,
        "final_deg": float(np.mean(y[-window:])),
    }
