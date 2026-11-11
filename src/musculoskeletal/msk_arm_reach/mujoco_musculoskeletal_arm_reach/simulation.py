"""闭环到达仿真与指标。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import Config
from .control import ReachController
from .model import ArmModel


@dataclass
class ReachRun:
    t: np.ndarray
    tip: np.ndarray
    q: np.ndarray
    errors: np.ndarray
    activations: np.ndarray
    forces: np.ndarray

    def metrics(self) -> dict[str, float | None]:
        err = self.errors
        first = next((i for i, e in enumerate(err) if e < 0.01), None)
        return {
            "final_error_m": float(err[-1]),
            "min_error_m": float(np.min(err)),
            "max_error_m": float(np.max(err)),
            "time_to_1cm_s": None if first is None else float(self.t[first]),
        }


def run_reach(cfg: Config, target=None, seconds: float = 3.2,
              q0=(0.0, 0.6)) -> ReachRun:
    if target is None:
        target = (cfg.ctrl.target_x, cfg.ctrl.target_z)
    target = np.asarray(target, dtype=float)
    if target.shape != (2,):
        raise ValueError("target 必须是 (x, z)")

    arm = ArmModel(cfg)
    arm.reset(q0[0], q0[1])
    ctl = ReachController(cfg)
    n = int(round(seconds / cfg.sim.control_period))

    t = np.zeros(n + 1)
    tip = np.zeros((n + 1, 2))
    q = np.zeros((n + 1, 2))
    acts = np.zeros((n + 1, 6))
    forces = np.zeros((n + 1, 6))

    for k in range(n + 1):
        tip[k] = arm.tip_position
        q[k] = (arm.q1, arm.q2)
        acts[k] = arm.activations
        forces[k] = arm.forces
        t[k] = k * cfg.sim.control_period
        if k == n:
            break
        ctl.step(arm, (float(target[0]), float(target[1])))
        arm.step(cfg.sim.steps_per_control)

    errors = np.linalg.norm(tip - target[None, :], axis=1)
    return ReachRun(t=t, tip=tip, q=q, errors=errors, activations=acts,
                    forces=forces)
