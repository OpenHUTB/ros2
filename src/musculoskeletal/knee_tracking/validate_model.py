#!/usr/bin/env python3
"""模型自检：加载 MJCF、检查关节/执行器/力矩/重力矩是否与参数一致。

    python3 validate_model.py
"""

from __future__ import annotations

import sys

import mujoco
import numpy as np

from mujoco_musculoskeletal_knee_tracking.config import load_config
from mujoco_musculoskeletal_knee_tracking.model import KneeSim


def main() -> int:
    cfg = load_config()
    sim = KneeSim(cfg)
    failures = 0

    def check(name: str, ok: bool, detail: str) -> None:
        nonlocal failures
        if not ok:
            failures += 1
        print(f"[{' OK ' if ok else 'FAIL'}] {name:<22} {detail}")

    check("执行器数量", sim.model.nu == 2, f"nu={sim.model.nu}（屈肌 + 伸肌）")
    check("关节自由度", sim.model.nq == 1, f"nq={sim.model.nq}，铰链")
    check("时间步长", abs(sim.model.opt.timestep - cfg.sim.dt) < 1e-12,
          f"{sim.model.opt.timestep:g} s")

    # 力矩上限 = F_max × 力臂
    limit = cfg.muscle.f_max * cfg.muscle.moment_arm
    check("最大肌肉力矩", abs(limit - cfg.ctrl.max_torque) < 1e-6,
          f"{limit:.2f} N·m（{cfg.muscle.f_max:g} N × {cfg.muscle.moment_arm:g} m）")

    # 重力矩幅值：静态偏置 50° 时 mgl·sin(50°)
    sim.reset(50.0)
    sim.data.qvel[:] = 0.0
    mujoco.mj_forward(sim.model, sim.data)
    tau_g = float(sim.data.qfrc_bias[sim.dof_idx])
    expected = 5.05 * np.sin(np.radians(50.0))
    check("重力矩", abs(abs(tau_g) - expected) < 0.15,
          f"{abs(tau_g):.3f} N·m（期望约 {expected:.3f}）")

    # 限位
    lower, upper = sim.model.jnt_range[sim.joint.id]
    check("关节限位", abs(np.degrees(lower) + 15) < 0.01
          and abs(np.degrees(upper) - 90) < 0.01,
          f"[{np.degrees(lower):.1f}, {np.degrees(upper):.1f}] deg")

    print("\n全部通过。" if failures == 0 else f"\n{failures} 项失败。")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
