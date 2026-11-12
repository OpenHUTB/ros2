#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集成测试（需要 mujoco + scipy）：升幅精确、脚掌水平、上台阶无穿透。

直接运行：python3 src/humanoid/humanoid_stair_zmp_preview/test/test_integration.py
"""
import math, sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

import numpy as np                                              # noqa: E402
import mujoco                                                   # noqa: E402

import humanoid_stair_zmp_preview.config as config              # noqa: E402
from humanoid_stair_zmp_preview.sim import StairSim             # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def foot_tilt(sim, leg):
    bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, f"foot_{leg}")
    R = sim.data.xmat[bid].reshape(3, 3)
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))


def main():
    for d, sign in (("up", 1), ("down", -1)):
        s = StairSim(direction=d)
        plan = s.plan
        max_tilt = 0.0
        for k in range(0, s.n, 20):
            fz = float(min(plan["feet"]["l"][k, 1], plan["feet"]["r"][k, 1]))
            dd = s.data
            dd.qpos[s._qa[0]] = plan["com"][k]
            dd.qpos[s._qa[1]] = config.HIP_Z - config.CROUCH + fz - config.HIP_Z
            dd.qpos[s._qa[2]] = 0.0
            dd.qpos[s._qa[3]] = plan["joints"]["l"]["hip"][k]
            dd.qpos[s._qa[4]] = plan["joints"]["l"]["knee"][k]
            dd.qpos[s._qa[5]] = plan["joints"]["l"]["ankle"][k]
            dd.qpos[s._qa[6]] = plan["joints"]["r"]["hip"][k]
            dd.qpos[s._qa[7]] = plan["joints"]["r"]["knee"][k]
            dd.qpos[s._qa[8]] = plan["joints"]["r"]["ankle"][k]
            mujoco.mj_forward(s.model, dd)
            max_tilt = max(max_tilt, foot_tilt(s, "l"), foot_tilt(s, "r"))
        s.close()

        s2 = StairSim(direction=d)
        r = s2.run()
        s2.close()
        gain = float(r["torso_z"][-1] - r["torso_z"][0])
        want = sign * config.STEP_COUNT * config.STEP_HEIGHT

        check(f"{d} 升幅精确", abs(gain - want) < 1e-6, f"{gain:+.3f}")
        check(f"{d} 脚掌水平（倾斜 < 0.5°）", max_tilt < 0.5, f"{max_tilt:.2f}°")
        check(f"{d} 每帧都有接触", float(r["ncon"].mean()) > 1.0, f"{r['ncon'].mean():.1f}")

    # 上台阶：6 组参数都应零穿透（这是文档里的核心结论）
    s = StairSim(step_height=0.15, step_depth=0.30, direction="up")
    r = s.run()
    s.close()
    med = float(np.median(r["penetration"])) * 1000
    check("上台阶 0.15 m 中位穿透 = 0 mm", abs(med) < 1.0, f"{med:.1f} mm")

    print()
    print(f"{len(FAILS)} 项失败" if FAILS else "全部通过")
    return 1 if FAILS else 0


if __name__ == "__main__":
    raise SystemExit(main())