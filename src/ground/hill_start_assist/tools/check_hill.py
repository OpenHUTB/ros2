#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""坡道起步辅助自检。

关键：车身**必须按坡度摆正**再落稳。早期版本让车"世界竖直"地放在斜面上，
三点支撑对不上，落稳阶段就翻了（实测俯仰 −65°），后面测到的"溜车"其实是
翻倒的车身在滑。这里把出生位姿、落稳、俯仰判据都写死在代码里。

    A. 无辅助：4 轮力矩为 0，车沿坡滚下
    B. 有辅助：四轮轮角保持 PD + 坡度前馈，应基本不动
    C. 两项都要检查"车身俯仰是否仍贴近坡度"，翻了直接判不通过
"""

import argparse
import math
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

import mujoco                                          # noqa: E402

from hill_start_assist import config                    # noqa: E402
from hill_start_assist.control import HillStartController   # noqa: E402

WHEELS = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")


def main(argv=None):
    ap = argparse.ArgumentParser(description="坡道起步辅助自检")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args(argv)

    m = mujoco.MjModel.from_xml_path(config.resolve_scene_path(None))
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    qadr, vadr = [], []
    for name in WHEELS:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        qadr.append(int(m.jnt_qposadr[j]))
        vadr.append(int(m.jnt_dofadr[j]))

    slope = math.radians(config.SLOPE_DEG)
    ctrl = HillStartController()

    print("[环境] python %s | mujoco %s" % (sys.version.split()[0], mujoco.__version__))
    print("[场景] nq=%d nu=%d，%.1f kg，%.0f° 斜坡（4 轮两轴）"
          % (m.nq, m.nu, m.body_mass.sum(), config.SLOPE_DEG))
    print("[控制器] %s" % ctrl.gains())

    def pitch_deg():
        M = d.xmat[bid].reshape(3, 3)
        return math.degrees(math.asin(max(-1.0, min(1.0, -M[2, 0]))))

    def spawn_and_settle():
        """按坡度摆正车身，沿法向抬高一个轮半径，然后静置落稳。"""
        mujoco.mj_resetData(m, d)
        n = (-math.sin(-slope), 0.0, math.cos(-slope))
        d.qpos[0] = config.WHEEL_RADIUS * n[0]
        d.qpos[2] = config.WHEEL_RADIUS * n[2]
        d.qpos[3] = math.cos(slope / 2)
        d.qpos[5] = -math.sin(slope / 2)
        mujoco.mj_forward(m, d)
        for _ in range(int(config.SETTLE_S / m.opt.timestep)):
            mujoco.mj_step(m, d)
        return [float(d.qpos[a]) for a in qadr]

    def run(assist):
        phi0 = spawn_and_settle()
        x0 = float(d.xpos[bid][0])
        peak_pitch = 0.0
        for _ in range(int(args.seconds / m.opt.timestep)):
            for i in range(4):
                if assist:
                    tau = ctrl.torque(float(d.qpos[qadr[i]]) - phi0[i],
                                      float(d.qvel[vadr[i]]))
                    d.ctrl[i] = tau / 20.0      # gear=20
                else:
                    d.ctrl[i] = 0.0
            mujoco.mj_step(m, d)
            peak_pitch = max(peak_pitch, abs(pitch_deg() + config.SLOPE_DEG))
        return (float(d.xpos[bid][0]) - x0, pitch_deg(), peak_pitch,
                int(d.ncon))

    print("\n=== 松刹车后 %.0f s（车身出生即按 %.0f° 摆正）==="
          % (args.seconds, config.SLOPE_DEG))
    print("  %-12s%12s%12s%14s" % ("方案", "沿坡位移", "末俯仰", "车身姿态"))
    rows = []
    for name, assist in (("A. 无辅助", False), ("B. 有辅助", True)):
        dx, p_end, p_peak, ncon = run(assist)
        upright = abs(p_end + config.SLOPE_DEG) < config.PITCH_TOL_DEG
        rows.append((name, dx, p_end, p_peak, upright, ncon))
        print("  %-12s%10.3f m%11.1f°%14s"
              % (name, dx, p_end, "未翻 ✓" if upright else "**翻了 ✗**"))

    a, b = abs(rows[0][1]), abs(rows[1][1])
    print("\n  沿坡位移：%.3f m → %.3f m（减少 %.0f%%）"
          % (a, b, 100 * (1 - b / a) if a > 1e-9 else 0))

    ok_a = a > 0.05                       # 无辅助必须明显滚下来
    ok_b = b < config.DRIFT_OK_M          # 有辅助必须基本不动
    ok_upright = all(r[4] for r in rows)  # 两段都不能翻
    print("\n[判据] A 无辅助会滚下 %s ｜ B 有辅助能稳住 %s ｜ 车身未翻 %s"
          % ("✓" if ok_a else "✗", "✓" if ok_b else "✗",
             "✓" if ok_upright else "✗"))
    all_ok = ok_a and ok_b and ok_upright
    print("[结论] %s" % ("三项全部达标" if all_ok else "存在未达标项"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
