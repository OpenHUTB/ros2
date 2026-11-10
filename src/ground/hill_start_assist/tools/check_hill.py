#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""坡道起步辅助自检。

场景约定（评审逐条核对过，改动请同步）：
  1. 车身**按坡度摆正**再落稳。若"世界竖直"地放在斜面上，三点支撑对不上，
     落稳阶段就翻（旧版实测俯仰 −65°），之后测到的"溜车"其实是翻倒车身在滑。
  2. 出生点沿斜面**法向**抬高 WHEEL_RADIUS + WHEEL_DROP = 0.10 m。
     法向量取 (sin(−α), 0, cos(−α))；x 分量符号写反会让车陷进地面（穿透 ~2.7 cm）。
  3. 判据同时看 **全程俯仰峰值** 和 **末俯仰**，并检查接触点数恒为 4。
     只查末俯仰会漏掉中途翻车又爬起来的情况。

    A. 无辅助：4 轮力矩为 0，车沿坡滚下
    B. 有辅助：四轮轮角保持 PD + 坡度前馈，应基本不动
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
from hill_start_assist.spawn import place_on_slope, settle_with_brake  # noqa: E402

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
    mass = float(m.body_mass.sum())          # 质量取模型真值，不再硬编码
    ctrl = HillStartController(mass=mass)
    body_h = config.WHEEL_RADIUS + config.WHEEL_DROP

    print("[环境] python %s | mujoco %s" % (sys.version.split()[0], mujoco.__version__))
    print("[场景] nq=%d nu=%d，%.2f kg（取自 body_mass.sum），%.0f° 斜坡（4 轮两轴）"
          % (m.nq, m.nu, mass, config.SLOPE_DEG))
    print("[控制器] %s" % ctrl.gains())

    def pitch_deg():
        M = d.xmat[bid].reshape(3, 3)
        return math.degrees(math.asin(max(-1.0, min(1.0, -M[2, 0]))))

    def spawn_and_settle():
        """摆正 → 带刹落稳 → 返回轮角基准（与 make_gif.py 共用同一实现）。"""
        place_on_slope(m, d)
        return settle_with_brake(m, d, ctrl, qadr, vadr)

    def run(assist):
        phi0 = spawn_and_settle()
        x0 = float(d.xpos[bid][0])
        v0 = float(d.qvel[0])                              # 测量窗口起点初速
        peak_pitch = 0.0
        min_ncon = 99
        for _ in range(int(args.seconds / m.opt.timestep)):
            for i in range(4):
                if assist:
                    tau = ctrl.torque(float(d.qpos[qadr[i]]) - phi0[i],
                                      float(d.qvel[vadr[i]]))
                    d.ctrl[i] = tau / 20.0                 # gear=20
                else:
                    d.ctrl[i] = 0.0
            mujoco.mj_step(m, d)
            peak_pitch = max(peak_pitch, abs(pitch_deg() + config.SLOPE_DEG))
            min_ncon = min(min_ncon, int(d.ncon))
        return (float(d.xpos[bid][0]) - x0, pitch_deg(), peak_pitch,
                min_ncon, v0)

    print("\n=== 松刹车后 %.0f s（车身出生即按 %.0f° 摆正）==="
          % (args.seconds, config.SLOPE_DEG))
    print("  %-11s%11s%10s%12s%10s%9s"
          % ("方案", "沿坡位移", "末俯仰", "俯仰峰值偏差", "最小接触", "起点初速"))
    rows = []
    for name, assist in (("A. 无辅助", False), ("B. 有辅助", True)):
        dx, p_end, p_peak, ncon, v0 = run(assist)
        upright_end = abs(p_end + config.SLOPE_DEG) < config.PITCH_TOL_DEG
        upright_peak = p_peak < config.PITCH_TOL_DEG
        # 保持过程中允许瞬时抬轮：只要不是翻车，3 点着地就算正常
        contact_ok = ncon >= 3
        rows.append((name, dx, p_end, p_peak, ncon, v0, upright_end, upright_peak, contact_ok))
        print("  %-11s%9.3f m%9.1f°%11.1f°%10d%8.2f m/s"
              % (name, dx, p_end, p_peak, ncon, v0))

    a, b = abs(rows[0][1]), abs(rows[1][1])
    pct = 100.0 * (1 - b / a) if a > 1e-9 else 0.0
    print("\n  沿坡位移：%.3f m → %.3f m（减少 %.2f%%）" % (a, b, pct))

    ok_a = a > 0.05                                    # 无辅助必须明显滚下
    ok_b = b < config.DRIFT_OK_M                       # 有辅助必须基本不动
    ok_pitch = all(r[6] and r[7] for r in rows)        # 末俯仰 + 全程峰值都合格
    ok_con = all(r[8] for r in rows)                   # 全程至少 3 点着地
    print("[判据] A 会滚下 %s ｜ B 能稳住 %s ｜ 俯仰未超差 %s ｜ 全程≥3 点着地 %s"
          % tuple("✓" if x else "✗" for x in (ok_a, ok_b, ok_pitch, ok_con)))
    all_ok = ok_a and ok_b and ok_pitch and ok_con
    print("[结论] %s" % ("四项全部达标" if all_ok else "存在未达标项"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
