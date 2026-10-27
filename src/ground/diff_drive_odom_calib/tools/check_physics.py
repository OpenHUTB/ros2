#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 物理仿真自检：确认这台车"真的在物理引擎里跑"，并量化打滑。

为什么要有这个脚本：
    里程计标定/滑移补偿的前提是"存在真实的、非线性的打滑"。
    如果仿真里轮子不打滑，或者打滑是数值噪声（粘滑振荡），
    后面所有标定和神经网络的对比都失去意义。
    这里逐项把这件事测出来，任何一项不达标就说明场景配错了。

四项检查：
    A. 静置：车体水平、四点接触、不下陷
    B. 直行：稳态纵向滑移率（应在 3%~15%）
    C. 转向：原地旋转与弧线行驶的角速度损失
    D. 一致性：滑移随速度变化 → 证明它是非线性、可学习的

用法：
    python3 tools/check_physics.py
    python3 tools/check_physics.py --scene ../models/diff_drive.xml
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

from diff_drive_odom_calib import config                        # noqa: E402
from diff_drive_odom_calib.mujoco_env import DiffDriveEnv, describe_env   # noqa: E402


def segment_slip(env, v, w, seconds=4.0):
    """分段统计纵向滑移率：%(1 - 车体位移/轮缘位移)。"""
    env.reset((0.0, 0.0, 0.0))
    env.set_twist(v, w)
    segs = []
    for _ in range(int(seconds)):
        d = env.measure(1.0)
        segs.append(d["slip"])
    return segs


def main(argv=None):
    ap = argparse.ArgumentParser(description="MuJoCo 物理仿真自检")
    ap.add_argument("--scene", default=None)
    args = ap.parse_args(argv)

    print("[环境] %s" % describe_env())
    env = DiffDriveEnv(config.resolve_scene_path(args.scene))
    print("[模型] 自由度 nq=%d，执行器 nu=%d，整备质量 %.2f kg，步长 %.4f s，控制 %.0f Hz"
          % (env.model.nq, env.model.nu, float(env.model.body_mass.sum()),
             env.dt, config.CONTROL_HZ))

    # ---- A. 静置 ----
    print("\n=== A. 静置检查 ===")
    env.reset((0.0, 0.0, 0.0))
    st = env.state()
    contact = env.contact_count()
    print("  车体高度 z=%.4f m（轮半径 %.3f）" % (st.z, config.WHEEL_RADIUS))
    print("  姿态 up_z=%.4f（1.0 表示完全水平）" % st.up_z)
    print("  接触点数 %d（两驱动轮 + 一个万向球，正常 3~7）" % contact)
    ok_a = abs(st.z - config.WHEEL_RADIUS) < 0.01 and st.up_z > 0.98 and contact >= 3
    print("  → %s" % ("通过" if ok_a else "**不通过**"))

    # ---- B. 直行滑移 ----
    print("\n=== B. 直行滑移（每秒一段，第一段含起步瞬态）===")
    print("  %6s%10s%10s%10s%10s%12s" % ("速度", "第1秒", "第2秒", "第3秒", "第4秒", "稳态均值"))
    ok_b = True
    rows = []
    for v in (0.2, 0.3, 0.5, 0.7):
        segs = segment_slip(env, v, 0.0, 4.0)
        # 低速时前 2 秒仍在加速，稳态取后两秒
        steady = float(np.mean(segs[2:])) * 100.0
        rows.append((v, steady))
        print("  %6.1f%9.1f%%%9.1f%%%9.1f%%%9.1f%%%11.1f%%"
              % (v, segs[0] * 100, segs[1] * 100, segs[2] * 100, segs[3] * 100, steady))
        if not (3.0 <= steady <= 15.0):
            ok_b = False
    print("  → %s（判据：稳态滑移 3%%~15%%，太小说明轮子没接触，太大说明数值不稳定）"
          % ("通过" if ok_b else "**不通过**"))

    # ---- C. 转向 ----
    print("\n=== C. 转向损失（指令角速度 vs 实际角速度）===")
    print("  %10s%12s%12s%10s" % ("指令 w", "实际 w", "损失", "备注"))
    ok_c = True
    for w_cmd in (0.5, 1.0, 1.5):
        env.reset((0.0, 0.0, 0.0))
        env.set_twist(0.0, w_cmd)
        d = env.measure(1.5)
        loss = 1.0 - abs(d["w_actual"]) / w_cmd
        note = "正常" if loss < 0.2 else ("偏大" if loss < 0.4 else "打滑严重")
        print("  %10.2f%12.3f%11.0f%%%10s" % (w_cmd, d["w_actual"], loss * 100, note))
        if abs(loss) > 0.5:
            ok_c = False

    # ---- D. 非线性：滑移随转向率变化 ----
    # 实测滑移几乎不随速度变（4.5%~5.4%，极差不到 1 个百分点），
    # 但随**转向率**明显上升 —— 差速车转向时轮胎要横向刮擦，
    # 这部分误差与 (v, ω) 强相关，常数标定系数 c_s/c_theta 修不掉，
    # 正是神经网络要学的东西。
    print("\n=== D. 滑移随转向率的变化（线性标定修不掉的非线性）===")
    print("  %8s%10s%12s" % ("转向率 w", "纵向滑移", "说明"))
    slips_w = []
    for w_cmd in (0.0, 0.4, 0.8, 1.2):
        env.reset((0.0, 0.0, 0.0))
        env.set_twist(0.30, w_cmd)
        env.measure(1.5)                       # 丢掉起步瞬态
        d = env.measure(1.5)
        slips_w.append(abs(d["slip"]) * 100.0)
        print("  %8.1f%9.1f%%%12s" % (w_cmd, abs(d["slip"]) * 100,
                                      "直行" if w_cmd == 0 else "转向刮擦加重"))
    spread = max(slips_w) - min(slips_w)
    print("  极差 %.1f 个百分点" % spread)
    ok_d = spread > 1.5
    print("  → %s" % ("滑移随转向率显著变化，属于非线性误差 —— 需要神经网络补偿"
                      if ok_d else "滑移几乎恒定，NN 补偿意义不大"))

    # ---- E. 渲染（证明真的在 MuJoCo 里，不是纸上积分）----
    frame_ok = False
    try:
        from PIL import Image
        env.reset((0.0, 0.0, 0.0))
        env.set_twist(0.4, 0.0)
        env.step(4)
        f1 = env.render(480, 270)
        env.set_twist(0.2, 1.0)
        env.step(6)
        f2 = env.render(480, 270)
        out = PKG_ROOT / "results" / "physics_render.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        canv = Image.new("RGB", (480 * 2 + 8, 270), (255, 255, 255))
        canv.paste(Image.fromarray(f1), (0, 0))
        canv.paste(Image.fromarray(f2), (488, 0))
        canv.save(out)
        frame_ok = True
        print("\n=== E. MuJoCo 渲染 ===")
        print("  已保存直行 / 转向两帧：%s" % out)
    except Exception as exc:
        print("\n=== E. MuJoCo 渲染 ===\n  渲染不可用（%s）" % exc)

    all_ok = ok_a and ok_b and ok_c and ok_d
    print("\n[结论] %s" % ("物理仿真自检通过，可以进入标定与 NN 训练" if all_ok
                           else "存在未通过项，见上面 **不通过** 标记"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
