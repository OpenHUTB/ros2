"""命令行入口：check / run / bench / gif。"""

import argparse
import json
import os

import mujoco

from . import core, extra, runs


def cmd_check(_args):
    print(f"mujoco {mujoco.__version__}")
    m = mujoco.MjModel.from_xml_string(core.gripper_xml())
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mujoco.mj_step(m, d)
    print(f"  模型 nv={m.nv}（手掌 + 两指 + 方块），"
          f"方块 z={d.qpos[m.jnt_qposadr[m.joint('objfree').id] + 2]:+.3f} OK")
    print(f"  手指行程上限 {m.jnt_range[m.joint('flj').id][1]:.3f} m，"
          f"肌肉 F_max={core.F_MAX:.0f} N")


def cmd_run(args):
    print(json.dumps(SCENES[args.scene](), ensure_ascii=False, indent=2))


def cmd_bench(args):
    os.makedirs(args.out, exist_ok=True)
    res = {name: SCENES[name]() for name in ("g1", "g2", "g3", "g4", "g5", "g6", "g7", "g8")}
    path = os.path.join(args.out, "metrics.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(f"指标已写入 {path}")
    for lv in res["g1"]["levels"]:
        print(f"  G1 激活 {lv['activation']:.2f} -> 夹持力 {lv['grip_force_n']:.2f} N")
    ok = [lv for lv in res["g2"]["levels"] if lv["gripped"]]
    print(f"  G2 能夹住的激活阈值 ≈ {min(l['activation'] for l in ok):.2f}" if ok else "  G2 全部打滑")
    print(f"  G4 松开后 {res['g4']['drop_after_release_s']:.2f} s 掉落")
    for lv in res["g5"]["levels"]:
        print(f"  G5 目标 {lv['target_n']:.0f} N -> 稳态 {lv['steady_n']:.2f} N，"
              f"上升 {lv['rise_time_s']:.2f} s，末激活 {lv['final_activation']:.2f}")
    print(f"  G6 无反射 {res['g6']['no_reflex']['lift_mm']:.1f} mm / "
          f"有反射 {res['g6']['with_reflex']['lift_mm']:.1f} mm，"
          f"加力 {res['g6']['with_reflex']['reflex_steps']} 步 "
          f"(+{res['g6']['with_reflex']['reflex_activation_gain']:.2f} 激活)")
    for lv in res["g7"]["levels"]:
        print(f"  G7 {lv['mass_g']:.0f} g -> 最小激活 {lv['min_activation']}")
    print(f"  G8 手臂-夹爪抬升 {res['g8']['lift_mm']:.1f} mm")


def cmd_gif(args):
    os.makedirs(args.out, exist_ok=True)
    rec = runs.Recorder()
    runs._simulate(0.60, rec=rec)
    size = rec.save(os.path.join(args.out, "g2_grasp_lift.gif"), scale=0.55)
    print(f"  g2_grasp_lift.gif  {len(rec.frames)} 帧, {size / 1e6:.2f} MB")

    rec2 = runs.Recorder(every=40)
    runs._simulate(0.15, rec=rec2, seconds=3.0)
    size2 = rec2.save(os.path.join(args.out, "g2_slip.gif"), scale=0.55)
    print(f"  g2_slip.gif  {len(rec2.frames)} 帧, {size2 / 1e6:.2f} MB")

    from . import arm_grasp
    rec3 = arm_grasp.Rec(every=75)
    arm_grasp.run(rec=rec3)
    size3 = rec3.save(os.path.join(args.out, "g8_arm_grasp.gif"), scale=0.55)
    print(f"  g8_arm_grasp.gif  {len(rec3.frames)} 帧, {size3 / 1e6:.2f} MB")


SCENES = {"g1": runs.run_g1, "g2": runs.run_g2,
          "g3": runs.run_g3, "g4": runs.run_g4,
          "g5": extra.run_g5, "g6": extra.run_g6, "g7": extra.run_g7,
          "g8": extra.run_g8}


def main(argv=None):
    ap = argparse.ArgumentParser(description="肌肉驱动夹爪：真实接触抓取")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="模型自检").set_defaults(func=cmd_check)

    p = sub.add_parser("run", help="运行单个实验")
    p.add_argument("--scene", required=True, choices=sorted(SCENES))
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("bench", help="运行全部实验")
    p.add_argument("--out", default="out")
    p.set_defaults(func=cmd_bench)

    p = sub.add_parser("gif", help="渲染动图")
    p.add_argument("--out", default="gif")
    p.set_defaults(func=cmd_gif)

    args = ap.parse_args(argv)
    args.func(args)
