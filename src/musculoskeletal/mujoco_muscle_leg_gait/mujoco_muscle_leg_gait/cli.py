"""命令行入口实现（setup.py 的 console_scripts 指向这里）。"""

import argparse
import json
import os
import sys

import mujoco

from . import core, runs


def cmd_check(_args):
    print(f"mujoco {mujoco.__version__} / python {sys.version.split()[0]}")
    models = (("single_joint", core.single_joint_xml()),
              ("two_joint", core.two_joint_xml()),
              ("legs_1", core.legs_xml(1)),
              ("legs_2", core.legs_xml(2, 0.35)))
    for name, xml in models:
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        mujoco.mj_step(m, d)
        print(f"  {name:<12} nq={m.nq} ngeom={m.ngeom} "
              f"timestep={m.opt.timestep:.4f} OK")
    print("全部模型加载并推进一步成功")


def cmd_run(args):
    print(json.dumps(SCENES[args.scene](), ensure_ascii=False, indent=2))


def cmd_bench(args):
    os.makedirs(args.out, exist_ok=True)
    res = {}
    for name in ("l1", "l2", "l3", "l4", "l5", "l6"):
        print(f"运行 {name} ...", flush=True)
        res[name] = SCENES[name]()
    path = os.path.join(args.out, "metrics.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(f"指标已写入 {path}")


def cmd_gif(args):
    os.makedirs(args.out, exist_ok=True)
    jobs = [
        ("l1_single_muscle.gif", runs.run_l1,
         dict(every=15, lookat=(0, 0, -0.10), distance=0.75)),
        ("l2_pd_tracking.gif", runs.run_l2,
         dict(every=20, lookat=(0, 0, -0.10), distance=0.75)),
        ("l5_weight_bearing_stand.gif", runs.run_l5,
         dict(every=20, lookat=(0, 0, -0.30), distance=1.15)),
        ("l6_stepping.gif", runs.run_l6,
         dict(every=25, lookat=(0, 0, -0.30), distance=1.15)),
    ]
    for fname, fn, kw in jobs:
        rec = runs.Recorder(**kw)
        fn(rec=rec)
        size = rec.save(os.path.join(args.out, fname))
        print(f"  {fname}  {len(rec.frames)} 帧, {size / 1e6:.2f} MB")


SCENES = {
    "l1": runs.run_l1, "l2": runs.run_l2, "l3": runs.run_l3,
    "l4": runs.run_l4, "l5": runs.run_l5, "l6": runs.run_l6,
}


def main(argv=None):
    ap = argparse.ArgumentParser(description="肌肉驱动下肢：六级实验")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="模型自检").set_defaults(func=cmd_check)

    p_run = sub.add_parser("run", help="运行单个实验")
    p_run.add_argument("--scene", required=True, choices=sorted(SCENES))
    p_run.set_defaults(func=cmd_run)

    p_bench = sub.add_parser("bench", help="运行全部实验并导出指标")
    p_bench.add_argument("--out", default="out")
    p_bench.set_defaults(func=cmd_bench)

    p_gif = sub.add_parser("gif", help="渲染动图")
    p_gif.add_argument("--out", default="gif")
    p_gif.set_defaults(func=cmd_gif)

    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
