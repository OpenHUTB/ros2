"""命令行入口：check / run / bench。"""

import argparse
import json
import os

from . import core_mj as core, runs


def cmd_check(_args):
    meas, clean = core.make_measurement(core.profile_square, 0.0)
    print(f"  MuJoCo 关节模型，{len(meas)} 个采样点")
    print(f"  关节角范围 {clean.min():.2f} ~ {clean.max():.2f} rad"
          f"（限位 ±{core.THETA_LIMIT}）")
    print(f"  待辨识参数 {core.PARAM_NAMES}")
    print(f"  真值 {core.TRUE}")


def cmd_run(args):
    print(json.dumps(SCENES[args.scene](), ensure_ascii=False, indent=2))


def cmd_bench(args):
    os.makedirs(args.out, exist_ok=True)
    res = {n: SCENES[n]() for n in ("i1", "i2", "i3", "i4", "i5")}
    path = os.path.join(args.out, "metrics.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(f"指标已写入 {path}")
    for p in res["i1"]["params"]:
        print(f"  I1 {p['name']:8s} {p['true']:9.4f} -> {p['est']:9.4f}"
              f"（{p['err_pct']:.2f}%）")
    for lv in res["i2"]["levels"]:
        print(f"  I2 噪声 {lv['noise_pct']:3.0f}% -> 中位误差 {lv['mean_err_pct']:.2f}%")
    print("  I3 相关性: " + ", ".join(
        f"{p['a']}~{p['b']} {p['corr']:+.2f}" for p in res["i3"]["top_pairs"]))
    for lv in res["i4"]["levels"]:
        print(f"  I4 初值 {lv['range_lo']:.1f}~{lv['range_hi']:.1f}x -> "
              f"成功率 {lv['success_rate_pct']:.0f}%")
    for lv in res["i5"]["levels"]:
        print(f"  I5 {lv['excitation']:12s} D={lv['d_optimal']:.2e} "
              f"误差 {lv['mean_err_pct']:.2f}%")


SCENES = {"i1": runs.run_i1, "i2": runs.run_i2, "i3": runs.run_i3,
          "i4": runs.run_i4, "i5": runs.run_i5}


def main(argv=None):
    ap = argparse.ArgumentParser(description="肌肉参数辨识")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="模型自检").set_defaults(func=cmd_check)
    p = sub.add_parser("run", help="运行单个实验")
    p.add_argument("--scene", required=True, choices=sorted(SCENES))
    p.set_defaults(func=cmd_run)
    p = sub.add_parser("bench", help="运行全部实验")
    p.add_argument("--out", default="out")
    p.set_defaults(func=cmd_bench)
    args = ap.parse_args(argv)
    args.func(args)

if __name__ == "__main__":
    main()