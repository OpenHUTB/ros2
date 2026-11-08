# -*- coding: utf-8 -*-
"""命令行入口：跑三组实验并输出 PNG / CSV。"""

from __future__ import annotations

import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from experiments import (  # noqa: E402
    isometric_sweep,
    release_pair,
    tendon_stiffness_sweep,
)
from joint_tendon import JointParams  # noqa: E402
from muscle_tendon import TendonParams  # noqa: E402

try:
    from plots import plot_isometric, plot_release, plot_tradeoff
except ImportError:  # 没有 matplotlib 时仍然可以跑模型、输出 CSV
    plot_isometric = plot_release = plot_tradeoff = None


def _maybe_plot(func, result, path, **kwargs):
    if func is None:
        print("  未安装 matplotlib，跳过 %s" % os.path.basename(path))
        return
    func(result, path, **kwargs)
    print("  图: %s" % path)


def _hr(title):
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def _write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


def run(outdir, quick=False):
    os.makedirs(outdir, exist_ok=True)
    mp = TendonParams()
    jp = JointParams()

    _hr("肌肉-肌腱串联弹性实验")
    print("肌肉: F_max=%.1f N, l_opt=%.3f m, l_slack=%.3f m"
          % (mp.f_max, mp.l_opt, mp.l_slack))
    print("肌腱: 名义应变 %.1f%%, k_t=%.0f N/m"
          % (mp.tendon_strain * 100.0, mp.tendon_stiffness))
    print("关节: 力矩臂 %.3f m, 惯量 %.3f kg m^2, 阻尼 %.2f N m s/rad"
          % (jp.moment_arm, jp.inertia, jp.damping))

    # ---- 1. 等长传递 -------------------------------------------------
    _hr("1. 等长收缩：肌腱分走了一部分长度")
    n = 11 if quick else 21
    iso = isometric_sweep(mp, jp, n=n)
    acts = iso["acts"]
    comp, rigid = iso["compliant"], iso["rigid"]

    print("  %8s %12s %12s %8s %10s %10s %10s"
          % ("a", "F_elastic", "F_rigid", "loss", "l_m/l_opt", "strain", "E(J)"))
    for i in (n // 2, n - 1):
        f_c = comp["force"][i]
        f_r = rigid["force"][i]
        loss = (f_r - f_c) / f_r * 100.0 if f_r > 1e-9 else 0.0
        print("  %8.2f %12.1f %12.1f %7.1f%% %10.3f %9.1f%% %10.3f"
              % (acts[i], f_c, f_r, loss, comp["fiber_length_norm"][i],
                 comp["strain"][i] * 100.0, comp["energy"][i]))

    _write_csv(
        os.path.join(outdir, "isometric.csv"),
        ["activation", "force_elastic", "force_rigid", "fiber_norm",
         "tendon_strain", "energy"],
        [[acts[i], comp["force"][i], rigid["force"][i],
          comp["fiber_length_norm"][i], comp["strain"][i],
          comp["energy"][i]] for i in range(n)],
    )
    _maybe_plot(plot_isometric, iso,
                os.path.join(outdir, "isometric.png"))

    # ---- 2. 刚度权衡 -------------------------------------------------
    _hr("2. 肌腱越软，储能越多，但传力越少")
    trade = tendon_stiffness_sweep(mp, jp)
    print("  %8s %12s %12s %10s %10s"
          % ("factor", "k_t(N/m)", "force(N)", "strain", "E(J)"))
    for i, factor in enumerate(trade["factors"]):
        print("  %8.1f %12.0f %12.1f %9.1f%% %10.3f"
              % (factor, trade["k_t"][i], trade["force"][i],
                 trade["strain"][i] * 100.0, trade["energy"][i]))
    _write_csv(
        os.path.join(outdir, "tendon_stiffness.csv"),
        ["factor", "k_t", "force", "strain", "energy"],
        [[trade["factors"][i], trade["k_t"][i], trade["force"][i],
          trade["strain"][i], trade["energy"][i]]
         for i in range(len(trade["factors"]))],
    )
    _maybe_plot(plot_tradeoff, trade,
                os.path.join(outdir, "tendon_stiffness.png"))

    # ---- 3. 松手释放 -------------------------------------------------
    _hr("3. 等长保持后松手：肌腱储能释放")
    dt = 2e-3 if quick else 1e-3
    pair = release_pair(mp, jp, dt=dt)
    for name in ("compliant", "rigid"):
        m = pair[name]["metrics"]
        print("  %-10s 保持力=%6.1f N  保持能量=%6.3f J  "
              "松手后最低=%6.3f J  释放=%5.1f%%  峰值角=%5.2f rad"
              % (name, m["hold_force"], m["hold_energy"],
                 m["min_energy_after_release"], m["released_fraction"] * 100.0,
                 m["peak_theta"]))
    _write_csv(
        os.path.join(outdir, "release.csv"),
        ["t", "theta_elastic", "theta_rigid", "f_elastic", "f_rigid",
         "energy_elastic"],
        [[pair["compliant"]["history"]["t"][i],
          pair["compliant"]["history"]["theta"][i],
          pair["rigid"]["history"]["theta"][i],
          pair["compliant"]["history"]["f_flex"][i],
          pair["rigid"]["history"]["f_flex"][i],
          pair["compliant"]["history"]["energy"][i]]
         for i in range(len(pair["compliant"]["history"]["t"]))],
    )
    _maybe_plot(plot_release, pair,
                os.path.join(outdir, "release.png"), hold_time=0.4)

    print()
    print("完成。输出目录: %s" % os.path.abspath(outdir))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="肌肉-肌腱串联弹性基础实验（不依赖 ROS/MuJoCo）")
    parser.add_argument("--outdir", default="demo_out",
                        help="图片和 CSV 输出目录（默认 demo_out）")
    parser.add_argument("--quick", action="store_true",
                        help="减少采样点，快速跑一遍")
    # roslaunch 会在节点命令行末尾追加 __name:=... / __log:=...，
    # 用 parse_known_args 忽略这些 ROS 私有参数。
    args, _unknown = parser.parse_known_args(argv)
    run(args.outdir, quick=args.quick)
    return 0


if __name__ == "__main__":
    sys.exit(main())
