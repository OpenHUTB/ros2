#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形足迹下的局部避障算法对比：窄通道宽度扫描。

    python3 main.py --version        版本
    python3 main.py --test           离线自检（不依赖 ROS）
    python3 main.py --headless       宽度扫描 + CSV + 图
    python3 main.py --snapshots      三张场景快照
    python3 main.py --gui            交互窗口（需图形环境）
    python3 main.py --export-model   重新生成 models/corridor_scene.xml
"""

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import humanoid_passage_width_sweep.config as config          # noqa: E402


def _real(p):
    q = Path(p)
    return q if q.is_absolute() else (HERE / q)


def cmd_version():
    print(f"{config.PACKAGE_NAME} {config.VERSION}")
    print(f"  footprint r={config.FOOTPRINT_RADIUS:.2f} m, "
          f"inflate {config.INFLATE_POINT:.2f}/{config.INFLATE_FOOTPRINT:.2f} m, "
          f"rays {config.N_RAYS}")
    try:
        import mujoco
        print(f"  mujoco {mujoco.__version__}")
    except ImportError:
        print("  mujoco: 未安装")
    return 0


def cmd_test():
    """离线自检：模型可加载、激光有效、四变体都能跑几步。"""
    import numpy as np

    from humanoid_passage_width_sweep import planners, scene
    from humanoid_passage_width_sweep.sim import PassageSim

    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    check("净宽 0.90 m 的门口单侧净空 0.15 m",
          abs(scene.gate_clearance(0.90) - 0.15) < 1e-9, f"{scene.gate_clearance(0.90):.3f} m")
    check("0.62 m 门仍可通行（比足迹宽 2 cm）", scene.is_passable(0.62))
    check("0.58 m 门判为不可通行", not scene.is_passable(0.58))

    sim = PassageSim(width=0.90, variant="mppi_footprint")
    check("场景加载", sim.model.ngeom > 10, f"{sim.model.ngeom} geoms")
    pts = sim.scan()
    n_hit = int(np.isfinite(pts).all(axis=1).sum())
    check("激光有命中", n_hit > 10, f"{n_hit}/{config.N_RAYS} 束")
    check("起点不与墙接触", not sim.in_contact())
    sim.close()

    for v in planners.VARIANT_ORDER:
        sim = PassageSim(width=0.90, variant=v)
        log = sim.run(duration=2.0)
        moved = float(np.hypot(np.diff(log["x"]).sum(), np.diff(log["y"]).sum())) if len(log["x"]) > 1 else 0.0
        check(f"变体 {v} 能推动机器人", moved > 0.3, f"位移 {moved:.2f} m")
        sim.close()

    print(f"\n{len(fails)} 项失败" if fails else "\n全部通过")
    return 1 if fails else 0


def cmd_headless(repeats, outdir, export_docs):
    from humanoid_passage_width_sweep import experiments, figures, metrics, planners

    import numpy as np
    outdir.mkdir(parents=True, exist_ok=True)
    print("=" * 78)
    print(f"宽度扫描：{len(planners.VARIANT_ORDER)} 个变体 x {len(config.WIDTH_GRID)} 档门宽 "
          f"x {repeats} 次重复")
    done = [0]
    total = len(planners.VARIANT_ORDER) * len(config.WIDTH_GRID) * repeats

    def prog(r):
        done[0] += 1
        if done[0] % 4 == 0 or done[0] == total:
            print(f"  {done[0]}/{total}  {r.variant:20s} w={r.width:.2f}  "
                  f"{'通过' if r.row['reached'] else '失败'}  "
                  f"净空 {r.row['min_clear']:+.3f}")

    results = experiments.sweep_widths(repeats=repeats, progress=prog)
    agg = experiments.summarize(results)
    metrics.write_csv(outdir / "runs.csv", [r.row for r in results])
    metrics.write_csv(outdir / "summary.csv", agg)

    print("=" * 78)
    print(f"{'变体':22s} {'门宽':>5s} {'通过':>6s} {'用时(s)':>8s} {'净空(m)':>8s} {'路径(m)':>8s}")
    for r in agg:
        print(f"{r['label']:22s} {r['width']:5.2f} {r['success']}/{r['runs']:<4d} "
              f"{r['time']:8.2f} {r['min_clear']:8.3f} {r['path']:8.2f}")

    print("=" * 78)
    print("各变体总通过率：")
    for v in planners.VARIANT_ORDER:
        rows = [r for r in agg if r["variant"] == v]
        s = sum(r["success"] for r in rows)
        n = sum(r["runs"] for r in rows)
        print(f"  {planners.label_of(v):22s} {s}/{n}  ({100.0 * s / n:.0f}%)")

    figures.plot_success(agg, outdir / "success_vs_width.png")
    figures.plot_clearance(agg, outdir / "clearance_vs_width.png")
    # 轨迹图：取通过的变体在最窄档上的一次成功运行
    best = None
    for r in results:
        if r.row["reached"] and r.variant == "mppi_footprint":
            best = r
            break
    if best is not None:
        from humanoid_passage_width_sweep.experiments import run_once
        keep = run_once(best.variant, best.width, keep_log=True)
        figures.plot_trajectory(keep, outdir / "trajectory.png")

    if export_docs:
        import shutil
        docdir = HERE.parent.parent.parent / "docs" / "img" / "humanoid" / config.PACKAGE_NAME
        docdir.mkdir(parents=True, exist_ok=True)
        for name in ("success_vs_width.png", "clearance_vs_width.png", "trajectory.png"):
            src = outdir / name
            if src.is_file():
                shutil.copyfile(src, docdir / name)
        print(f"图表已导出到 {docdir}")
    print(f"结果已写入 {outdir}")
    return 0


def cmd_snapshots(width, outdir):
    from humanoid_passage_width_sweep import figures
    saved = figures.snapshots(width=width, outdir=outdir)
    for k, v in saved.items():
        print(f"  {k:8s} -> {v.name}  ({v.stat().st_size // 1024} KB)")
    return 0


def cmd_gui(width, variant):
    import mujoco.viewer
    from humanoid_passage_width_sweep.sim import PassageSim
    sim = PassageSim(width=width, variant=variant)
    print("窗口已打开，关闭窗口结束。")
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            sim.step()
            viewer.sync()
    return 0


def cmd_export_model():
    from humanoid_passage_width_sweep import scene
    p = HERE / "models" / config.MODEL_NAME
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(scene.build_scene_xml(config.DEFAULT_WIDTH), encoding="utf-8")
    print(f"已写出 {p}（默认净宽 {config.DEFAULT_WIDTH:.2f} m）")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="人形足迹下的局部避障算法对比")
    ap.add_argument("--version", action="store_true")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--snapshots", action="store_true")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--export-model", action="store_true")
    ap.add_argument("--export-docs", action="store_true")
    ap.add_argument("--repeats", type=int, default=config.REPEATS)
    ap.add_argument("--width", type=float, default=config.DEFAULT_WIDTH)
    ap.add_argument("--variant", type=str, default="mppi_footprint")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--outdir", type=str, default="results")
    args = ap.parse_args(argv)

    if args.version:
        return cmd_version()
    if args.test:
        return cmd_test()
    if args.export_model:
        return cmd_export_model()
    if args.snapshots:
        return cmd_snapshots(args.width, _real(args.outdir))
    if args.gui:
        return cmd_gui(args.width, args.variant)
    if args.headless:
        return cmd_headless(args.repeats, _real(args.outdir), args.export_docs)
    ap.print_help()
    return 2


if __name__ == "__main__":
    import numpy as np  # noqa: F401  (cmd_test 里用到)
    raise SystemExit(main())