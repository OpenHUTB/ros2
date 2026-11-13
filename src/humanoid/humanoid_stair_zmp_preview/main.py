#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形上下台阶：ZMP 预览控制 + 台阶参数扫描。

    python3 main.py --version      版本
    python3 main.py --test         离线自检（含 ZMP 前视有效性验证）
    python3 main.py --headless     台阶尺寸扫描 + CSV + 图
    python3 main.py --snapshots    场景快照
    python3 main.py --gui          交互窗口（需图形环境）
"""

import argparse
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np                                              # noqa: E402

import humanoid_stair_zmp_preview.config as config              # noqa: E402
import humanoid_stair_zmp_preview.gait as gait                  # noqa: E402
import humanoid_stair_zmp_preview.zmp as zmp_core              # noqa: E402


def _real(p):
    q = Path(p)
    return q if q.is_absolute() else (HERE / q)


def cmd_version():
    print(f"{config.PACKAGE_NAME} {config.VERSION}")
    print(f"  LIPM z_c={config.ZC:.2f} m  preview={config.N_PREVIEW} 步  "
          f"台阶 {config.STEP_COUNT}x{config.STEP_HEIGHT:.2f}m")
    return 0


def cmd_test():
    """自检：ZMP 闭环、前视有效性、IK 往返、上下台阶升幅。"""
    import numpy as np
    from humanoid_stair_zmp_preview.sim import StairSim

    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    # 1) 常量 ZMP 应稳态收敛
    ref = np.full(1000, 0.10)
    r = zmp_core.simulate(ref, z_c=config.ZC, dt=config.DT_ZMP,
                          n_preview=config.N_PREVIEW, q_e=config.Q_E, r=config.R_JERK)
    tail = slice(500, None)
    err = float(np.max(np.abs(r["zmp"][tail] - r["zmp_ref"][tail])))
    check("常量 ZMP 稳态误差 < 2 mm", err < 0.002, f"{err*1000:.2f} mm")

    # 2) 前视必须显著优于无前视（这是「预览控制」的立身之本）
    ref2 = zmp_core.staircase_zmp(n_steps=3, step_duration=1.6)
    a = zmp_core.simulate(ref2, q_e=config.Q_E, r=config.R_JERK, use_preview=True)
    b = zmp_core.simulate(ref2, q_e=config.Q_E, r=config.R_JERK, use_preview=False)
    rms_a = float(np.sqrt(np.mean((a["zmp"] - a["zmp_ref"]) ** 2)))
    rms_b = float(np.sqrt(np.mean((b["zmp"] - b["zmp_ref"]) ** 2)))
    check("开前视比关前视误差小一半以上", rms_a < 0.5 * rms_b,
          f"{rms_a*1000:.1f} vs {rms_b*1000:.1f} mm")

    # 3) IK 往返精度（中位数）
    rng = np.random.default_rng(0)
    errs = []
    for _ in range(200):
        hz = config.HIP_Z + rng.uniform(-0.1, 0.2)
        fx = rng.uniform(-0.4, 0.4)
        fz = rng.uniform(0.0, 0.4)
        hip, knee = gait.ik_leg(0.0, hz, fx, fz)
        ex, ez = gait.foot_from_angles(0.0, hz, hip, knee)
        errs.append(np.hypot(ex - fx, ez - fz))
    check("IK 往返误差中位数 < 1 mm", float(np.median(errs)) < 0.001,
          f"{np.median(errs)*1000:.2f} mm")

    # 4) 上下台阶升幅必须精确
    for d, sign in (("up", 1), ("down", -1)):
        s = StairSim(direction=d)
        r = s.run()
        s.close()
        gain = float(r["torso_z"][-1] - r["torso_z"][0])
        want = sign * config.STEP_COUNT * config.STEP_HEIGHT
        check(f"{d} 台阶升幅 = 台阶总高", abs(gain - want) < 1e-6,
              f"{gain:+.3f} vs {want:+.3f}")

    print()
    print(f"{len(fails)} 项失败" if fails else "全部通过")
    return 1 if fails else 0


def cmd_headless(outdir):
    from humanoid_stair_zmp_preview.sim import StairSim

    outdir.mkdir(parents=True, exist_ok=True)
    rows = []
    print(f"{'dir':>5} {'h(m)':>5} {'d(m)':>5} {'升幅(m)':>8} {'穿透中位(mm)':>12} {'接触':>6}")
    for d in ("up", "down"):
        for h in config.HEIGHT_GRID:
            for dep in config.DEPTH_GRID:
                s = StairSim(step_height=h, step_depth=dep, direction=d)
                r = s.run()
                s.close()
                gain = float(r["torso_z"][-1] - r["torso_z"][0])
                pen = float(np.median(r["penetration"])) * 1000
                want = (1 if d == "up" else -1) * config.STEP_COUNT * h
                rows.append(dict(direction=d, step_height=h, step_depth=dep,
                                 height_gain=gain, height_err_mm=abs(gain - want) * 1000,
                                 pen_median_mm=pen,
                                 pen_p95_mm=float(np.percentile(r["penetration"], 5)) * 1000,
                                 contacts=float(r["ncon"].mean())))
                print(f"{d:>5} {h:5.2f} {dep:5.2f} {gain:+8.3f} {pen:12.1f} {r['ncon'].mean():6.1f}")

    csv_path = outdir / "sweep.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nCSV 已写入 {csv_path}")

    # 图：升幅误差 与 穿透
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2, figsize=(8.4, 3.0))
    for d, color, marker in (("up", "#1f77b4", "o"), ("down", "#d95f02", "s")):
        sub = [r for r in rows if r["direction"] == d and r["step_depth"] == config.DEPTH_GRID[-1]]
        axs[0].plot([r["step_height"] for r in sub], [r["height_err_mm"] for r in sub],
                    marker=marker, color=color, label=d)
        axs[1].plot([r["step_height"] for r in sub], [r["pen_median_mm"] for r in sub],
                    marker=marker, color=color, label=d)
    axs[0].set_xlabel("step height (m)")
    axs[0].set_ylabel("height gain error (mm)")
    axs[0].set_title("Torso rise vs step height")
    axs[1].set_xlabel("step height (m)")
    axs[1].set_ylabel("median foot penetration (mm)")
    axs[1].set_title("Foot/step penetration")
    for ax in axs:
        ax.grid(alpha=0.25, ls="--")
        ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(outdir / "sweep.png")
    plt.close(fig)

    # 轨迹图：躯干高度随时间
    fig, ax = plt.subplots(figsize=(6.2, 3.0))
    for d, color in (("up", "#1f77b4"), ("down", "#d95f02")):
        s = StairSim(direction=d)
        r = s.run()
        s.close()
        ax.plot(r["t"], r["torso_z"], color=color, lw=1.4, label=d)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("torso height (m)")
    ax.set_title("Torso height while climbing / descending 3 steps")
    ax.grid(alpha=0.25, ls="--")
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(outdir / "torso_height.png")
    plt.close(fig)
    print(f"图已写入 {outdir}")
    return 0


def cmd_snapshots(outdir):
    from PIL import Image
    from humanoid_stair_zmp_preview.sim import StairSim
    outdir.mkdir(parents=True, exist_ok=True)
    saved = {}
    for d in ("up", "down"):
        s = StairSim(direction=d)
        plan = s.plan
        for frac, tag in ((0.0, "start"), (0.5, "mid"), (0.99, "end")):
            k = int(frac * (s.n - 1))
            fz = float(min(plan["feet"]["l"][k, 1], plan["feet"]["r"][k, 1]))
            d_ = s.data
            d_.qpos[s._qa[0]] = plan["com"][k]
            d_.qpos[s._qa[1]] = config.HIP_Z - config.CROUCH + fz - config.HIP_Z
            d_.qpos[s._qa[2]] = 0.0
            # 左右腿各 3 个关节，按 (腿, 起始 qpos 下标) 配对，避免手写 3..8 的映射表
            for leg, base_i in (("l", 3), ("r", 6)):
                joints = plan["joints"][leg]
                for off, joint_name in enumerate(("hip", "knee", "ankle")):
                    d_.qpos[s._qa[base_i + off]] = joints[joint_name][k]
            s.mujoco.mj_forward(s.model, d_)
            img = s.render(640, 360, "side")
            p = outdir / f"scene_{d}_{tag}.png"
            Image.fromarray(img).save(p)
            saved[f"{d}_{tag}"] = p
        s.close()
    for k, v in saved.items():
        print(f"  {k:12s} -> {v.name} ({v.stat().st_size//1024} KB)")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="人形上下台阶（ZMP 预览控制）")
    ap.add_argument("--version", action="store_true")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--snapshots", action="store_true")
    ap.add_argument("--outdir", type=str, default="results")
    a = ap.parse_args(argv)
    if a.version:
        return cmd_version()
    if a.test:
        return cmd_test()
    if a.snapshots:
        return cmd_snapshots(_real(a.outdir))
    if a.headless:
        return cmd_headless(_real(a.outdir))
    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())