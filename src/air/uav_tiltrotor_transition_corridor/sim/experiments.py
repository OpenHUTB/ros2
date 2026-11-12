"""实验与作图：走廊地图、过渡剖面、保护效果、高度影响。

运行::

    python sim/experiments.py --out ../../../docs/air/uav_tiltrotor_transition_corridor
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "src"))

from uav_tiltrotor_transition_corridor import Corridor, VehicleSpec  # noqa: E402
from sim import scenarios  # noqa: E402

# 中文字体：Windows/macOS/Linux 各取常见项，找不到就退回默认（图里坐标轴会变方框，
# 但数值不受影响；CI 里只跑测试不画图）。
for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"):
    if name in {f.name for f in matplotlib.font_manager.fontManager.ttflist}:
        plt.rcParams["font.sans-serif"] = [name]
        break
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 130


_CURVE_CACHE: dict[float, tuple] = {}


def _corridor_curves(alt_m: float, v_max: float = 26.0, dv: float = 0.5):
    """走廊上下边界。倾角按 1° 扫描即可，0.5° 只是为了画得顺滑，代价是三倍时间。"""
    if alt_m in _CURVE_CACHE:
        return _CURVE_CACHE[alt_m]
    c = Corridor(alt_m=alt_m)
    vs, los, his = [], [], []
    v = 0.0
    while v <= v_max:
        b = c.tilt_bounds(v, step_deg=1.0)
        if b is not None:
            vs.append(v)
            los.append(b[0])
            his.append(b[1])
        v += dv
    out = (np.array(vs), np.array(los), np.array(his), c)
    _CURVE_CACHE[alt_m] = out
    return out


# ---------------------------------------------------------------- 图 1：走廊
def plot_corridor(out: pathlib.Path) -> dict:
    fig, ax = plt.subplots(figsize=(7.6, 4.8))
    for alt, color, label in ((0.0, "#1f77b4", "海平面"), (4000.0, "#d62728", "4000 m 机场")):
        vs, los, his, c = _corridor_curves(alt)
        ax.plot(vs, los, color=color, lw=1.8, label=f"{label} 下边界 δ_min")
        ax.plot(vs, his, color=color, lw=1.1, ls="--", alpha=0.85, label=f"{label} 上边界 δ_max")
        ax.axvline(c.stall_speed, color=color, lw=0.7, ls=":", alpha=0.7)

    # 轨迹按阶段拆开：加速段实线、减速段点线，避免同一条 V 上画两遍看不出方向
    for key, color, name in (("nominal", "#2ca02c", "标称（海平面）"),
                             ("high_altitude", "#ff7f0e", "高原（4000 m）")):
        res = scenarios.run(key)
        up = [i for i, m in enumerate(res.mode) if m in ("up", "cruise")]
        down = [i for i, m in enumerate(res.mode) if m == "down"]
        if up:
            ax.plot(res.v[up], res.tilt[up], color=color, lw=2.2, label=f"{name} 加速段")
        if down:
            ax.plot(res.v[down], res.tilt[down], color=color, lw=1.4, ls=":", label=f"{name} 减速段")

    ax.set_xlabel("空速 V (m/s)")
    ax.set_ylabel("倾转角 δ (°)")
    ax.set_title("过渡走廊：可行平飞域与实飞轨迹")
    ax.set_xlim(0, 26)
    ax.set_ylim(0, 100)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="lower left", ncol=2)
    fig.tight_layout()
    fig.savefig(out / "corridor_map.png")
    plt.close(fig)
    return {"nominal": scenarios.run("nominal").summary(),
            "high_altitude": scenarios.run("high_altitude").summary()}


# ---------------------------------------------------------------- 图 2：剖面
def plot_profile(out: pathlib.Path) -> dict:
    res = scenarios.run("nominal")
    fig, axes = plt.subplots(2, 2, figsize=(8.4, 5.6), sharex=True)
    ax = axes[0][0]
    ax.plot(res.t, res.v, color="#1f77b4", lw=1.6)
    ax.set_ylabel("空速 (m/s)")
    ax.grid(alpha=0.3)

    ax = axes[0][1]
    ax.plot(res.t, res.h, color="#2ca02c", lw=1.6)
    ax.set_ylabel("高度 (m)")
    ax.set_ylim(res.h.min() - 0.5, res.h.max() + 0.5)
    ax.grid(alpha=0.3)

    ax = axes[1][0]
    ax.plot(res.t, res.tilt, color="#d62728", lw=1.4, label="倾转角")
    ax.plot(res.t, 90.0 - res.tilt, color="#9467bd", lw=0.9, alpha=0.6, label="90° − δ")
    ax.set_ylabel("倾转角 (°)")
    ax.set_xlabel("时间 (s)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)

    ax = axes[1][1]
    ax.plot(res.t, res.lift_margin, color="#ff7f0e", lw=1.4)
    ax.axhline(0.0, color="k", lw=0.8, ls=":")
    ax.set_ylabel("升力裕度 δ − δ_min (°)")
    ax.set_xlabel("时间 (s)")
    ax.grid(alpha=0.3)

    fig.suptitle("标称过渡剖面（30 m 定高，0 → 20 m/s → 0）")
    fig.tight_layout()
    fig.savefig(out / "transform_profile.png")
    plt.close(fig)
    return res.summary()


# ---------------------------------------------------------------- 图 3：保护
def plot_guard(out: pathlib.Path) -> dict:
    a = scenarios.run("fixed_tilt_no_guard")
    b = scenarios.run("fixed_tilt_guard")
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.4))
    ax = axes[0]
    ax.plot(a.t, a.h - a.h[0], color="#d62728", lw=1.6, label="无走廊保护")
    ax.plot(b.t, b.h - b.h[0], color="#2ca02c", lw=1.6, label="有走廊保护")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("高度变化 (m)")
    ax.set_title("4000 m 机场，固定 15° 倾角")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    ax.plot(a.t, a.tilt, color="#d62728", lw=1.2, label="无保护")
    ax.plot(b.t, b.tilt, color="#2ca02c", lw=1.2, label="有保护")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("倾转角 (°)")
    ax.set_title("保护把倾角压回走廊下边界")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out / "guard_effect.png")
    plt.close(fig)
    return {"no_guard": a.summary(), "guard": b.summary()}


# ---------------------------------------------------------------- 图 4：高度
def plot_altitude(out: pathlib.Path) -> dict:
    fig, ax = plt.subplots(figsize=(7.4, 4.2))
    rows = {}
    for alt, color in ((0.0, "#1f77b4"), (2000.0, "#ff7f0e"), (4000.0, "#d62728")):
        vs, los, his, c = _corridor_curves(alt)
        ax.plot(vs, los, color=color, lw=1.6, label=f"{int(alt)} m 下边界")
        ax.plot(vs, his, color=color, lw=1.0, ls="--", alpha=0.7)
        rows[f"{int(alt)}m"] = c.summary()
    ax.set_xlabel("空速 V (m/s)")
    ax.set_ylabel("倾转角 δ (°)")
    ax.set_title("密度高度对走廊的影响：下边界整体上移")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "altitude_shift.png")
    plt.close(fig)
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="图片输出目录")
    args = ap.parse_args()
    root = pathlib.Path(__file__).resolve().parents[1]
    out = pathlib.Path(args.out) if args.out else root.parents[2] / "docs" / "air" / root.name
    out.mkdir(parents=True, exist_ok=True)

    print("== 走廊特征 ==")
    for alt in (0.0, 2000.0, 4000.0):
        print(f"  {int(alt):>5} m:", Corridor(alt_m=alt).summary())

    print("== 场景指标 ==")
    print(f"  {'场景':<22}{'中止':<6}{'能量Wh':<9}{'高度误差m':<11}{'下沉m/s':<9}{'升力裕度°'}")
    results = {}
    for key, sc in scenarios.SCENARIOS.items():
        r = scenarios.run(key)
        results[key] = r.summary()
        s = results[key]
        print(f"  {sc.title:<20}{str(s['aborted']):<8}{s['energy_wh']:<10}"
              f"{s['alt_error_m']:<11}{s['max_sink_mps']:<9}{s['min_lift_margin_deg']}")

    plot_corridor(out)
    plot_profile(out)
    plot_guard(out)
    plot_altitude(out)
    print(f"\n图片已写入 {out}")


if __name__ == "__main__":
    main()
