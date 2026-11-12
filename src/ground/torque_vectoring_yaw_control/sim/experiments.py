"""作图与指标：横摆跟踪、正弦扫掠、四轮分配、失效降级、超限对比。"""

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

from sim import scenarios  # noqa: E402

for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"):
    if name in {f.name for f in matplotlib.font_manager.fontManager.ttflist}:
        plt.rcParams["font.sans-serif"] = [name]
        break
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 130

RED, GREEN, BLUE, ORANGE = "#d62728", "#2ca02c", "#1f77b4", "#ff7f0e"
WHEEL_LABELS = ["左前", "右前", "左后", "右后"]
WHEEL_COLORS = [BLUE, ORANGE, GREEN, RED]


def _label(key: str) -> str:
    """从场景定义生成图标题，避免标题与工况参数脱节。"""
    sc = scenarios.SCENARIOS[key]
    if sc.mode == "sine":
        return f"{sc.steer_deg:g}° 正弦 {sc.freq_hz:g} Hz @ {sc.u0:g} m/s"
    return f"{sc.steer_deg:g}° 阶跃 @ {sc.u0:g} m/s"


def plot_yaw_tracking(out: pathlib.Path) -> dict:
    """阶跃工况：横摆率跟踪 + 跟踪误差。误差曲线最能说明问题——
    无扭矩矢量时误差长时间为正，说明车身"跟不上"参考。"""
    on = scenarios.run("step_steer", True)
    off = scenarios.run("step_steer", False)
    fig, axes = plt.subplots(1, 2, figsize=(8.8, 3.6))

    ax = axes[0]
    ax.plot(on.t, np.degrees(on.r_ref), color="k", lw=1.0, ls=":", label="参考横摆率")
    ax.plot(off.t, np.degrees(off.r), color=RED, lw=1.6, label="无扭矩矢量")
    ax.plot(on.t, np.degrees(on.r), color=GREEN, lw=1.6, label="有扭矩矢量")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("横摆角速度 (°/s)")
    ax.set_title(_label("step_steer"))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    ax.plot(off.t, np.degrees(off.r - off.r_ref), color=RED, lw=1.4, label="无扭矩矢量")
    ax.plot(on.t, np.degrees(on.r - on.r_ref), color=GREEN, lw=1.4, label="有扭矩矢量")
    ax.axhline(0.0, color="k", lw=0.8, ls=":")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("横摆率误差 (°/s)")
    ax.set_title("跟踪误差（RMS 改善 {:.1f}%）".format(
        100 * (1 - on.yaw_rms_error / max(off.yaw_rms_error, 1e-12))))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out / "yaw_tracking.png")
    plt.close(fig)
    return {"no_tv": off.yaw_rms_error, "tv": on.yaw_rms_error}


def plot_sine(out: pathlib.Path) -> dict:
    """正弦扫掠：左图看相位与幅值，右图看侧偏角是否被推大。
    扭矩矢量不应该靠"多拐一点"来换跟踪——侧偏角必须基本不变。"""
    on = scenarios.run("sine_steer", True)
    off = scenarios.run("sine_steer", False)
    fig, axes = plt.subplots(1, 2, figsize=(8.8, 3.6))

    ax = axes[0]
    ax.plot(on.t, np.degrees(on.r_ref), color="k", lw=1.0, ls=":", label="参考横摆率")
    ax.plot(off.t, np.degrees(off.r), color=RED, lw=1.3, label="无扭矩矢量")
    ax.plot(on.t, np.degrees(on.r), color=GREEN, lw=1.3, label="有扭矩矢量")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("横摆角速度 (°/s)")
    ax.set_title(_label("sine_steer"))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    beta_off = np.degrees(np.arctan2(off.v, np.maximum(off.u, 1.0)))
    beta_on = np.degrees(np.arctan2(on.v, np.maximum(on.u, 1.0)))
    ax.plot(off.t, beta_off, color=RED, lw=1.3, label="无扭矩矢量")
    ax.plot(on.t, beta_on, color=GREEN, lw=1.3, label="有扭矩矢量")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("侧偏角 β (°)")
    ax.set_title("侧偏角（不该被推大）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out / "sine_tracking.png")
    plt.close(fig)
    return {"no_tv": off.yaw_rms_error, "tv": on.yaw_rms_error}


def plot_allocation(out: pathlib.Path) -> dict:
    """四轮分配与力矩兑现：力矩指令和实际兑现要贴住，
    否则说明指令已经超出附着能力，PI 在空转。"""
    r = scenarios.run("step_steer", True)
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.6))

    ax = axes[0]
    # 前后轴力臂相同、额度接近，曲线会完全重合——用线型区分前后轴，
    # 否则蓝色被绿色盖住、橙色被红色盖住，图上只剩两条线。
    for i, (lb, c) in enumerate(zip(WHEEL_LABELS, WHEEL_COLORS)):
        style = "-" if i < 2 else "--"
        ax.plot(r.t, r.fx[:, i], color=c, lw=1.3, ls=style, label=lb)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("纵向力 (N)")
    ax.set_title("四轮纵向力（实线前轴 / 虚线后轴）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=2)

    ax = axes[1]
    ax.plot(r.t, r.mz_cmd, color="#9467bd", lw=1.4, label="力矩指令")
    ax.plot(r.t, r.mz_actual, color="#8c564b", lw=1.2, ls="--", label="实际兑现")
    ax.plot(r.t, r.utilization, color="#7f7f7f", lw=1.0, alpha=0.7, label="最大附着利用率")
    ax.set_xlabel("时间 (s)")
    ax.set_title("横摆力矩与轮胎负荷")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out / "allocation.png")
    plt.close(fig)
    return {"peak_utilization": float(r.utilization.max())}


def plot_failure(out: pathlib.Path) -> dict:
    """单电机失效：右后轮额度为 0，其余三轮必须补上它的驱动力，
    同时抵掉"单侧驱动"带来的偏航力矩。"""
    off = scenarios.run("motor_failure", False)
    on = scenarios.run("motor_failure", True)
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.6))

    ax = axes[0]
    ax.plot(on.t, np.degrees(on.r_ref), color="k", lw=1.0, ls=":", label="参考")
    ax.plot(off.t, np.degrees(off.r), color=RED, lw=1.5, label="无扭矩矢量")
    ax.plot(on.t, np.degrees(on.r), color=GREEN, lw=1.5, label="有扭矩矢量")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("横摆角速度 (°/s)")
    ax.set_title("右后电机失效：横摆跟踪")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    for i, (lb, c) in enumerate(zip(WHEEL_LABELS, WHEEL_COLORS)):
        style = "-" if i != 3 else ":"
        ax.plot(on.t, on.fx[:, i], color=c, lw=1.2, ls=style, label=lb + ("（失效）" if i == 3 else ""))
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("纵向力 (N)")
    ax.set_title("失效后剩余三轮分担驱动")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=2)

    fig.tight_layout()
    fig.savefig(out / "motor_failure.png")
    plt.close(fig)
    return {"no_tv": off.yaw_rms_error, "tv": on.yaw_rms_error}


def plot_over_limit(out: pathlib.Path) -> dict:
    """超附着极限：参考被 μ 截断，实际横摆率却压不下来，侧偏角持续增大。
    这幅图是用来划边界的——这个工况救不回来。"""
    off = scenarios.run("over_limit", False)
    on = scenarios.run("over_limit", True)
    fig, axes = plt.subplots(1, 2, figsize=(8.8, 3.6))

    ax = axes[0]
    ax.plot(on.t, np.degrees(on.r_ref), color="k", lw=1.0, ls=":", label="参考（已按 μg/u 截断）")
    ax.plot(off.t, np.degrees(off.r), color=RED, lw=1.4, label="无扭矩矢量")
    ax.plot(on.t, np.degrees(on.r), color=GREEN, lw=1.4, label="有扭矩矢量")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("横摆角速度 (°/s)")
    ax.set_title(_label("over_limit"))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    ax.plot(off.t, np.degrees(np.arctan2(off.v, np.maximum(off.u, 1.0))), color=RED,
            lw=1.4, label="无扭矩矢量")
    ax.plot(on.t, np.degrees(np.arctan2(on.v, np.maximum(on.u, 1.0))), color=GREEN,
            lw=1.4, label="有扭矩矢量")
    ax.axhline(-15.0, color="k", lw=0.8, ls=":")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("侧偏角 β (°)")
    ax.set_title("侧偏角越过 15° 即已失控")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out / "over_limit.png")
    plt.close(fig)
    return {"no_tv": off.yaw_rms_error, "tv": on.yaw_rms_error}


PLOTS = (plot_yaw_tracking, plot_sine, plot_allocation, plot_failure, plot_over_limit)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = pathlib.Path(args.out) if args.out else _ROOT.parents[2] / "docs" / "ground" / _ROOT.name
    out.mkdir(parents=True, exist_ok=True)

    print("== 有/无扭矩矢量对比 ==")
    header = f"  {'场景':<14}{'RMS(无)':>10}{'RMS(有)':>10}{'改善':>8}{'β(无→有)':>14}{'利用率':>12}"
    print(header)
    for key, sc in scenarios.SCENARIOS.items():
        a = scenarios.run(key, False)
        b = scenarios.run(key, True)
        gain = 100.0 * (1.0 - b.yaw_rms_error / max(a.yaw_rms_error, 1e-9))
        print(f"  {sc.title:<12}{a.yaw_rms_error:>10.4f}{b.yaw_rms_error:>10.4f}"
              f"{gain:>7.1f}%{a.max_sideslip_deg:>7.1f} →{b.max_sideslip_deg:>5.1f}"
              f"{a.utilization.max():>7.2f}/{b.utilization.max():<5.2f}")

    for fn in PLOTS:
        fn(out)
    print(f"\n图片已写入 {out}")


if __name__ == "__main__":
    main()
