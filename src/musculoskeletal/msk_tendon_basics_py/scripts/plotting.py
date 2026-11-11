"""实验绘图（matplotlib 可选；缺失时安全跳过）。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

try:  # pragma: no cover - 取决于运行环境
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAS_MPL = True
except Exception:  # pragma: no cover - 无 matplotlib 的分支
    plt = None  # type: ignore[assignment]
    HAS_MPL = False


_ELASTIC = "#1f77b4"
_RIGID = "#d62728"
_ACCENT = "#2ca02c"
_GRAY = "#555555"


def _finish(fig, path: Path) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return True


def plot_isometric(data: dict, path: Path) -> bool:
    """等长传力：弹性 vs 刚性肌腱。"""

    if not HAS_MPL:
        return False
    a = np.asarray(data["activations"])
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))

    ax = axes[0]
    ax.plot(a, data["elastic_force"], "-o", ms=3.5, lw=2.0,
            color=_ELASTIC, label="Elastic tendon")
    ax.plot(a, data["rigid_force"], "--", lw=2.0,
            color=_RIGID, label="Rigid tendon")
    text_positions = {0.5: (0.30, 66.0), 1.0: (0.78, 118.0)}
    for key in (0.5, 1.0):
        idx = int(np.argmin(np.abs(a - key)))
        ax.axvline(key, color=_GRAY, lw=0.8, ls=":", alpha=0.7)
        ax.annotate(
            f"a={key:.1f}\n{data['elastic_force'][idx]:.1f} / "
            f"{data['rigid_force'][idx]:.1f} N",
            xy=(key, data["elastic_force"][idx]),
            xytext=text_positions[key],
            fontsize=8, ha="center", color=_GRAY,
            arrowprops=dict(arrowstyle="-", color=_GRAY, lw=0.7),
        )
    ax.set_xlabel("Activation $a$ (-)")
    ax.set_ylabel("Muscle-tendon force (N)")
    ax.set_title("Isometric force transmission")
    ax.set_xlim(-0.02, 1.02)
    ax.set_ylim(-4.0, 128.0)
    ax.grid(alpha=0.25)
    ax.legend(loc="upper left", fontsize=9)

    ax = axes[1]
    ax.plot(a, data["elastic_fiber_length"], "-o", ms=3.5, lw=2.0,
            color=_ELASTIC, label="Elastic tendon")
    ax.plot(a, data["rigid_fiber_length"], "--", lw=2.0,
            color=_RIGID, label="Rigid tendon")
    ax.set_xlabel("Activation $a$ (-)")
    ax.set_ylabel("Fiber length $l_m/l_{opt}$ (-)")
    ax.set_title("Fiber operating length")
    ax.set_xlim(-0.02, 1.02)
    ax.grid(alpha=0.25)
    ax.legend(loc="lower left", fontsize=9)

    ax = axes[2]
    ax.plot(a, data["elastic_strain_pct"], "-o", ms=3.5, lw=2.0,
            color=_ELASTIC, label="Tendon strain")
    ax.set_xlabel("Activation $a$ (-)")
    ax.set_ylabel("Tendon strain (%)")
    ax.set_title("Series elastic strain")
    ax.set_xlim(-0.02, 1.02)
    ax.grid(alpha=0.25)
    ax2 = ax.twinx()
    ax2.plot(a, data["elastic_energy"], "-s", ms=3.2, lw=1.5,
             color=_ACCENT, label="Stored energy")
    ax2.set_ylabel("Stored energy (J)", color=_ACCENT)
    ax2.tick_params(axis="y", labelcolor=_ACCENT)
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [ln.get_label() for ln in lines], loc="upper left", fontsize=9)

    return _finish(fig, Path(path))


def plot_tendon_stiffness(data: dict, path: Path) -> bool:
    """肌腱刚度扫描。"""

    if not HAS_MPL:
        return False
    k = np.asarray(data["stiffness"], dtype=float)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))

    ax = axes[0]
    ax.semilogx(k, data["force"], "-o", ms=5, lw=2.0, color=_ELASTIC,
                label="Simulated")
    if "reference_force" in data:
        ax.semilogx(k, data["reference_force"], "x", ms=8, mew=2,
                    color=_RIGID, label="Report")
    ax.set_xlabel("Tendon stiffness $k_t$ (N/m)")
    ax.set_ylabel("Force at a=1 (N)")
    ax.set_title("Force transmission vs stiffness")
    ax.grid(alpha=0.25, which="both")
    ax.legend(fontsize=9)

    ax = axes[1]
    ax.semilogx(k, data["strain_pct"], "-o", ms=5, lw=2.0, color=_ELASTIC)
    ax.set_xlabel("Tendon stiffness $k_t$ (N/m)")
    ax.set_ylabel("Tendon strain (%)")
    ax.set_title("Tendon strain")
    ax.grid(alpha=0.25, which="both")

    ax = axes[2]
    ax.loglog(k, data["energy"], "-o", ms=5, lw=2.0, color=_ACCENT)
    ax.set_xlabel("Tendon stiffness $k_t$ (N/m)")
    ax.set_ylabel("Stored energy (J)")
    ax.set_title("Series elastic energy")
    ax.grid(alpha=0.25, which="both")

    return _finish(fig, Path(path))


def plot_release(data: dict, path: Path) -> bool:
    """松手释放时间历程。"""

    if not HAS_MPL:
        return False
    elastic = data["elastic"]
    rigid = data.get("rigid")
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2), sharex=True)

    ax = axes[0]
    ax.plot(elastic.time, elastic.theta, lw=2.0, color=_ELASTIC,
            label="Elastic tendon")
    if rigid is not None:
        ax.plot(rigid.time, rigid.theta, "--", lw=1.8, color=_RIGID,
                label="Rigid tendon")
    ax.axvline(elastic.hold_time, color=_GRAY, ls=":", lw=1.0)
    ax.text(elastic.hold_time + 0.02, 0.24,
            "release", fontsize=8, color=_GRAY, rotation=90)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Joint angle (rad)")
    ax.set_title("Joint release")
    ax.grid(alpha=0.25)
    ax.legend(loc="lower right", fontsize=9)

    ax = axes[1]
    ax.plot(elastic.time, elastic.flex_force, lw=2.0, color=_ELASTIC)
    ax.axvline(elastic.hold_time, color=_GRAY, ls=":", lw=1.0)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Flexor force (N)")
    ax.set_title("Flexor muscle-tendon force")
    ax.grid(alpha=0.25)

    ax = axes[2]
    ax.plot(elastic.time, elastic.flex_energy, lw=2.0, color=_ACCENT)
    ax.axvline(elastic.hold_time, color=_GRAY, ls=":", lw=1.0)
    ax.axhline(elastic.pre_energy, color=_GRAY, lw=0.8, ls="--")
    ax.axhline(elastic.min_energy, color=_RIGID, lw=0.8, ls="--")
    ax.annotate(f"initial {elastic.pre_energy:.3f} J",
                xy=(0.02, elastic.pre_energy), xytext=(0.06, elastic.pre_energy),
                fontsize=8, color=_GRAY, va="bottom")
    ax.annotate(f"minimum {elastic.min_energy:.3f} J",
                xy=(elastic.peak_time, elastic.min_energy),
                xytext=(elastic.peak_time + 0.08, elastic.min_energy + 0.012),
                fontsize=8, color=_RIGID,
                arrowprops=dict(arrowstyle="->", color=_RIGID, lw=0.8))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Flexor tendon energy (J)")
    ax.set_title("Series elastic energy release")
    ax.grid(alpha=0.25)

    return _finish(fig, Path(path))


def make_all_plots(isometric: dict, stiffness: dict, release: dict,
                   outdir: Path) -> dict[str, bool]:
    """生成全部 PNG；任何一张失败都不影响主流程。"""

    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    jobs = {
        "isometric": (plot_isometric, isometric, outdir / "isometric.png"),
        "tendon_stiffness": (
            plot_tendon_stiffness, stiffness, outdir / "tendon_stiffness.png"
        ),
        "release": (plot_release, release, outdir / "release.png"),
    }
    status: dict[str, bool] = {}
    if not HAS_MPL:
        print("未检测到 matplotlib，跳过 PNG（CSV / summary.txt 仍会生成）。")
        return {name: False for name in jobs}

    for name, (func, data, path) in jobs.items():
        try:
            status[name] = bool(func(data, path))
        except Exception as exc:  # pragma: no cover - 绘图环境差异
            status[name] = False
            print(f"绘图 {name} 失败，已跳过：{exc}")
    return status
