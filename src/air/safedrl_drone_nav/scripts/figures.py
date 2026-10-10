"""
Publication-quality figure generation for the evaluation report.

All figures are rendered headlessly (``Agg`` backend) at 200 dpi into
``docs/figures/`` and are embedded in the README.  Every function takes plain
arrays/dicts so it can be driven either from a live evaluation or from the
JSON traces stored under ``logs/traces/``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless - required inside the ROS/VM environment

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

plt.rcParams.update(
    {
        "figure.dpi": 200,
        "savefig.dpi": 200,
        "font.size": 9,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "axes.axisbelow": True,
        "figure.autolayout": True,
    }
)

C_DRONE = "#1b6ec2"
C_DES = "#9aa0a6"
C_SAFE = "#e8710a"
C_STATIC = "#c0392b"
C_DYN = "#e6a700"
C_GATE = "#12805c"
C_MARGIN = "#6a1b9a"


def _save(fig, out: Path):
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  [figure] {out}")
    return out


# ==========================================================================
# 1. 3-D flight trajectory
# ==========================================================================


def plot_trajectory_3d(trace: dict, gates: np.ndarray, obstacles: list, out: Path,
                       title: str = "Safe DRL navigation - 3-D flight trajectory",
                       obstacle_paths: dict | None = None):
    """3-D path with start/finish, gate frames and obstacle bounding spheres."""
    pos = np.asarray(trace["pos"], dtype=float)
    t = np.asarray(trace["t"], dtype=float)
    margin = np.asarray(trace["margin"], dtype=float)

    fig = plt.figure(figsize=(9.5, 7.0))
    ax = fig.add_subplot(111, projection="3d")

    # colour the path by safety margin -> one glance shows *where* it got tight.
    # A plain LineCollection is 2-D only; in 3-D the segments must be wrapped in
    # a Line3DCollection (or mpl raises "'vertices' must be 2D with shape (N, 2)").
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    vmin, vmax = float(np.min(margin)), float(np.max(margin))
    if vmax - vmin < 1e-6:
        vmax = vmin + 1e-6
    norm = plt.Normalize(vmin=vmin, vmax=vmax)
    pts = pos.reshape(-1, 1, 3)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = Line3DCollection(segs, cmap="viridis", norm=norm, linewidth=2.6)
    lc.set_array(margin[:-1])
    ax.add_collection3d(lc)

    # gates
    for i, g in enumerate(gates):
        _draw_gate_3d(ax, g, label=f"gate {i + 1}" if i == 0 else None)

    # static obstacles
    u, v = np.mgrid[0:2 * np.pi:16j, 0:np.pi:8j]
    for ob in obstacles:
        if ob["kind"] != "static":
            continue
        r, c = ob["radius"], np.asarray(ob["centre"], float)
        ax.plot_surface(
            c[0] + r * np.cos(u) * np.sin(v), c[1] + r * np.sin(u) * np.sin(v),
            c[2] + r * np.cos(v), color=C_STATIC, alpha=0.28, linewidth=0, shade=True,
        )

    # dynamic obstacles: bounding spheres at sampled times
    dyn = [o for o in obstacles if o["kind"] == "dynamic"]
    if dyn:
        for k, ob in enumerate(dyn):
            r = ob["radius"]
            centres = ob.get("centres")
            if centres is not None and len(centres):
                centres = np.asarray(centres, float)
                step = max(1, len(centres) // 14)
                for c in centres[::step]:
                    ax.plot_surface(
                        c[0] + r * np.cos(u) * np.sin(v), c[1] + r * np.sin(u) * np.sin(v),
                        c[2] + r * np.cos(v), color=C_DYN, alpha=0.13, linewidth=0, shade=False,
                    )
                ax.plot(centres[:, 0], centres[:, 1], centres[:, 2],
                        color=C_DYN, ls="--", lw=1.0, alpha=0.75,
                        label="dynamic obstacle path" if k == 0 else None)
            else:
                c = np.asarray(ob["centre"], float)
                ax.plot_surface(
                    c[0] + r * np.cos(u) * np.sin(v), c[1] + r * np.sin(u) * np.sin(v),
                    c[2] + r * np.cos(v), color=C_DYN, alpha=0.2, linewidth=0, shade=False,
                )

    ax.scatter(*pos[0], color="lime", s=70, marker="o", edgecolor="k", zorder=10,
               label="start")
    ax.scatter(*pos[-1], color="red", s=90, marker="*", edgecolor="k", zorder=10,
               label="end")
    ax.plot([], [], [], color="k", lw=2.6, label="flight path (colour = safety margin)")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=7.5, framealpha=0.9)
    ax.view_init(elev=22, azim=-62)

    sm = plt.cm.ScalarMappable(cmap="viridis", norm=norm)
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax, shrink=0.62, pad=0.09)
    cb.set_label("safety margin  h(x)  [m]")
    return _save(fig, out)


def _draw_gate_3d(ax, g, aperture=0.58, bar=0.055, label=None):
    x, y, z = float(g[0]), float(g[1]), float(g[2])
    half = aperture + bar
    for sy in (-1.0, 1.0):
        ax.plot([x, x], [y + sy * half, y + sy * half], [z - half, z + half],
                color=C_GATE, lw=3.0, solid_capstyle="round")
    for sz in (-1.0, 1.0):
        ax.plot([x, x], [y - half, y + half], [z + sz * half, z + sz * half],
                color=C_GATE, lw=3.0, solid_capstyle="round", label=label)


# ==========================================================================
# 2. State time-domain response
# ==========================================================================


def plot_state_response(trace: dict, out: Path, title: str = "State time-domain response"):
    """Velocity, attitude, safety margin, rotor speed and wind vs time."""
    t = np.asarray(trace["t"], float)
    vel = np.asarray(trace["vel"], float)
    rpy = np.asarray(trace["rpy"], float)
    margin = np.asarray(trace["margin"], float)
    v_des = np.asarray(trace["v_des"], float)
    v_safe = np.asarray(trace["v_safe"], float)
    rpm = np.asarray(trace["rpm"], float)
    wind = np.asarray(trace["wind"], float)

    fig, axes = plt.subplots(5, 1, figsize=(10.0, 11.5), sharex=True)
    lab = ["x", "y", "z"]

    ax = axes[0]
    for i in range(3):
        ax.plot(t, vel[:, i], lw=1.3, label=f"$v_{lab[i]}$ actual")
        ax.plot(t, v_des[:, i], lw=0.9, ls=":", color=C_DES,
                label=f"$v_{lab[i]}$ commanded" if i == 0 else None)
    ax.plot(t, np.linalg.norm(v_des, axis=1), color=C_DES, lw=1.0, ls="--",
            label="$\\|v_{des}\\|$")
    ax.plot(t, np.linalg.norm(v_safe, axis=1), color=C_SAFE, lw=1.0,
            label="$\\|v_{safe}\\|$ (after CBF)")
    ax.set_ylabel("velocity [m/s]")
    ax.legend(ncol=3, fontsize=6.5, loc="upper right")
    ax.set_title(title)

    ax = axes[1]
    for i, name in enumerate(["roll $\\phi$", "pitch $\\theta$", "yaw $\\psi$"]):
        ax.plot(t, np.degrees(rpy[:, i]), lw=1.2, label=name)
    ax.axhline(0, color="k", lw=0.6)
    ax.set_ylabel("attitude [deg]")
    ax.legend(ncol=3, fontsize=7)

    ax = axes[2]
    ax.plot(t, margin, color=C_MARGIN, lw=1.5, label="safety margin $h(x)=d-d_{safe}$")
    ax.axhline(0, color="red", lw=1.3, ls="--", label="collision / unsafe boundary")
    ax.fill_between(t, margin, 0, where=(margin < 0), color="red", alpha=0.25,
                    label="violation")
    ax.set_ylabel("margin [m]")
    ax.legend(fontsize=7)
    if margin.size:
        ax.set_ylim(min(-0.15, float(margin.min()) * 1.15), float(margin.max()) * 1.2 + 0.1)

    ax = axes[3]
    for i in range(4):
        ax.plot(t, rpm[:, i], lw=0.9, label=f"rotor {i}")
    ax.axhline(25000, color="r", lw=0.9, ls="--", label="max rpm")
    ax.set_ylabel("rotor speed [rpm]")
    ax.legend(ncol=5, fontsize=6.5)

    ax = axes[4]
    for i, name in enumerate(lab):
        ax.plot(t, wind[:, i], lw=1.1, label=f"$w_{name}$")
    ax.plot(t, np.linalg.norm(wind, axis=1), color="k", lw=1.0, ls=":", label="$\\|w\\|$")
    ax.set_ylabel("wind [m/s]")
    ax.set_xlabel("time [s]")
    ax.legend(ncol=4, fontsize=7)

    return _save(fig, out)


# ==========================================================================
# 3. Training convergence
# ==========================================================================


def plot_convergence(curves: dict, out: Path, title: str = "Training reward convergence",
                     smooth: int = 15):
    """Reward / cost / multiplier / success convergence from TensorBoard scalars.

    ``curves`` maps a label to ``{"step": [...], "value": [...]}``.
    """
    n = len(curves)
    fig, axes = plt.subplots(1, n, figsize=(4.6 * n, 3.4), squeeze=False)
    axes = axes[0]
    for ax, (label, series) in zip(axes, curves.items()):
        xs = np.asarray(series["step"], float)
        ys = np.asarray(series["value"], float)
        ax.plot(xs, ys, lw=0.9, alpha=0.35, color=C_DRONE, label="raw")
        if ys.size >= smooth:
            k = np.ones(smooth) / smooth
            ax.plot(xs[smooth - 1:], np.convolve(ys, k, mode="valid"), lw=1.8,
                    color=C_DRONE, label=f"moving avg ({smooth})")
        ax.set_title(label, fontsize=9)
        ax.set_xlabel("environment steps")
        ax.legend(fontsize=6.5)
    fig.suptitle(title, fontsize=11)
    return _save(fig, out)


def plot_training_dashboard(series: dict, out: Path, title: str = "PPO-Lagrangian training dashboard"):
    """Four-panel dashboard: return, cost vs limit, multiplier, success rate."""
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 6.6))

    def _series(*names):
        for nm in names:
            if nm in series and len(series[nm]["step"]):
                return (np.asarray(series[nm]["step"], float),
                        np.asarray(series[nm]["value"], float))
        return None, None

    ax = axes[0, 0]
    x, y = _series("rollout/ep_rew_mean", "train/mean_reward", "ep_rew_mean")
    if x is not None:
        ax.plot(x, y, lw=1.0, alpha=0.4, color=C_DRONE)
        if y.size > 15:
            ax.plot(x[14:], np.convolve(y, np.ones(15) / 15, "valid"), lw=1.9, color=C_DRONE)
    ax.set_title("episodic return"); ax.set_xlabel("env steps"); ax.set_ylabel("reward")

    ax = axes[0, 1]
    x, y = _series("constraint/mean_episode_cost", "eval/mean_episode_cost", "ep_cost")
    if x is not None:
        ax.plot(x, y, lw=1.6, color=C_STATIC, label="mean episodic cost")
    lim = series.get("_cost_limit")
    if lim is not None:
        ax.axhline(lim, color="k", ls="--", lw=1.2, label=f"constraint limit d={lim}")
    ax.set_title("constraint satisfaction"); ax.set_xlabel("env steps"); ax.set_ylabel("cost")
    ax.legend(fontsize=7)

    ax = axes[1, 0]
    x, y = _series("constraint/lambda")
    if x is not None:
        ax.plot(x, y, lw=1.6, color=C_SAFE)
    ax.set_title("Lagrange multiplier $\\lambda$ (dual ascent)")
    ax.set_xlabel("env steps"); ax.set_ylabel("$\\lambda$")

    ax = axes[1, 1]
    x, y = _series("eval/success_rate", "rollout/success_rate", "ep_success")
    if x is not None:
        ax.plot(x, y, lw=1.7, color=C_GATE)
    ax.set_ylim(0, 1.05)
    ax.set_title("evaluation success rate"); ax.set_xlabel("env steps"); ax.set_ylabel("rate")

    fig.suptitle(title, fontsize=12)
    return _save(fig, out)


# ==========================================================================
# 4. Safety-margin analysis and ablation
# ==========================================================================


def plot_margin_distribution(per_policy: dict, out: Path,
                             title: str = "Minimum safety margin distribution"):
    """Box + strip plot of the per-episode minimum margin for each policy."""
    labels = list(per_policy.keys())
    data = [np.asarray(per_policy[k], float) for k in labels]
    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    bp = ax.boxplot(data, labels=labels, widths=0.5, patch_artist=True, showfliers=False)
    for patch, colour in zip(bp["boxes"], [C_DES, C_SAFE, C_GATE, C_DRONE][: len(labels)]):
        patch.set_facecolor(colour)
        patch.set_alpha(0.45)
    for i, d in enumerate(data, start=1):
        ax.scatter(np.full(d.size, i) + np.linspace(-0.12, 0.12, d.size), d,
                   s=9, color="k", alpha=0.45, zorder=3)
    ax.axhline(0, color="red", ls="--", lw=1.4, label="unsafe boundary  h = 0")
    ax.set_ylabel("min safety margin over episode [m]")
    ax.set_title(title)
    ax.legend(fontsize=8)
    return _save(fig, out)


def plot_ablation(table: dict, out: Path, title: str = "Ablation: safety vs. task performance"):
    """Grouped bar chart over the three metrics that matter to a reviewer."""
    policies = list(table.keys())
    # keys must match MetricsRecorder.summary()
    metrics = [
        ("success_rate", "success rate", 1.0, C_GATE),
        ("collision_rate", "collision rate", 1.0, C_STATIC),
        ("mean_cbf_intervention_rate", "CBF intervention rate", 1.0, C_SAFE),
        ("worst_contact_clearance_m", "worst contact clearance [m]", None, C_DRONE),
    ]
    fig, axes = plt.subplots(1, len(metrics), figsize=(3.4 * len(metrics), 3.8))
    for ax, (key, label, scale, colour) in zip(axes, metrics):
        vals = [float(table[p].get(key) or 0.0) for p in policies]
        bars = ax.bar(range(len(policies)), vals, color=colour, alpha=0.8, width=0.6)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.3g}",
                    ha="center", va="bottom", fontsize=7.5)
        ax.set_xticks(range(len(policies)))
        ax.set_xticklabels(policies, rotation=18, ha="right", fontsize=7.5)
        ax.set_title(label, fontsize=9)
        if scale:
            ax.set_ylim(0, max(scale, max(vals) * 1.25 if vals else scale))
    fig.suptitle(title, fontsize=11.5)
    return _save(fig, out)


def plot_metric_table(table: dict, out: Path, title: str = "Evaluation metrics"):
    """Render the metrics table itself as a figure (handy for slides)."""
    policies = list(table.keys())
    keys = [k for k in table[policies[0]].keys() if k != "policy"]
    fig, ax = plt.subplots(figsize=(9.5, 0.42 * len(keys) + 1.6))
    ax.axis("off")
    cell = [[f"{table[p].get(k, float('nan')):.4g}" if isinstance(table[p].get(k), (int, float))
             else str(table[p].get(k, "")) for p in policies] for k in keys]
    tb = ax.table(cellText=cell, rowLabels=keys, colLabels=policies,
                  loc="center", cellLoc="center")
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    tb.scale(1, 1.25)
    for (r, c), cst in tb.get_celld().items():
        if r == 0:
            cst.set_facecolor("#dfe7ef")
            cst.set_text_props(weight="bold")
        elif c % 2 == 0:
            cst.set_facecolor("#f7f9fb")
    ax.set_title(title, fontsize=11, pad=14)
    return _save(fig, out)


def plot_gate_progress(trace: dict, n_gates: int, out: Path,
                       title: str = "Gate traversal progress"):
    """Distance to the active gate and gate index vs time."""
    t = np.asarray(trace["t"], float)
    d = np.asarray(trace["dist_gate"], float)
    gi = np.asarray(trace["gate_idx"], float)
    fig, ax = plt.subplots(figsize=(8.4, 3.4))
    ax.plot(t, d, color=C_DRONE, lw=1.5, label="distance to active gate")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance [m]", color=C_DRONE)
    ax2 = ax.twinx()
    ax2.step(t, gi, color=C_GATE, lw=1.6, where="post", label="gates passed")
    ax2.set_ylabel("gates passed", color=C_GATE)
    ax2.set_ylim(0, n_gates + 0.4)
    ax2.grid(False)
    ax.set_title(title)
    return _save(fig, out)
