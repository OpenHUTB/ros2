"""Matplotlib figures for the report (optional dependency).

The module runs headless: the ``Agg`` backend is selected before ``pyplot`` is
imported so that figures can be generated on a server or inside a container.
Every figure is drawn from the same simulated run that produces the numbers in
``results/summary.json``.
"""

from __future__ import annotations

import numpy as np

try:  # pragma: no cover - exercised only when matplotlib is installed
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle, Ellipse

    HAVE_MATPLOTLIB = True
except Exception:  # pragma: no cover - numpy-only environments
    HAVE_MATPLOTLIB = False


COLORS = {
    "truth": "#111111",
    "odometry_only": "#ef8a3c",
    "trilateration": "#2c6fbb",
    "ekf_ungated": "#c0392b",
    "ekf_gated": "#1e8449",
}

LABELS = {
    "odometry_only": "wheel odometry",
    "trilateration": "per-epoch trilateration",
    "ekf_ungated": "EKF, no gating",
    "ekf_gated": "EKF + chi-square gating",
}


def _require_matplotlib() -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError(
            "matplotlib is required for figures: pip install matplotlib "
            "(or run with --no-plots)"
        )


def _style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 150,
            "font.size": 9,
            "axes.grid": True,
            "grid.alpha": 0.3,
            "axes.axisbelow": True,
        }
    )


def _draw_field(ax, simulation, label_path: bool = True) -> None:
    config = simulation.config
    path = simulation.path
    if label_path:
        ax.plot(path.points[:, 0], path.points[:, 1], "--", color="#999999", lw=1.0,
                label="reference path")
    else:
        ax.plot(path.points[:, 0], path.points[:, 1], "--", color="#999999", lw=1.0)
    for index, (x, y) in enumerate(config.anchor_array):
        ax.plot(x, y, "s", color="#222222", ms=7, mec="white", mew=0.8)
        ax.annotate(f"A{index}", (x, y), textcoords="offset points", xytext=(6, 5),
                    fontsize=8)
    for px, py, radius in config.pillars:
        ax.add_patch(Circle((px, py), radius, color="#b0b0b0", alpha=0.6, zorder=1))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-config.half_width - 2.6, config.half_width + 2.6)
    ax.set_ylim(-config.half_height - 2.4, config.half_height + 2.4)


def _blocked_window(simulation, half_window_s: float = 0.3) -> np.ndarray:
    """Epochs where at least one range is blocked, widened by half a window."""
    blocked = np.any(simulation.nlos, axis=1).astype(float)
    width = max(1, int(round(2.0 * half_window_s * simulation.config.control_hz)))
    kernel = np.ones(width) / width
    smoothed = np.convolve(blocked, kernel, mode="same")
    return smoothed > 1e-6


def plot_trajectories(simulation, runs, output_path) -> None:
    """Ground truth against every estimator on the same run."""
    _require_matplotlib()
    _style()
    figure, ax = plt.subplots(figsize=(7.2, 5.2))
    _draw_field(ax, simulation)
    ax.plot(simulation.truth[:, 0], simulation.truth[:, 1], "-", color=COLORS["truth"],
            lw=3.4, alpha=0.32, label="ground truth", zorder=2)
    for run in runs:
        ax.plot(run.positions[:, 0], run.positions[:, 1], "-",
                color=COLORS.get(run.name, None), lw=1.1, label=LABELS.get(run.name, run.name))
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3, frameon=False)
    ax.set_title(f"Trajectories, {simulation.config.duration_s:.0f} s run (seed 0)")
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_errors(simulation, runs, output_path) -> None:
    """Position error over time with non-line-of-sight episodes shaded."""
    _require_matplotlib()
    _style()
    times = simulation.times
    figure, ax = plt.subplots(figsize=(7.2, 3.6))
    blocked = _blocked_window(simulation)
    edges = np.diff(np.concatenate([[0], blocked.astype(int), [0]]))
    starts = np.flatnonzero(edges == 1)
    stops = np.flatnonzero(edges == -1)
    for index, (start, stop) in enumerate(zip(starts, stops, strict=True)):
        ax.axvspan(times[start], times[min(stop, len(times) - 1)], color="#f2c9c9",
                   alpha=0.5, lw=0, label="NLOS episode" if index == 0 else None)
    for run in runs:
        error = np.linalg.norm(run.positions - simulation.truth[:, :2], axis=1)
        ax.semilogy(times, np.maximum(error, 1e-4), color=COLORS.get(run.name, None),
                    lw=1.2, label=LABELS.get(run.name, run.name))
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error [m]")
    ax.set_title("Position error versus time (same run, seed 0)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3, frameon=False)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_gdop(config_square, config_collinear, output_path) -> None:
    """GDOP of a well-spread and of a collinear anchor layout."""
    _require_matplotlib()
    from .geometry import gdop_map

    _style()
    figure, axes = plt.subplots(1, 2, figsize=(9.0, 3.8), sharey=True)
    xs = np.arange(-7.5, 7.51, 0.25)
    ys = np.arange(-5.5, 5.51, 0.25)
    plotted = None
    for ax, config, title in (
        (axes[0], config_square, "corner layout"),
        (axes[1], config_collinear, "collinear layout (y = 0.2 m)"),
    ):
        grid = gdop_map(xs, ys, config.anchor_array)
        plotted = ax.pcolormesh(xs, ys, np.clip(grid, 0.0, 8.0), shading="auto",
                                cmap="viridis", vmin=1.0, vmax=8.0)
        if np.any(np.isinf(grid)):
            masked = np.ma.masked_where(~np.isinf(grid), np.ones_like(grid))
            ax.pcolormesh(xs, ys, masked, shading="auto", cmap="Reds", vmin=0, vmax=1)
        for x, y in config.anchor_array:
            ax.plot(x, y, "s", color="white", mec="black", ms=6)
        ax.set_title(f"{title}\nmean GDOP inside the loop = {_loop_mean(grid, xs, ys):.2f}")
        ax.set_xlabel("x [m]")
        ax.set_aspect("equal", adjustable="box")
    axes[0].set_ylabel("y [m]")
    figure.colorbar(plotted, ax=axes, shrink=0.85, label="GDOP (clipped at 8)")
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def _loop_mean(grid, xs, ys) -> float:
    mask = (np.abs(xs)[None, :] <= 4.5) & (np.abs(ys)[:, None] <= 2.8)
    values = grid[mask]
    values = values[np.isfinite(values)]
    return float(np.mean(values)) if values.size else float("inf")


def plot_nees(simulation, run, output_path) -> None:
    """NEES against the 95 % band of a 3-DoF chi-square variable."""
    _require_matplotlib()
    from .metrics import chi2_quantile

    _style()
    figure, ax = plt.subplots(figsize=(7.2, 3.2))
    if run.nees is not None:
        values = run.nees[np.isfinite(run.nees)]
        ax.plot(simulation.times[: len(values)], values, color=COLORS["ekf_gated"], lw=0.7,
                label="NEES per epoch")
        ax.axhline(float(np.mean(values)), color=COLORS["ekf_gated"], ls=":", lw=1.0,
                   label=f"mean = {np.mean(values):.2f}")
    ax.axhline(3.0, color="#333333", lw=1.0, label="E[NEES] = 3")
    ax.axhline(chi2_quantile(3, 0.975), color="#777777", ls="--", lw=0.9,
               label="95 % quantile of chi2(3)")
    ax.set_yscale("log")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("NEES")
    ax.set_title("Filter consistency of the gated EKF")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=4, frameon=False)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def animate(
    simulation, run, output_path, fps: float = 6.0, seconds: float | None = None
) -> None:
    """Animated GIF: estimate, uncertainty ellipse and accepted/rejected ranges."""
    _require_matplotlib()
    _style()
    config = simulation.config
    times = simulation.times
    if seconds is None:
        seconds = float(times[-1])
    indices = np.flatnonzero(times <= seconds)
    stride = max(1, int(round(config.control_hz / fps)))
    indices = indices[::stride]
    figure, ax = plt.subplots(figsize=(4.8, 3.9))
    _draw_field(ax, simulation, label_path=False)
    links = []
    for _ in config.anchor_array:
        line, = ax.plot([], [], "-", lw=0.8, color="#bbbbbb", zorder=0)
        links.append(line)
    truth_line, = ax.plot([], [], "-", color=COLORS["truth"], lw=1.4, label="ground truth")
    estimate_line, = ax.plot([], [], "-", color=COLORS["ekf_gated"], lw=1.6,
                             label="EKF + gating")
    covariance_patch = Ellipse((0, 0), 0.2, 0.2, fill=False, color=COLORS["ekf_gated"],
                               lw=1.0, alpha=0.9)
    ax.add_patch(covariance_patch)
    dot, = ax.plot([], [], "o", color=COLORS["ekf_gated"], ms=5, zorder=5)
    title = ax.set_title("")
    ax.legend(loc="upper right", frameon=False, fontsize=7)

    def update(frame_index):
        index = int(indices[frame_index])
        history = indices[: frame_index + 1]
        truth_line.set_data(simulation.truth[history, 0], simulation.truth[history, 1])
        estimate_line.set_data(run.positions[history, 0], run.positions[history, 1])
        dot.set_data([run.positions[index, 0]], [run.positions[index, 1]])
        for anchor_index, line in enumerate(links):
            line.set_data(
                [run.positions[index, 0], config.anchor_array[anchor_index, 0]],
                [run.positions[index, 1], config.anchor_array[anchor_index, 1]],
            )
            if run.accepted is not None and run.rejected is not None:
                if run.rejected[index, anchor_index]:
                    line.set_color("#c0392b")
                    line.set_linewidth(1.6)
                elif run.accepted[index, anchor_index]:
                    line.set_color("#9fd3a8")
                    line.set_linewidth(0.9)
                else:
                    line.set_color("#dddddd")
                    line.set_linewidth(0.6)
        if run.covariance is not None:
            block = run.covariance[index][:2, :2]
            values, vectors = np.linalg.eigh(block)
            values = np.maximum(values, 1e-6)
            angle = float(np.degrees(np.arctan2(vectors[1, 1], vectors[0, 1])))
            covariance_patch.set_center(run.positions[index])
            covariance_patch.width = 4.0 * float(np.sqrt(values[1]))
            covariance_patch.height = 4.0 * float(np.sqrt(values[0]))
            covariance_patch.angle = angle
        title.set_text(f"t = {times[index]:5.1f} s")
        return [truth_line, estimate_line, dot, covariance_patch, title, *links]

    animation = FuncAnimation(figure, update, frames=len(indices), interval=1000.0 / fps,
                              blit=False)
    animation.save(output_path, writer=PillowWriter(fps=fps))
    plt.close(figure)
