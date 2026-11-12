"""Figures for the MPC tracking report."""

from __future__ import annotations

import numpy as np

try:  # pragma: no cover
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAVE_MATPLOTLIB = True
except Exception:  # pragma: no cover
    HAVE_MATPLOTLIB = False


def _style() -> None:
    plt.rcParams.update(
        {"figure.dpi": 150, "font.size": 9, "axes.grid": True, "grid.alpha": 0.3}
    )


def plot_tracking(path, mpc_run, pursuit_run, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    figure, axes = plt.subplots(1, 2, figsize=(9.0, 3.8))
    axes[0].plot(path.x, path.y, "--", color="#999999", lw=1.0, label="reference")
    axes[0].plot(pursuit_run["states"][:, 0], pursuit_run["states"][:, 1], color="#c0392b",
                 lw=1.2, label="pure pursuit")
    axes[0].plot(mpc_run["states"][:, 0], mpc_run["states"][:, 1], color="#1e8449",
                 lw=1.4, label="MPC")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_title("Path tracking")
    axes[0].legend(frameon=False, fontsize=8)

    time_s = mpc_run["time_s"]
    axes[1].plot(time_s, _errors(path, pursuit_run), color="#c0392b", lw=0.9,
                 label="pure pursuit")
    axes[1].plot(time_s, _errors(path, mpc_run), color="#1e8449", lw=0.9, label="MPC")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("lateral error [m]")
    axes[1].set_title("Lateral error")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def _errors(path, run):
    errors = []
    for state in run["states"]:
        nearest = path.project(state[:2], 0.0, window=0.0)
        sample, _ = path.sample(nearest)
        errors.append(float(np.hypot(*(state[:2] - sample[0][:2]))))
    return np.asarray(errors)


def plot_controls(mpc_run, pursuit_run, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    figure, axes = plt.subplots(2, 1, figsize=(7.2, 4.6), sharex=True)
    time_s = mpc_run["time_s"]
    axes[0].plot(time_s, pursuit_run["controls"][:, 1], color="#c0392b", lw=0.9,
                 label="pure pursuit")
    axes[0].plot(time_s, mpc_run["controls"][:, 1], color="#1e8449", lw=1.2, label="MPC")
    axes[0].set_ylabel("yaw rate [rad/s]")
    axes[0].legend(frameon=False, fontsize=8)
    axes[0].set_title("Control effort")
    axes[1].plot(time_s, mpc_run["controls"][:, 0], color="#1e8449", lw=1.2)
    axes[1].axhline(1.0, color="#333333", ls="--", lw=0.8, label="speed limit")
    axes[1].set_ylabel("speed [m/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
