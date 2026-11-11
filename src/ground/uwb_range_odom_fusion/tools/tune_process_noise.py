"""Regenerate the process-noise tuning table quoted in the documentation.

    python tools/tune_process_noise.py

Prints one row per ``(process_sigma_v, process_sigma_omega)`` pair: mean position
RMSE, mean NEES and the two gating rates over the first five seeds.
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uwb_range_odom_fusion.runner import run_ekf  # noqa: E402
from uwb_range_odom_fusion.scenario import ScenarioConfig, simulate  # noqa: E402

GRID = (
    (0.02, 0.03),
    (0.05, 0.03),
    (0.05, 0.06),
    (0.05, 0.07),
    (0.05, 0.12),
    (0.10, 0.07),
)
SEEDS = range(5)


def main() -> int:
    print(f"{'sigma_v':>8}{'sigma_w':>9}{'RMSE [m]':>11}{'NEES':>8}{'false_rej%':>12}{'missed_nlos%':>14}")
    for sigma_v, sigma_omega in GRID:
        config = ScenarioConfig(
            duration_s=70.0,
            process_sigma_v=sigma_v,
            process_sigma_omega=sigma_omega,
        )
        rmse, nees, false_reject, missed = [], [], [], []
        for seed in SEEDS:
            simulation = simulate(config, seed=seed)
            metrics = run_ekf(simulation).metrics(simulation)
            rmse.append(metrics["position_m"]["rmse"])
            nees.append(metrics["nees"]["mean"])
            gating = metrics["gating"]
            false_reject.append(
                100.0 * gating["rejected_los"] / max(1, gating["los_total"])
            )
            missed.append(
                100.0 * gating["accepted_nlos"] / max(1, gating["nlos_total"])
            )
        print(
            f"{sigma_v:>8.2f}{sigma_omega:>9.2f}{np.mean(rmse):>11.4f}"
            f"{np.mean(nees):>8.2f}{np.mean(false_reject):>12.2f}{np.mean(missed):>14.2f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
