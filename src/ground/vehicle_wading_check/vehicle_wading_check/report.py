"""Static report: allowed wading speed and buoyancy margin against water depth."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .core import WadingChecker, WadingConfig, float_depth  # noqa: E402


def build_report(
    config: WadingConfig,
    out_path: Path,
    max_depth_m: float = 0.60,
) -> dict[str, object]:
    checker = WadingChecker(config)
    sweep = checker.sweep(max_depth_m)
    depths = [row.depth_m for row in sweep]
    v_max_kmh = [row.v_max_mps * 3.6 for row in sweep]
    buoyancy_pct = [row.buoyancy_ratio * 100.0 for row in sweep]
    traction_kn = [row.traction_limit_n / 1000.0 for row in sweep]

    depth_limit = sweep[0].depth_limit_m
    safe_depth = sweep[0].safe_depth_m
    float_d = float_depth(config)
    v_at_safe = next(
        row.v_max_mps * 3.6 for row in sweep if row.depth_m >= safe_depth
    )

    fig, (ax_speed, ax_force) = plt.subplots(
        1, 2, figsize=(11.6, 4.7), dpi=150, constrained_layout=True
    )

    ax_speed.plot(depths, v_max_kmh, color="#1f4e79", linewidth=2.0,
                  label="allowed wading speed")
    ax_speed.axvline(safe_depth, color="#2e7d32", linestyle="--", linewidth=1.6,
                     label=f"safe depth {safe_depth:.2f} m")
    ax_speed.axvline(depth_limit, color="#c62828", linestyle="--", linewidth=1.6,
                     label=f"depth limit {depth_limit:.2f} m")
    ax_speed.axvline(float_d, color="#6a1b9a", linestyle=":", linewidth=1.6,
                     label=f"flotation {float_d:.2f} m")
    ax_speed.set_xlabel("water depth [m]")
    ax_speed.set_ylabel("allowed speed [km/h]")
    ax_speed.set_title("Allowed wading speed vs water depth")
    ax_speed.grid(alpha=0.3)
    ax_speed.legend(fontsize=8, loc="upper right")
    ax_speed.set_xlim(0.0, max_depth_m)
    ax_speed.set_ylim(0.0, max(v_max_kmh) * 1.15)

    ax_trac = ax_force.twinx()
    buoy_line, = ax_force.plot(depths, buoyancy_pct, color="#ef6c00", linewidth=2.0,
                               label="buoyancy / weight [%]")
    trac_line, = ax_trac.plot(depths, traction_kn, color="#00695c", linewidth=2.0,
                              label="wet traction limit [kN]")
    ax_force.set_xlim(0.0, max_depth_m)
    ax_force.set_xlabel("water depth [m]")
    ax_force.set_ylabel("buoyancy / weight [%]", color="#ef6c00")
    ax_trac.set_ylabel("wet traction limit [kN]", color="#00695c")
    ax_force.set_title("Buoyancy and traction limit vs water depth")
    ax_force.grid(alpha=0.3)
    ax_force.set_ylim(0.0, max(buoyancy_pct) * 1.25)
    ax_trac.set_ylim(0.0, max(traction_kn) * 1.45)
    safe_line = ax_force.axvline(safe_depth, color="#2e7d32", linestyle="--",
                                 linewidth=1.6,
                                 label=f"safe depth {safe_depth:.2f} m")
    float_line = ax_force.axvline(float_d, color="#6a1b9a", linestyle=":",
                                  linewidth=1.6,
                                  label=f"flotation {float_d:.2f} m")
    limit_line = ax_force.axhline(config.float_ratio_limit * 100.0, color="#6a1b9a",
                                  linewidth=1.0, alpha=0.6,
                                  label=f"flotation limit {config.float_ratio_limit * 100.0:.0f} %")
    handles = [buoy_line, trac_line, limit_line, safe_line, float_line]
    ax_force.legend(handles, [line.get_label() for line in handles],
                    fontsize=8, loc="upper left")

    fig.suptitle(
        "vehicle_wading_check: "
        f"mass {config.mass_kg:.0f} kg, mu_wet {config.friction_coef_wet:.2f}, "
        f"allowed speed at safe depth {v_at_safe:.1f} km/h",
        fontsize=9,
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)

    return {
        "out_path": str(out_path),
        "depth_limit_m": round(depth_limit, 3),
        "safe_depth_m": round(safe_depth, 3),
        "float_depth_m": round(float_d, 3),
        "v_max_kmh_at_zero": round(v_max_kmh[0], 2),
        "v_max_kmh_at_safe": round(v_at_safe, 2),
        "buoyancy_at_safe_pct": round(
            next(row.buoyancy_ratio for row in sweep if row.depth_m >= safe_depth) * 100.0,
            2,
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Render the wading report PNG")
    parser.add_argument("--out", type=Path, default=Path("wading_report.png"))
    parser.add_argument("--max-depth", type=float, default=0.60)
    args = parser.parse_args()
    info = build_report(WadingConfig(), args.out, args.max_depth)
    for key, value in info.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
