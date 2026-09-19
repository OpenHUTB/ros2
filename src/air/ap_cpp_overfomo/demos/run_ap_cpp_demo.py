#!/usr/bin/env python
"""End-to-end demo / verification harness for the AP-CPP module.

Runs the full active perception coverage loop without AirSim, without
TensorFlow and without GDAL, and writes a JSON mission report plus a set of
diagnostic figures.

Examples
--------
Run the self-contained synthetic field (no repository data needed)::

    python demos/run_ap_cpp_demo.py

Run against the real field shipped in the repository, using the pre-baked
``TurnWPs.txt`` route and ``Polygon002.geojson``::

    python demos/run_ap_cpp_demo.py --source geojson

Run the reference-only ablation side by side to quantify the benefit of the
active perception layer::

    python demos/run_ap_cpp_demo.py --compare
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import asdict

import numpy as np

# Make the repository root importable when the script is run from anywhere.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from ap_cpp.control import APCPPSpeedController  # noqa: E402
from ap_cpp.grid_model import CoverageGrid, GridConfig  # noqa: E402
from ap_cpp.mission import APCPPMission, MissionConfig  # noqa: E402
from ap_cpp.planner import PlannerConfig, RollingHorizonPlanner  # noqa: E402
from ap_cpp.pose import Pose  # noqa: E402
from ap_cpp.runtime import DemoPerceptionModel  # noqa: E402
from ap_cpp.sensor import SensorConfig, SensorModel  # noqa: E402
from ap_cpp.utility import UtilityModel, UtilityWeights  # noqa: E402


# --------------------------------------------------------------------------- #
# Scenario construction
# --------------------------------------------------------------------------- #
def synthetic_field():
    """An L-shaped field with a rectangular no-fly zone, in local NED metres.

    Deliberately non-convex so the lawnmower route has to skip a region, which
    is precisely the situation where an information-driven detour can pay off.
    """
    polygon = [
        (0.0, 0.0),
        (140.0, 0.0),
        (140.0, 60.0),
        (70.0, 60.0),
        (70.0, 130.0),
        (0.0, 130.0),
    ]
    obstacles = [
        [
            (95.0, 8.0),
            (128.0, 8.0),
            (128.0, 38.0),
            (95.0, 38.0),
        ]
    ]
    return polygon, obstacles


def route_budget(route, step_length):
    """Observation steps in exactly one pass over ``route``.

    ``MissionConfig.max_steps`` is a wall-clock safety valve, not a mission
    plan, but it silently becomes the plan when a route is much shorter than
    the field it covers.  A mission that outlives its reference route falls
    through to the fallback generators and flies on for hundreds of steps that
    have nothing to do with either configuration being compared - which turns
    an ablation into a comparison of two different missions.

    The reference route is never re-flown, so one pass is the budget: a route
    that runs out ends the mission rather than restarting it, and both arms of
    the comparison get the identical allowance.
    """
    if not route or len(route) < 2:
        return 1
    pts = np.array([p.xy for p in route], dtype=float)
    length = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
    return max(int(math.ceil(length / max(step_length, 1e-6))), 1)


def route_time_budget(max_steps, step_length, nominal_speed, q_max):
    """Wall-clock allowance generous enough to let ``max_steps`` bind first.

    ``max_time`` is a safety valve; if it fires before the step budget does,
    the two arms of an ablation stop at different step counts - the slower one
    (active perception) simply gets less done - and the comparison silently
    becomes "equal wall-clock" rather than "equal route".  Sized against the
    slowest speed the controller can command, so it cannot fire early.
    """
    v_min = max(float(nominal_speed) - float(q_max), 0.5)
    return max_steps * float(step_length) / v_min


def seed_prior(grid, kind="anomaly", weight=0.85, seed=0):
    """Overlay a non-uniform uncertainty prior on the belief.

    This is the layer that makes active perception worth doing.  A robot flying
    with no prior knowledge has nothing to be curious *about* - the gain of
    looking anywhere is the same, and the optimal policy collapses to the plain
    sweep.  Real missions never have an empty prior: last season's yield map,
    an NDVI anomaly from a coarser overflight, a grower's report of a problem
    patch, or simply the gaps left by the previous flight all concentrate
    uncertainty in a few places.

    ``kind="anomaly"`` reproduces that: a mostly-resolved field with one hot
    zone of unresolved ground.
    """
    if kind == "none":
        return None

    nx, ny = grid.nx, grid.ny
    centres = grid.cell_centers().reshape(nx, ny, 2)
    x, y = centres[:, :, 0], centres[:, :, 1]

    span_x = max(grid.config.x_max - grid.config.x_min, 1e-6)
    span_y = max(grid.config.y_max - grid.config.y_min, 1e-6)
    u = (x - grid.config.x_min) / span_x
    v = (y - grid.config.y_min) / span_y

    if kind == "anomaly":
        # One agronomic hot zone in a corner of the field.
        blob = np.exp(-(((u - 0.72) ** 2 + (v - 0.28) ** 2) / (2 * 0.16 ** 2)))
    elif kind == "two_zones":
        blob = np.exp(-(((u - 0.25) ** 2 + (v - 0.70) ** 2) / (2 * 0.11 ** 2)))
        blob = np.maximum(blob, np.exp(-(((u - 0.78) ** 2 + (v - 0.30) ** 2) / (2 * 0.09 ** 2))))
    else:
        raise ValueError("unknown prior kind: {}".format(kind))

    grid.set_uncertainty_prior(np.clip(blob, 0.0, 1.0), weight=weight)
    return blob


def build_grid(args):
    """Assemble the belief raster for the requested scenario."""
    if args.source == "geojson":
        from ap_cpp.geo_bridge import GeoBridge, load_qgis_polygon, load_waypoints_txt

        geojson = os.path.join(_REPO_ROOT, "CPP", args.field, "Polygon{}.geojson".format(args.field))
        route_file = os.path.join(_REPO_ROOT, "CPP", args.field, "TurnWPs.txt")
        if not os.path.exists(geojson):
            raise SystemExit("missing scenario file: {}".format(geojson))

        polygon_wgs84, obstacles_wgs84, name = load_qgis_polygon(geojson)
        bridge = GeoBridge(polygon_wgs84, obstacles_wgs84)
        grid = bridge.build_grid(resolution=args.resolution)

        route_ned = None
        if os.path.exists(route_file):
            route_wgs84 = load_waypoints_txt(route_file)
            route_ned = bridge.wgs84_to_ned(route_wgs84)

        if args.prior != "none":
            seed_prior(grid, args.prior, args.prior_weight, args.seed)
        print("[scenario] QGIS field '{}' ({} vertices, {} obstacles)".format(
            name, len(polygon_wgs84), len(obstacles_wgs84)))
        print("[scenario] NED origin at WGS84 {}".format(bridge.describe()["origin_wgs84"]))
        return grid, bridge, route_ned

    polygon, obstacles = synthetic_field()
    cfg = GridConfig.from_polygon(polygon, resolution=args.resolution)
    grid = CoverageGrid(cfg)
    grid.rasterize_polygon(polygon)
    grid.rasterize_obstacles(obstacles)
    if args.prior != "none":
        seed_prior(grid, args.prior, args.prior_weight, args.seed)
    print("[scenario] synthetic L-shaped field, {} x {} cells @ {:.1f} m".format(
        grid.nx, grid.ny, args.resolution))
    return grid, None, None


# --------------------------------------------------------------------------- #
# Mission assembly
# --------------------------------------------------------------------------- #
def build_mission(
    args, grid, reference_route, active=True, seed=7, max_steps=None, max_time=None
):
    """Wire up sensor -> utility -> planner -> perception -> controller.

    Returns ``(mission, sensor_config, max_steps)``; ``max_steps`` is the step
    budget the mission was actually built with, which may have been derived
    from the route rather than taken from ``--max-steps``.
    """
    if max_steps is None:
        max_steps = mission_budget(args, reference_route)
    if max_time is None:
        max_time = (
            float(args.max_time)
            if args.max_time is not None
            else route_time_budget(
                max_steps, args.step_length, args.nominal_speed, args.q_max
            )
        )
    sensor_cfg = SensorConfig(altitude=args.altitude)
    sensor_cfg.coverage_gain = args.coverage_gain
    sensor = SensorModel(sensor_cfg)

    if reference_route is None:
        lane_spacing = sensor_cfg.ground_width * (1.0 - args.sidelap / 100.0)
        reference_route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane_spacing)

    weights = UtilityWeights()
    utility = UtilityModel(weights=weights, sensor=sensor)

    planner_cfg = PlannerConfig(
        horizon=args.horizon,
        execute_steps=args.execute_steps,
        step_length=args.step_length,
        frontier_targets=args.frontier_targets if active else 0,
        fan_offsets_deg=(-60, -30, 0, 30, 60) if active else (),
        entropy_bias=args.entropy_bias if active else 0.0,
        # The reference-only ablation must be exactly that: no excursions, no
        # fan, and no hole patching.  Leaving coverage repair on would let the
        # baseline keep flying after the reference route is spent and bank
        # coverage that the published sweep never performs.
        enable_coverage_repair=active,
    )
    planner = RollingHorizonPlanner(grid, sensor=sensor, utility=utility, config=planner_cfg)

    perception = DemoPerceptionModel(grid, sensor, seed=seed)

    controller = APCPPSpeedController(
        nominal_speed=args.nominal_speed,
        q_max=args.q_max,
        gain_weight=args.gain_weight,
    )

    mission_cfg = MissionConfig(
        coverage_target=args.coverage_target,
        entropy_target=args.entropy_target,
        max_steps=max_steps,
        max_time=max_time,
    )

    return APCPPMission(
        grid=grid,
        planner=planner,
        perception=perception,
        controller=controller,
        reference_route=reference_route,
        config=mission_cfg,
        sensor=sensor,
    ), sensor_cfg, mission_cfg.max_steps


def mission_budget(args, reference_route):
    """Resolve ``--max-steps``: explicit wins, otherwise derive from the route.

    Defaulting to the route length rather than a fixed 400 steps is what keeps
    both arms of the ablation on the same mission.  A fixed budget is fine on
    the synthetic field, whose route spans the whole raster, but the shipped
    ``TurnWPs.txt`` routes are far shorter than the field they cover: field 002
    is 591.6 m, i.e. 99 observation steps at the default 6 m spacing, so a
    400-step budget flies 301 steps past the end of the reference route.
    """
    if args.max_steps is not None:
        return int(args.max_steps)
    return route_budget(reference_route, args.step_length)


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #
def print_progress(step, total_steps):
    if step.index == 0 or (step.index + 1) % 10 == 0:
        print(
            "  step {:3d} | {:<18s} | H {:.4f} -> {:.4f} | cov {:.3f} | "
            "IG {:.4f} | v {:.2f} m/s".format(
                step.index + 1,
                step.operator,
                step.entropy_before,
                step.entropy_after,
                step.coverage_fraction,
                step.information_gain,
                step.speed,
            )
        )


def summarise(label, mission, grid, elapsed):
    summary = mission.record.final_summary(grid)
    print("\n=== {} ===".format(label))
    print("  termination            : {}".format(summary["termination"]))
    print("  observation steps      : {}".format(summary["steps"]))
    print("  replans                : {}".format(summary["replans"]))
    print("  flight distance        : {:.1f} m".format(summary["total_distance_m"]))
    print("  simulated duration     : {:.1f} s".format(summary["duration_s"]))
    print("  mean ground speed      : {:.2f} m/s".format(summary["mean_speed"]))
    print("  mean belief entropy    : {:.4f} (was {:.4f})".format(
        summary["mean_entropy"], grid.config.initial_entropy))
    print("  coverage fraction      : {:.4f}".format(summary["coverage_fraction"]))
    print("  cumulative information : {:.3f}".format(summary["cumulative_information_gain"]))
    print("  operator histogram     : {}".format(mission.record.operator_histogram()))
    print("  wall-clock             : {:.2f} s".format(elapsed))
    return summary


def save_figures(out_dir, grid, mission, label, snapshot_before, truth=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - plotting is best-effort
        print("[warn] matplotlib unavailable, skipping figures ({})".format(exc))
        return None

    steps = mission.record.steps
    if not steps:
        return None

    xs = [s.pose.x for s in steps]
    ys = [s.pose.y for s in steps]

    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    fig.suptitle("AP-CPP mission - {}".format(label), fontsize=13)

    extent = (
        grid.config.x_min,
        grid.config.x_max,
        grid.config.y_min,
        grid.config.y_max,
    )

    def field(array):
        """Mask non-traversable cells so obstacles and off-field ground render
        as grey rather than as 'fully covered' / 'fully resolved'."""
        masked = np.ma.masked_array(array, mask=~grid.passable)
        masked.set_fill_value(np.nan)
        return masked

    mask_cmap = plt.get_cmap("magma").copy()
    mask_cmap.set_bad("#d9d9d9", alpha=1.0)

    ax = axes[0, 0]
    im = ax.imshow(field(snapshot_before["entropy"]).T, origin="lower", extent=extent,
                   cmap=mask_cmap, vmin=0, vmax=1)
    ax.set_title("Belief entropy - before")
    fig.colorbar(im, ax=ax, fraction=0.046)

    ax = axes[0, 1]
    im = ax.imshow(field(grid.entropy).T, origin="lower", extent=extent,
                   cmap=mask_cmap, vmin=0, vmax=1)
    ax.plot(xs, ys, "-", color="#1f77b4", lw=1.2, label="flight path")
    ax.plot(xs[0], ys[0], "o", color="lime", ms=7, label="start")
    ax.set_title("Belief entropy - after")
    ax.legend(loc="upper right", fontsize=8)
    fig.colorbar(im, ax=ax, fraction=0.046)

    ax = axes[1, 0]
    cov_cmap = plt.get_cmap("viridis").copy()
    cov_cmap.set_bad("#d9d9d9", alpha=1.0)
    im = ax.imshow(field(grid.coverage).T, origin="lower", extent=extent,
                   cmap=cov_cmap, vmin=0, vmax=1)
    ax.plot(xs, ys, "-", color="white", lw=1.0, alpha=0.85)
    ax.set_title("Accumulated coverage")
    fig.colorbar(im, ax=ax, fraction=0.046)

    ax = axes[1, 1]
    ax.plot([s.index for s in steps], [s.entropy_after for s in steps], label="mean entropy")
    ax.plot(
        [s.index for s in steps],
        [s.coverage_fraction for s in steps],
        label="coverage fraction",
    )
    ax.set_xlabel("observation step")
    ax.set_ylabel("normalised")
    ax.set_title("Mission convergence")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax2 = ax.twinx()
    ax2.bar(
        [s.index for s in steps],
        [s.information_gain for s in steps],
        color="#d62728",
        alpha=0.25,
        label="per-step information gain",
    )
    ax2.set_ylabel("information gain")

    for ax in axes.ravel():
        ax.set_xlabel("north [m]") if ax in (axes[0, 0], axes[0, 1], axes[1, 0]) else None
        ax.set_ylabel("east [m]") if ax in (axes[0, 0], axes[0, 1], axes[1, 0]) else None

    fig.tight_layout()
    path = os.path.join(out_dir, "ap_cpp_{}.png".format(label))
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print("[figure] {}".format(path))
    return path


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=("synthetic", "geojson"), default="synthetic",
                   help="field geometry source (default: synthetic)")
    p.add_argument("--field", default="002", help="field id for --source geojson")
    p.add_argument("--resolution", type=float, default=2.5, help="raster cell size [m]")
    p.add_argument("--altitude", type=float, default=10.0, help="flight altitude [m]")
    p.add_argument("--sidelap", type=float, default=70.0, help="side overlap [%%]")
    p.add_argument("--coverage-gain", type=float, default=0.6,
                   help="coverage credited per observation")
    p.add_argument("--horizon", type=int, default=6, help="rolling horizon length")
    p.add_argument("--execute-steps", type=int, default=2,
                   help="poses committed before re-planning")
    p.add_argument("--step-length", type=float, default=6.0,
                   help="spacing between observation poses [m]")
    p.add_argument("--frontier-targets", type=int, default=3,
                   help="number of uncertainty-frontier A* excursions per replan")
    p.add_argument("--entropy-bias", type=float, default=0.55,
                   help="uncertainty discount in the A* edge cost")
    p.add_argument("--nominal-speed", type=float, default=3.0, help="nominal speed [m/s]")
    p.add_argument("--q-max", type=float, default=2.0, help="speed authority [m/s]")
    p.add_argument("--gain-weight", type=float, default=0.35,
                   help="weight of the active-perception term in the speed law")
    p.add_argument("--coverage-target", type=float, default=0.85)
    p.add_argument("--entropy-target", type=float, default=0.25)
    p.add_argument("--max-steps", type=int, default=None,
                   help="step budget; default is the observation-step count of one "
                        "pass over the reference route (route length / step length)")
    p.add_argument("--max-time", type=float, default=None,
                   help="wall-clock budget in s; default is derived from the step "
                        "budget so that max_steps binds first")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--prior", choices=("none", "anomaly", "two_zones"), default="anomaly",
                   help="non-uniform uncertainty prior to seed the belief with")
    p.add_argument("--prior-weight", type=float, default=0.85,
                   help="blend weight of the prior, in [0, 1]")
    p.add_argument("--output-dir", default=os.path.join(_REPO_ROOT, "results", "ap_cpp_demo"))
    p.add_argument("--no-plots", action="store_true", help="skip figure generation")
    p.add_argument("--compare", action="store_true",
                   help="also run the reference-only ablation")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 78)
    print(" Active Perception Coverage Path Planning (AP-CPP) - demo run")
    print("=" * 78)

    grid, bridge, route_ned = build_grid(args)

    reference_route = None
    if route_ned is not None and len(route_ned) > 1:
        reference_route = _clip_route_to_grid(_poses_from_route(route_ned), grid)
    if not reference_route:
        sensor_tmp = SensorModel(SensorConfig(altitude=args.altitude))
        lane_spacing = sensor_tmp.config.ground_width * (1.0 - args.sidelap / 100.0)
        reference_route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane_spacing)
    print("[route] reference sweep: {} observation poses".format(len(reference_route)))

    start_pose = reference_route[0] if reference_route else Pose(
        *grid.cell_center(*grid.free_cells()[0]), 0.0
    )

    max_steps = mission_budget(args, reference_route)
    max_time = args.max_time if args.max_time is not None else route_time_budget(
        max_steps, args.step_length, args.nominal_speed, args.q_max
    )
    print("[budget] max_steps = {} ({})  max_time = {:.0f} s".format(
        max_steps,
        "--max-steps" if args.max_steps is not None else "one pass over the reference route",
        max_time,
    ))

    # ---------------------------------------------------------------- active
    grid_active, _, _ = build_grid(args)
    mission, sensor_cfg, _ = build_mission(
        args, grid_active, reference_route, active=True, seed=args.seed,
        max_steps=max_steps, max_time=max_time,
    )
    snapshot_before = grid_active.snapshot()

    print("\n[sensor] footprint {:.1f} x {:.1f} m, sensing radius {:.1f} m".format(
        sensor_cfg.ground_width, sensor_cfg.ground_height, sensor_cfg.max_ground_radius))

    t0 = time.perf_counter()
    for step in mission.step(start_pose):
        print_progress(step, max_steps)
    elapsed = time.perf_counter() - t0

    active_summary = summarise("AP-CPP (active perception)", mission, grid_active, elapsed)
    figure = None
    if not args.no_plots:
        figure = save_figures(args.output_dir, grid_active, mission, "active", snapshot_before)

    report = {
        "config": {k: v for k, v in vars(args).items()},
        "sensor": asdict(sensor_cfg),
        "active": active_summary,
        "operator_histogram": mission.record.operator_histogram(),
        "steps": [s.as_dict() for s in mission.record.steps],
    }

    # -------------------------------------------------------------- ablation
    if args.compare:
        grid_base, _, _ = build_grid(args)
        baseline, _, _ = build_mission(
            args, grid_base, reference_route, active=False, seed=args.seed,
            max_steps=max_steps, max_time=max_time,
        )
        t0 = time.perf_counter()
        for _ in baseline.step(start_pose):
            pass
        base_elapsed = time.perf_counter() - t0
        base_summary = summarise("Reference-only ablation (baseline)", baseline, grid_base, base_elapsed)
        report["baseline"] = base_summary
        report["config"]["max_steps"] = max_steps
        if not args.no_plots:
            save_figures(args.output_dir, grid_base, baseline, "baseline", grid_base.snapshot())
        _print_comparison(active_summary, base_summary)

    report_path = os.path.join(args.output_dir, "ap_cpp_report.json")
    with open(report_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2, default=_json_default)
    print("\n[report] {}".format(report_path))
    if figure:
        print("[figure] {}".format(figure))

    print("\nDemo completed successfully.")
    return 0


def _json_default(obj):
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, tuple):
        return list(obj)
    return str(obj)


def _poses_from_route(route_ned):
    """Turn an ``(N, 2)`` NED polyline into poses with direction-following yaw."""
    poses = []
    pts = [(float(p[0]), float(p[1])) for p in route_ned]
    for k, (x, y) in enumerate(pts):
        if k + 1 < len(pts):
            dx, dy = pts[k + 1][0] - x, pts[k + 1][1] - y
        elif k > 0:
            dx, dy = x - pts[k - 1][0], y - pts[k - 1][1]
        else:
            dx, dy = 1.0, 0.0
        yaw = math.degrees(math.atan2(dy, dx)) if (dx or dy) else 0.0
        poses.append(Pose(x, y, yaw))
    return poses


def _clip_route_to_grid(poses, grid):
    """Drop reference waypoints that are not traversable or not on the map.

    The shipped ``TurnWPs.txt`` files cover the full field with no knowledge of
    the rasterised obstacles, so a couple of entries can land on a no-fly zone.
    """
    clipped = []
    for p in poses:
        cell = grid.nearest_free_cell(p.x, p.y)
        if cell is None:
            continue
        cx, cy = grid.cell_center(*cell)
        if math.hypot(cx - p.x, cy - p.y) > grid.config.resolution * 1.5:
            continue
        clipped.append(Pose(cx, cy, p.yaw_deg))
    return clipped


def _print_comparison(active, baseline):
    print("\n=== Active perception vs reference-only ablation ===")
    header = "{:<26s} {:>12s} {:>12s} {:>10s}".format("metric", "active", "reference", "delta")
    print(header)
    print("-" * len(header))
    rows = [
        ("flight distance [m]", "total_distance_m", "{:.1f}"),
        ("duration [s]", "duration_s", "{:.1f}"),
        ("observation steps", "steps", "{:.0f}"),
        ("mean entropy", "mean_entropy", "{:.4f}"),
        ("coverage fraction", "coverage_fraction", "{:.4f}"),
        ("information gain", "cumulative_information_gain", "{:.3f}"),
    ]
    for label, key, fmt in rows:
        a = float(active.get(key, 0.0))
        b = float(baseline.get(key, 0.0))
        print("{:<26s} {:>12s} {:>12s} {:>12s}".format(
            label,
            fmt.format(a),
            fmt.format(b),
            ("+" if a - b >= 0 else "-") + fmt.format(abs(a - b)),
        ))


if __name__ == "__main__":
    raise SystemExit(main())
