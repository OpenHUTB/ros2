#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 visualisation bridge for the AP-CPP planner.

Runs an AP-CPP coverage mission **headlessly** &mdash; against the same synthetic
field or shipped ``Polygon002.geojson`` the offline demo uses, and with the same
planner, belief grid and perception stub &mdash; then publishes the result for
RViz:

* ``~path``     (``nav_msgs/Path``)              the flown observation path
* ``~markers``  (``visualization_msgs/MarkerArray``)
      - every passable cell, coloured by accumulated coverage
      - every non-traversable cell, in flat grey
      - the reference sweep (yellow line strip)
      - the take-off point and the final pose
      - a text label with the mission summary

Neither the node nor the launch file needs AirSim: it is a pure
planning-and-visualisation loop, so ``rviz_demo.launch`` works on any machine
with ROS and NumPy.

Frames
------
AP-CPP plans in NED metres (x north, y east, z down).  RViz wants ENU, so every
point is mapped

    ros.x = ned.y      ros.y = ned.x      ros.z = -ned.z

which for a constant-altitude mission puts the field in the z = +altitude plane
with north up the +y axis.  Pass ``--no-frame-swap`` to publish raw NED instead.

Usage
-----
    rosrun ap_cpp_ros ap_cpp_rviz_node.py
    rosrun ap_cpp_ros ap_cpp_rviz_node.py --source geojson --field 002
    roslaunch ap_cpp_ros rviz_demo.launch
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import Point, PoseStamped, Quaternion
    from nav_msgs.msg import Path
    from std_msgs.msg import ColorRGBA, Header
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError as exc:  # pragma: no cover - ROS is not installable on the dev box
    sys.stderr.write(
        "This node needs a sourced ROS 1 environment (rospy, nav_msgs, "
        "visualization_msgs).\nImport failed with: {}\n"
        "Try: source /opt/ros/noetic/setup.bash\n".format(exc)
    )
    raise


# --------------------------------------------------------------------------- #
# Repo bootstrap
# --------------------------------------------------------------------------- #
# scripts/ap_cpp_rviz_node.py -> ap_cpp_ros -> src -> ros -> <repo root>
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    from ap_cpp.control import APCPPSpeedController
    from ap_cpp.grid_model import CoverageGrid, GridConfig
    from ap_cpp.mission import APCPPMission, MissionConfig
    from ap_cpp.planner import PlannerConfig, RollingHorizonPlanner
    from ap_cpp.pose import Pose
    from ap_cpp.runtime import DemoPerceptionModel
    from ap_cpp.sensor import SensorConfig, SensorModel
    from ap_cpp.utility import UtilityModel, UtilityWeights
except ImportError as exc:
    sys.stderr.write(
        "Could not import the ap_cpp package from '{}'.\n"
        "The node expects to live in <repo>/ros/src/ap_cpp_ros/scripts/.\n"
        "Underlying error: {}\n".format(_REPO_ROOT, exc)
    )
    raise


# --------------------------------------------------------------------------- #
# Geometry helpers (pure Python, no ROS) - kept importable for unit tests
# --------------------------------------------------------------------------- #
def nadir_to_enu(x_ned: float, y_ned: float, z_ned: float, swap: bool = True):
    """Map a NED point to the ENU convention RViz draws in."""
    if not swap:
        return float(x_ned), float(y_ned), float(-z_ned)
    return float(y_ned), float(x_ned), float(-z_ned)


def yaw_ned_to_enu(yaw_deg: float) -> float:
    """Heading in NED degrees (0 = north, clockwise) -> ENU yaw in radians.

    ENU yaw is measured anticlockwise from +x (east), so a NED heading of 0
    (north) becomes a quarter turn and the sense flips.
    """
    return math.radians(90.0 - float(yaw_deg))


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def coverage_colour(value: float) -> ColorRGBA:
    """Viridis-ish ramp: dark purple (uncovered) -> yellow (covered)."""
    v = min(max(float(value), 0.0), 1.0)
    r = 0.267 + v * (0.993 - 0.267)
    g = 0.005 + v * (0.906 - 0.005)
    b = 0.329 + v * (0.144 - 0.329)
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), 0.85
    return c


def grey_colour(level: float = 0.35, alpha: float = 0.9) -> ColorRGBA:
    c = ColorRGBA()
    c.r = c.g = c.b = float(level)
    c.a = alpha
    return c


# --------------------------------------------------------------------------- #
# Scenario construction
# --------------------------------------------------------------------------- #
def synthetic_field():
    """The L-shaped field with a no-fly zone used by the offline demo."""
    polygon = [
        (0.0, 0.0),
        (140.0, 0.0),
        (140.0, 60.0),
        (70.0, 60.0),
        (70.0, 130.0),
        (0.0, 130.0),
    ]
    obstacles = [[(95.0, 8.0), (128.0, 8.0), (128.0, 38.0), (95.0, 38.0)]]
    return polygon, obstacles


def seed_prior(grid, kind="anomaly", weight=0.85):
    """Overlay a non-uniform uncertainty prior, matching the offline demo."""
    if kind == "none":
        return
    nx, ny = grid.nx, grid.ny
    centres = grid.cell_centers().reshape(nx, ny, 2)
    x, y = centres[:, :, 0], centres[:, :, 1]
    span_x = max(grid.config.x_max - grid.config.x_min, 1e-6)
    span_y = max(grid.config.y_max - grid.config.y_min, 1e-6)
    u = (x - grid.config.x_min) / span_x
    v = (y - grid.config.y_min) / span_y
    if kind == "anomaly":
        blob = np.exp(-(((u - 0.72) ** 2 + (v - 0.28) ** 2) / (2 * 0.16 ** 2)))
    elif kind == "two_zones":
        blob = np.exp(-(((u - 0.25) ** 2 + (v - 0.70) ** 2) / (2 * 0.11 ** 2)))
        blob = np.maximum(
            blob, np.exp(-(((u - 0.78) ** 2 + (v - 0.30) ** 2) / (2 * 0.09 ** 2)))
        )
    else:
        raise ValueError("unknown prior kind: {}".format(kind))
    grid.set_uncertainty_prior(np.clip(blob, 0.0, 1.0), weight=weight)


def build_grid(args):
    """Assemble the belief raster for the requested scenario."""
    if args.source == "geojson":
        from ap_cpp.geo_bridge import GeoBridge, load_qgis_polygon, load_waypoints_txt

        geojson = os.path.join(_REPO_ROOT, "CPP", args.field,
                               "Polygon{}.geojson".format(args.field))
        route_file = os.path.join(_REPO_ROOT, "CPP", args.field, "TurnWPs.txt")
        if not os.path.exists(geojson):
            raise SystemExit("missing scenario file: {}".format(geojson))

        polygon_wgs84, obstacles_wgs84, name = load_qgis_polygon(geojson)
        bridge = GeoBridge(polygon_wgs84, obstacles_wgs84)
        grid = bridge.build_grid(resolution=args.resolution)

        route_ned = None
        if os.path.exists(route_file):
            route_ned = bridge.wgs84_to_ned(load_waypoints_txt(route_file))

        if args.prior != "none":
            seed_prior(grid, args.prior, args.prior_weight)
        rospy.loginfo("scenario: QGIS field '%s'", name)
        return grid, route_ned

    polygon, obstacles = synthetic_field()
    cfg = GridConfig.from_polygon(polygon, resolution=args.resolution)
    grid = CoverageGrid(cfg)
    grid.rasterize_polygon(polygon)
    grid.rasterize_obstacles(obstacles)
    if args.prior != "none":
        seed_prior(grid, args.prior, args.prior_weight)
    rospy.loginfo("scenario: synthetic L-shaped field, %d x %d cells @ %.1f m",
                  grid.nx, grid.ny, args.resolution)
    return grid, None


def route_budget(route, step_length):
    """Observation steps in exactly one pass over ``route``.

    Mirrors ``demos/run_ap_cpp_demo.py``: a fixed budget silently becomes the
    mission plan when the route is much shorter than the field it covers, and
    the reference-only arm then flies on past the end of its route doing work
    the published sweep never performs.  One pass keeps both configurations on
    the same mission.
    """
    if not route or len(route) < 2:
        return 1
    pts = np.array([p.xy for p in route], dtype=float)
    length = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
    return max(int(math.ceil(length / max(step_length, 1e-6))), 1)


def poses_from_route(route_ned):
    """NED polyline -> observation poses, direction-following yaw."""
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


def clip_route_to_grid(poses, grid):
    """Drop reference waypoints that are not traversable or off the raster."""
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


# --------------------------------------------------------------------------- #
# Mission
# --------------------------------------------------------------------------- #
def plan_mission(grid, reference_route, args):
    """Run the coverage mission headlessly and return the record."""
    sensor_cfg = SensorConfig(altitude=args.altitude)
    sensor_cfg.coverage_gain = args.coverage_gain
    sensor = SensorModel(sensor_cfg)

    route = list(reference_route)
    if not route:
        lane = sensor_cfg.ground_width * (1.0 - args.sidelap / 100.0)
        route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane)

    active = not args.reference_only
    planner = RollingHorizonPlanner(
        grid,
        sensor=sensor,
        utility=UtilityModel(UtilityWeights(), sensor=sensor),
        config=PlannerConfig(
            horizon=args.horizon,
            execute_steps=args.execute_steps,
            step_length=args.step_length,
            frontier_targets=3 if active else 0,
            fan_offsets_deg=(-60, -30, 0, 30, 60) if active else (),
            entropy_bias=0.55 if active else 0.0,
            # Same gate as the offline demo: a reference-only run must not be
            # handed a generator whose whole job is patching holes that active
            # perception created by leaving the corridor.
            enable_coverage_repair=active,
        ),
    )
    mission = APCPPMission(
        grid=grid,
        planner=planner,
        perception=DemoPerceptionModel(grid, sensor, seed=args.seed),
        controller=APCPPSpeedController(
            nominal_speed=args.nominal_speed, q_max=args.q_max
        ),
        reference_route=route,
        config=MissionConfig(
            coverage_target=args.coverage_target,
            entropy_target=args.entropy_target,
            max_steps=args.max_steps,
        ),
        sensor=sensor,
    )
    start = route[0] if route else Pose(*grid.cell_center(*grid.free_cells()[0]), 0.0)
    mission.run(start)
    return mission.record, route, start


# --------------------------------------------------------------------------- #
# Message construction
# --------------------------------------------------------------------------- #
def build_path(record, args, frame_id):
    """Flown observation path as a ``nav_msgs/Path``."""
    msg = Path()
    msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    for step in record.steps:
        rx, ry, rz = nadir_to_enu(step.pose.x, step.pose.y, -args.altitude, args.frame_swap)
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position = Point(rx, ry, rz)
        ps.pose.orientation = quaternion_from_yaw(yaw_ned_to_enu(step.pose.yaw_deg))
        msg.poses.append(ps)
    return msg


def build_markers(record, route, grid, args, frame_id):
    """Coverage raster, obstacles, reference sweep and endpoints."""
    markers = MarkerArray()
    alt = float(args.altitude)
    step_m = max(float(args.decimate), 1e-6)

    def make(ns, mid, mtype):
        m = Marker()
        m.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    # ---- 1. passable cells coloured by accumulated coverage ----------------
    cov = make("coverage", 0, Marker.CUBE_LIST)
    cov.scale.x = cov.scale.y = grid.config.resolution * 0.95
    cov.scale.z = 0.15
    width = grid.config.resolution * step_m
    for i in range(0, grid.nx, int(step_m)):
        for j in range(0, grid.ny, int(step_m)):
            if not grid.passable[i, j]:
                continue
            cx, cy = grid.cell_center(i, j)
            rx, ry, rz = nadir_to_enu(cx, cy, -alt, args.frame_swap)
            cov.points.append(Point(rx, ry, rz))
            cov.colors.append(coverage_colour(grid.coverage[i, j]))
    cov.scale.x = cov.scale.y = width
    markers.markers.append(cov)

    # ---- 2. non-traversable cells, flat grey -------------------------------
    blocked = make("blocked", 1, Marker.CUBE_LIST)
    blocked.scale.x = blocked.scale.y = width
    blocked.scale.z = 0.05
    blocked.color = grey_colour()
    for i in range(0, grid.nx, int(step_m)):
        for j in range(0, grid.ny, int(step_m)):
            if grid.passable[i, j]:
                continue
            cx, cy = grid.cell_center(i, j)
            rx, ry, rz = nadir_to_enu(cx, cy, -alt, args.frame_swap)
            blocked.points.append(Point(rx, ry, rz))
    markers.markers.append(blocked)

    # ---- 3. reference sweep ------------------------------------------------
    sweep = make("reference_sweep", 2, Marker.LINE_STRIP)
    sweep.scale.x = 0.4
    sweep.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.9)
    for p in route:
        rx, ry, rz = nadir_to_enu(p.x, p.y, -alt, args.frame_swap)
        sweep.points.append(Point(rx, ry, rz))
    markers.markers.append(sweep)

    # ---- 4. take-off point -------------------------------------------------
    if route:
        start = make("start", 3, Marker.SPHERE)
        start.scale.x = start.scale.y = start.scale.z = 3.0
        start.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=1.0)
        rx, ry, rz = nadir_to_enu(route[0].x, route[0].y, -alt, args.frame_swap)
        start.pose.position = Point(rx, ry, rz)
        start.pose.orientation.w = 1.0
        markers.markers.append(start)

    # ---- 5. final pose -----------------------------------------------------
    if record.steps:
        end = make("end", 4, Marker.SPHERE)
        end.scale.x = end.scale.y = end.scale.z = 3.0
        end.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        last = record.steps[-1].pose
        rx, ry, rz = nadir_to_enu(last.x, last.y, -alt, args.frame_swap)
        end.pose.position = Point(rx, ry, rz)
        end.pose.orientation.w = 1.0
        markers.markers.append(end)

    # ---- 6. summary label --------------------------------------------------
    summary = record.final_summary(grid)
    text = make("summary", 5, Marker.TEXT_VIEW_FACING)
    text.scale.z = 6.0
    text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    text.text = (
        "AP-CPP {}\n"
        "steps {}\n"
        "distance {:.0f} m\n"
        "coverage {:.3f}\n"
        "info gain {:.1f}"
    ).format(
        "reference-only" if args.reference_only else "active perception",
        summary["steps"],
        summary["total_distance_m"],
        summary["coverage_fraction"],
        summary["cumulative_information_gain"],
    )
    cx = 0.5 * (grid.config.x_min + grid.config.x_max)
    cy = 0.5 * (grid.config.y_min + grid.config.y_max)
    rx, ry, rz = nadir_to_enu(cx, cy, -alt, args.frame_swap)
    text.pose.position = Point(rx, ry, rz + 20.0)
    text.pose.orientation.w = 1.0
    markers.markers.append(text)

    return markers


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=("synthetic", "geojson"), default="synthetic")
    p.add_argument("--field", default="002", help="field id for --source geojson")
    p.add_argument("--resolution", type=float, default=2.5, help="raster cell size [m]")
    p.add_argument("--altitude", type=float, default=10.0, help="flight altitude [m]")
    p.add_argument("--sidelap", type=float, default=70.0, help="side overlap [%%]")
    p.add_argument("--coverage-gain", type=float, default=0.6)
    p.add_argument("--horizon", type=int, default=6)
    p.add_argument("--execute-steps", type=int, default=2)
    p.add_argument("--step-length", type=float, default=6.0)
    p.add_argument("--nominal-speed", type=float, default=3.0)
    p.add_argument("--q-max", type=float, default=2.0)
    p.add_argument("--coverage-target", type=float, default=0.85)
    p.add_argument("--entropy-target", type=float, default=0.25)
    p.add_argument("--max-steps", type=int, default=None,
                   help="step budget; default lets the mission run to its targets")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--prior", choices=("none", "anomaly", "two_zones"), default="anomaly")
    p.add_argument("--prior-weight", type=float, default=0.85)
    p.add_argument("--reference-only", action="store_true",
                   help="run the reference-only ablation instead of AP-CPP")
    p.add_argument("--frame-id", default="map", help="RViz fixed frame")
    p.add_argument("--no-frame-swap", dest="frame_swap", action="store_false",
                   help="publish raw NED coordinates instead of ENU")
    p.add_argument("--decimate", type=int, default=1,
                   help="publish every Nth raster cell (keeps RViz responsive)")
    p.add_argument("--republish", type=float, default=0.0,
                   help="seconds between republishes; 0 publishes once, latched")
    p.set_defaults(frame_swap=True)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(rospy.myargv(argv=argv)[1:] if argv is None else argv)

    rospy.init_node("ap_cpp_rviz", anonymous=False)

    grid, route_ned = build_grid(args)
    reference_route = []
    if route_ned is not None and len(route_ned) > 1:
        reference_route = clip_route_to_grid(poses_from_route(route_ned), grid)

    if not args.max_steps:
        # Mirror the offline demo exactly: the budget is one pass over the
        # reference route, so neither configuration can outlive its mission.
        if reference_route:
            args.max_steps = route_budget(reference_route, args.step_length)
        else:
            lane = SensorModel(
                SensorConfig(altitude=args.altitude)
            ).config.ground_width * (1.0 - args.sidelap / 100.0)
            sweep = RollingHorizonPlanner.lawnmower_from_grid(grid, lane)
            args.max_steps = route_budget(sweep, args.step_length)
        rospy.loginfo("step budget derived from the reference route: %d", args.max_steps)

    rospy.loginfo("planning mission ...")
    record, route, _ = plan_mission(grid, reference_route, args)
    summary = record.final_summary(grid)
    rospy.loginfo(
        "planned %d steps, %.0f m, coverage %.3f, info %.1f",
        summary["steps"], summary["total_distance_m"],
        summary["coverage_fraction"], summary["cumulative_information_gain"],
    )

    path_pub = rospy.Publisher("~path", Path, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)

    path_msg = build_path(record, args, args.frame_id)
    marker_msg = build_markers(record, route, grid, args, args.frame_id)

    def publish(_evt=None):
        path_msg.header.stamp = rospy.Time.now()
        marker_msg.markers[0].header.stamp = rospy.Time.now()
        path_pub.publish(path_msg)
        marker_pub.publish(marker_msg)

    publish()

    if args.republish and args.republish > 0:
        rospy.Timer(rospy.Duration(args.republish), publish)
    else:
        rospy.loginfo("published once on latched topics; Ctrl-C to exit")

    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
