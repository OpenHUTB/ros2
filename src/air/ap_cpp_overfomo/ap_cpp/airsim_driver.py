"""AirSim integration for the AP-CPP planner.

This is the drop-in counterpart to ``main.py``: same simulator, same
``parameters.py`` configuration, same RedEdge-M payload and same segmentation
network - but the route is re-planned online against the belief grid instead of
being flown open-loop with a speed heuristic.

The module imports ``airsim`` at call time, not at import time, so that the
planner, the tests and the demo all remain runnable on a machine without the
simulator installed.

Usage
-----
1. Launch an AirSim environment.
2. Point ``parameters.py`` at the desired field exactly as for ``main.py``.
3. Run::

       python -m ap_cpp.airsim_driver

The network and GDAL/TensorFlow imports only happen inside
:func:`run_mission`, so a misconfigured field path fails with a clear message
before any of the heavy stack is loaded.
"""

from __future__ import annotations

import math
import os
import time
from typing import List, Optional

import numpy as np

from ap_cpp.control import APCPPSpeedController
from ap_cpp.geo_bridge import GeoBridge, load_qgis_polygon, load_waypoints_txt
from ap_cpp.grid_model import CoverageGrid
from ap_cpp.mission import APCPPMission, MissionConfig
from ap_cpp.planner import PlannerConfig, RollingHorizonPlanner
from ap_cpp.pose import Pose, heading_between
from ap_cpp.sensor import SensorConfig, SensorModel
from ap_cpp.utility import UtilityModel, UtilityWeights

__all__ = ["AirSimConfig", "load_field", "run_mission"]


class AirSimConfig:
    """Everything the driver needs, with the repository defaults applied."""

    def __init__(self, parameters=None):
        p = parameters
        if p is None:
            import parameters as p  # noqa: F811

        self.qgis_path = p.qgis_path
        self.turnwps_path = p.turnwps_path
        self.weights_path = p.weights_path
        self.backbone = p.backbone
        self.save_path_wps = p.save_path_wps
        self.mission_type = getattr(p, "mission_type", "variable")
        self.initial_velocity = getattr(p, "initial_velocity", 4.0)
        self.distance_threshold = getattr(p, "distance_threshold", 0.3)

        self.altitude = 10.0
        self.resolution = 2.5
        self.side_lap = 70.0
        self.vehicle_name = "Drone1"


def load_field(cfg: AirSimConfig):
    """Load the QGIS polygon, obstacles and reference route from disk."""
    if not os.path.exists(cfg.qgis_path):
        raise FileNotFoundError("QGIS polygon not found: {}".format(cfg.qgis_path))

    polygon_wgs84, obstacles_wgs84, name = load_qgis_polygon(cfg.qgis_path)
    bridge = GeoBridge(polygon_wgs84, obstacles_wgs84)
    grid = bridge.build_grid(resolution=cfg.resolution)

    route_ned = None
    if os.path.exists(cfg.turnwps_path):
        route_ned = bridge.wgs84_to_ned(load_waypoints_txt(cfg.turnwps_path))

    return bridge, grid, route_ned, name


def _poses_from_ned(route_ned, grid: CoverageGrid) -> List[Pose]:
    """NED polyline -> observation poses, clipped to traversable ground."""
    poses: List[Pose] = []
    pts = [(float(p[0]), float(p[1])) for p in route_ned]
    for k, (x, y) in enumerate(pts):
        if k + 1 < len(pts):
            dx, dy = pts[k + 1][0] - x, pts[k + 1][1] - y
        elif k > 0:
            dx, dy = x - pts[k - 1][0], y - pts[k - 1][1]
        else:
            dx, dy = 1.0, 0.0
        yaw = math.degrees(math.atan2(dy, dx)) if (dx or dy) else 0.0
        cell = grid.nearest_free_cell(x, y)
        if cell is None:
            continue
        cx, cy = grid.cell_center(*cell)
        if math.hypot(cx - x, cy - y) > grid.config.resolution * 1.5:
            continue
        poses.append(Pose(cx, cy, yaw))
    return poses


def run_mission(cfg: Optional[AirSimConfig] = None, dry_run: bool = False):
    """Fly an active perception coverage mission in AirSim.

    Parameters
    ----------
    cfg:
        Driver configuration; repository ``parameters.py`` is used when omitted.
    dry_run:
        Run the whole planning/perception loop against the real orthomosaic but
        without connecting to a simulator.  Useful for validating a field's
        configuration without launching Unreal.
    """
    cfg = cfg or AirSimConfig()
    bridge, grid, route_ned, field_name = load_field(cfg)
    print("[field] '{}', polygon {} vertices, {} obstacles -> raster {}x{}".format(
        field_name, len(bridge.polygon_wgs84), len(bridge.obstacles_ned),
        grid.nx, grid.ny))

    sensor_cfg = SensorConfig(altitude=cfg.altitude)
    sensor = SensorModel(sensor_cfg)

    if route_ned is not None:
        reference_route = _poses_from_ned(route_ned, grid)
    else:
        reference_route = []
    if not reference_route:
        lane = sensor_cfg.lane_spacing(cfg.side_lap)
        reference_route = RollingHorizonPlanner.lawnmower_from_grid(grid, lane)
        print("[route] no TurnWPs file usable; generated a {:.2f} m sweep".format(lane))
    else:
        print("[route] {} reference waypoints from {}".format(
            len(reference_route), os.path.basename(cfg.turnwps_path)))

    planner = RollingHorizonPlanner(
        grid,
        sensor=sensor,
        utility=UtilityModel(UtilityWeights(), sensor=sensor),
        config=PlannerConfig(
            step_length=cfg.initial_velocity * 2.0,
            replan_interval=2.0,
        ),
    )
    controller = APCPPSpeedController(nominal_speed=cfg.initial_velocity)

    # -- perception: the real trained segmentation network ----------------
    from ap_cpp.runtime import SegmentationPerceptionModel

    perception = SegmentationPerceptionModel(cfg.weights_path, cfg.backbone)

    client = None
    if not dry_run:
        import airsim

        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True, cfg.vehicle_name)
        client.armDisarm(True, cfg.vehicle_name)

        z = -cfg.altitude
        client.simPlotLineStrip(
            points=[airsim.Vector3r(p.x, p.y, z) for p in reference_route],
            color_rgba=[1.0, 1.0, 0.0, 1.0],
            thickness=20,
            is_persistent=True,
        )
        airsim.wait_key("Press any key to take off")
        client.takeoffAsync(vehicle_name=cfg.vehicle_name).join()
        client.moveToZAsync(z, 5, vehicle_name=cfg.vehicle_name).join()
        print("[airsim] airborne at {:.1f} m".format(cfg.altitude))
    else:
        print("[dry-run] skipping the simulator connection")

    # -- mission wiring ----------------------------------------------------
    mission = APCPPMission(
        grid=grid,
        planner=planner,
        perception=perception,
        controller=controller,
        reference_route=reference_route,
        config=MissionConfig(),
        sensor=sensor,
    )
    mission.pose = reference_route[0]
    mission._replan(force=True)

    step = 0
    while True:
        target = mission.consume_pose()
        if target is None:
            print("[mission] nothing left worth visiting")
            break

        if client is None:
            # Dry run: warp to the target so the belief loop still advances.
            reached = target
        else:
            client.moveToPositionAsync(
                target.x, target.y, -cfg.altitude, controller.nominal_speed,
                vehicle_name=cfg.vehicle_name,
            ).join()
            state = client.getMultirotorState(cfg.vehicle_name)
            reached = Pose(
                state.kinematics_estimated.position.x_val,
                state.kinematics_estimated.position.y_val,
                target.yaw_deg,
            )

        # -- Take the picture and score it at the pose actually reached ----
        # The gap between `target` and `reached` is where model error, wind
        # and controller lag enter; the next replan is planned from `reached`,
        # which is what makes the receding horizon robust to them.
        waypoint = bridge.ned_to_wgs84([(reached.x, reached.y)])[0]
        reading = perception.perceive_wgs84(waypoint, reached.yaw_deg)

        # Fuse the reading into the belief, then let the controller set speed
        # from the belief plus the frame content.
        command = controller.command(
            coverage_ratio=reading.coverage_ratio,
            measurement_entropy=reading.measurement_entropy,
            planned_gain=mission._last_planned_gain,
            mean_entropy=grid.mean_entropy(),
        )
        diag = mission.observe_at(reached)
        step += 1
        print(
            "[{:3d}] ({:7.2f},{:7.2f}) H={:.3f} cov={:.3f} IG={:.4f} "
            "v={:.2f} m/s ({})".format(
                step, reached.x, reached.y, diag.entropy_after,
                diag.coverage_fraction, diag.information_gain,
                command.speed, command.reason,
            )
        )

        if grid.mission_complete(mission.config.coverage_target, mission.config.entropy_target):
            print("[mission] coverage and entropy targets reached")
            break
        if step >= mission.config.max_steps:
            print("[mission] step budget exhausted")
            break

    # -- persist -----------------------------------------------------------
    os.makedirs(cfg.save_path_wps, exist_ok=True)
    if mission.record.steps:
        path = os.path.join(cfg.save_path_wps, "ViewWPs_APCPP.txt")
        with open(path, "w", encoding="utf-8") as fh:
            for s in mission.record.steps:
                fh.write("{}, {}, {}\n".format(s.pose.x, s.pose.y, s.pose.yaw_deg))
        print("[save] viewpoints -> {}".format(path))
    print("[summary] {}".format(mission.record.final_summary(grid)))

    if client is not None:
        import airsim

        client.armDisarm(False, cfg.vehicle_name)
        client.reset()
        client.enableApiControl(False, cfg.vehicle_name)

    return mission.record


if __name__ == "__main__":
    import sys

    run_mission(dry_run="--dry-run" in sys.argv)
