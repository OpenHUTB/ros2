"""
Unit tests for the simulator interface layer.

These tests do not need ROS, AirSim, MuJoCo or PyBullet.  They pin the frame
conventions used by ``airsim_bridge_node.py`` and check that the bundled MJCF
scene and the interface specification keep the expected structure.

    python tests/test_bridge_frames.py
"""

from __future__ import annotations

import math
import sys
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG / "scripts"))

import sim_frames  # noqa: E402
from airsim_bridge_node import (  # noqa: E402
    pointcloud_to_scan,
    ros_odom_from_airsim,
    twist_to_airsim_velocity,
)
from mujoco_bridge_node import default_model_path  # noqa: E402


def _approx(a, b, tol=1e-9):
    return np.allclose(np.asarray(a, float), np.asarray(b, float), atol=tol)


# --------------------------------------------------------------------------
# NED <-> ENU
# --------------------------------------------------------------------------


def test_ned_to_enu_swaps_axes_and_flips_z():
    p = sim_frames.ned_to_enu([1.0, 2.0, 3.0])
    assert _approx(p, [2.0, 1.0, -3.0]), p
    back = sim_frames.enu_to_ned(p)
    assert _approx(back, [1.0, 2.0, 3.0]), back


def test_identity_airsim_attitude_yaws_north_in_enu():
    # FRD body aligned with NED world: forward = north.  In ENU that is +90 deg.
    roll, pitch, yaw = sim_frames.airsim_orientation_to_rpy(1.0, 0.0, 0.0, 0.0)
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9
    assert abs(yaw - math.pi / 2.0) < 1e-9, yaw


def test_airsim_yaw_90_clockwise_becomes_enu_east():
    # 90 deg clockwise from north = east = zero yaw in ENU.
    half = math.pi / 4.0
    _roll, _pitch, yaw = sim_frames.airsim_orientation_to_rpy(
        math.cos(half), 0.0, 0.0, math.sin(half)
    )
    assert abs(sim_frames.wrap_pi(yaw)) < 1e-9, yaw


def test_quaternion_round_trip():
    roll, pitch, yaw = 0.2, -0.3, 1.1
    qx, qy, qz, qw = sim_frames.rpy_to_quat_xyzw(roll, pitch, yaw)
    r2, p2, y2 = sim_frames.quat_wxyz_to_rpy(qw, qx, qy, qz)
    assert abs(r2 - roll) < 1e-9
    assert abs(p2 - pitch) < 1e-9
    assert abs(y2 - yaw) < 1e-9


# --------------------------------------------------------------------------
# command and sensor conversion
# --------------------------------------------------------------------------


def test_twist_to_airsim_velocity_flips_z_and_yaw_rate():
    vx, vy, vz, yaw_rate = twist_to_airsim_velocity([1.0, 2.0, 3.0], 0.5)
    assert _approx([vx, vy, vz, yaw_rate], [2.0, 1.0, -3.0, -0.5])


def test_pointcloud_to_scan_keeps_nearest_return_per_bearing():
    points = [
        [1.0, 0.0, 0.2],
        [0.0, 2.0, 0.2],
        [0.0, 0.0, 1.0],       # range 0 -> below range_min, ignored
        [3.0, 0.0, 0.2],       # same bearing as the first point, farther
    ]
    ranges = pointcloud_to_scan(points, n_bins=4, range_max=10.0)
    assert len(ranges) == 4
    assert abs(ranges[2] - 1.0) < 1e-9, ranges
    assert abs(ranges[3] - 2.0) < 1e-9, ranges
    assert abs(ranges[0] - 10.0) < 1e-9 and abs(ranges[1] - 10.0) < 1e-9


def test_ros_odom_from_airsim_returns_enu_fields():
    position, velocity, quat, rpy = ros_odom_from_airsim(
        [1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 1.0, 0.0, 0.0, 0.0
    )
    assert _approx(position, [2.0, 1.0, -3.0])
    assert _approx(velocity, [0.2, 0.1, -0.3])
    assert abs(rpy[2] - math.pi / 2.0) < 1e-9
    assert abs(np.linalg.norm(quat) - 1.0) < 1e-9


# --------------------------------------------------------------------------
# simulator-neutral CBF barriers
# --------------------------------------------------------------------------


def test_scan_barriers_point_away_from_returns():
    import drl_navigator_node as navigator
    from nav_config import EnvConfig

    node = object.__new__(navigator.DrlNavigatorNode)
    node._lock = threading.Lock()
    node._rpy = np.array([0.0, 0.0, 0.0])
    node.cfg = EnvConfig()
    node.barrier_sectors = 4

    class _Scan:
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        range_max = 10.0
        angle_min = -math.pi
        angle_increment = math.pi / 4.0

    node._scan = _Scan()
    h, g = node._scan_barriers()
    assert h.shape == (4,)
    assert g.shape == (4, 3)
    assert _approx(h, [0.6, 2.6, 4.6, 6.6])
    assert _approx(g[0], [1.0, 0.0, 0.0])
    assert _approx(g[1], [0.0, 1.0, 0.0])


# --------------------------------------------------------------------------
# MuJoCo scene and interface specification
# --------------------------------------------------------------------------


def test_mujoco_scene_has_free_flying_quadrotor_and_obstacles():
    path = Path(default_model_path())
    assert path.is_file(), path
    root = ET.parse(path).getroot()
    body = root.find("./worldbody/body[@name='quadrotor']")
    assert body is not None
    assert body.find("freejoint") is not None
    names = {g.get("name") for g in root.findall("./worldbody/geom")}
    assert {"obstacle_1", "obstacle_2", "obstacle_3", "obstacle_4"} <= names
    gates = [
        g for g in root.findall("./worldbody/geom")
        if g.get("class") == "visual_only"
    ]
    assert len(gates) == 12, len(gates)


def test_interface_spec_lists_standard_topics():
    text = (PKG / "config" / "interface_params.yaml").read_text(encoding="utf-8")
    for token in ("/drone/odom", "/drone/scan", "/drone/cmd_vel",
                  "/drone/nav_status", "airsim", "mujoco", "pybullet"):
        assert token in text, token


# --------------------------------------------------------------------------


def _main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {fn.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"  ERROR {fn.__name__}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} interface tests passed.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_main())
