#!/usr/bin/env python3
"""Mapping tests for the MSK-Arm workspace occupancy front end.

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_slam.py``

Metrics: visible-workspace coverage (%), occupied-cell IoU against the
ground-truth obstacle map.  The arm base is fixed, so the "SLAM" problem is
mapping the workspace from the depth camera (no pose estimation needed).
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from msk_arm_nn import config as C
from msk_arm_nn.scenes import get_scene
from msk_arm_nn.sim import MskArmSim
from msk_arm_nn.slam import (OccupancyGridMap, SlamFrontend,
                             decode_depth_ranges, depth_beam_angles)


def _dilate(grid: np.ndarray, cells: int = 1) -> np.ndarray:
    """Binary dilation with a square structuring element (numpy only)."""
    out = np.asarray(grid, dtype=bool).copy()
    for _ in range(int(cells)):
        padded = np.pad(out, 1, mode="constant", constant_values=False)
        h, w = out.shape
        acc = out.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                acc |= padded[1 + dy: 1 + dy + h, 1 + dx: 1 + dx + w]
        out = acc
    return out


def _iou(truth: np.ndarray, observed: np.ndarray, region: np.ndarray) -> float:
    t = np.asarray(truth, dtype=bool) & region
    o = np.asarray(observed, dtype=bool) & region
    union = np.count_nonzero(t | o)
    if union == 0:
        return 1.0
    return float(np.count_nonzero(t & o)) / float(union)


# ------------------------------------------------------------------ unit tests
def test_occupancy_grid_map() -> None:
    grid = OccupancyGridMap(0.20, 0.20, origin=(-0.05, -0.05),
                            resolution=C.MAP_RESOLUTION)
    assert (grid.width, grid.height) == (20, 20)
    free = np.zeros((20, 20), dtype=bool)
    occupied = np.zeros((20, 20), dtype=bool)
    free[5:15, 5:15] = True
    occupied[10, 10] = True
    free[10, 10] = False
    for _ in range(2):
        grid.integrate(free, occupied)
    prob = grid.probability
    assert prob.shape == (20, 20) and prob.dtype == np.float32
    assert 0.0 <= float(prob.min()) and float(prob.max()) <= 1.0
    c, r = grid.from_world(0.0, 0.0)
    assert bool(grid.free_mask()[r, c]), "carved cells should be free"
    assert bool(grid.occupied()[10, 10]), "hit cells should be occupied"
    assert bool(grid.known_mask()[10, 10])
    x, y = grid.to_world(c, r)
    assert grid.from_world(x, y) == (c, r)
    assert abs(x - 0.0) <= grid.resolution

    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "map.npz")
    grid.save(path)
    other = OccupancyGridMap(0.05, 0.05, origin=(0.0, 0.0),
                             resolution=C.MAP_RESOLUTION)
    other.load(path)
    assert other.log_odds.shape == grid.log_odds.shape
    assert np.allclose(other.log_odds, grid.log_odds)
    assert abs(other.origin[0] + 0.05) < 1e-9
    print("[unit] occupancy map: free/occupied/save/load OK")


def test_depth_decoding() -> None:
    depth = np.zeros((C.CAM_HEIGHT, C.CAM_WIDTH), dtype=np.float32)
    # 8 bright rows from the bottom of column 3 -> prox = 8/32 = 0.25
    depth[C.CAM_HEIGHT - 8:, 3] = 1.0
    ranges = decode_depth_ranges(depth)
    expected = C.CAM_RANGE * (1.0 - 8.0 / C.CAM_HEIGHT)
    assert abs(float(ranges[3]) - expected) < 1e-6, \
        "decoded range %.3f != %.3f" % (ranges[3], expected)
    assert abs(float(ranges[10]) - C.CAM_RANGE) < 1e-6, \
        "columns without a hit must read the full camera range"
    angles = depth_beam_angles()
    assert angles.size == C.CAM_WIDTH
    assert angles[0] > -math.pi / 2 and angles[-1] < math.pi
    print("[unit] depth decoding: %.3f m hit / %.3f m no-return, %d beams OK"
          % (ranges[3], ranges[10], angles.size))


# ------------------------------------------------------------------ mapping
def test_workspace_mapping() -> None:
    scene = get_scene("reach")
    slam = SlamFrontend(scene)
    assert slam.map.width == int(round((C.WORK_X[1] - C.WORK_X[0])
                                       / C.MAP_RESOLUTION))
    assert slam.map.height == int(round((C.WORK_Y[1] - C.WORK_Y[0])
                                        / C.MAP_RESOLUTION))
    sim = MskArmSim("reach", seed=11, noise=True)
    for _ in range(5):
        slam.update(sim.depth_camera())
    observable = slam.observable_mask()
    known = slam.known_mask()
    assert int(observable.sum()) > 500, "camera fan should cover the workspace"
    coverage = 100.0 * float(np.count_nonzero(known & observable)) \
        / max(1, int(observable.sum()))
    print("[mapping] reach : observable cells %d, coverage %.1f %%, updates %d"
          % (int(observable.sum()), coverage, slam.updates))
    assert coverage > 90.0, \
        "observable workspace coverage %.1f %% too low" % coverage

    # obstacle scenes: compare observed occupied cells with the truth obstacles.
    # The depth image quantises range to CAM_RANGE/32 = 0.028 m while the map is
    # 0.01 m, so the comparison uses a 2-cell (0.02 m) sensor tolerance and
    # reports precision/recall/F1 plus the tolerance IoU.
    for name in ("shelf", "draw"):
        scene = get_scene(name)
        slam = SlamFrontend(scene)
        sim = MskArmSim(name, seed=11, noise=True)
        for _ in range(5):
            slam.update(sim.depth_camera())
        truth = np.asarray(scene.occupancy(C.MAP_RESOLUTION), dtype=bool)
        visible = slam.visible_mask()
        observable = slam.observable_mask()
        observed = slam.occupancy(0.60) & visible
        coverage = 100.0 * float(
            np.count_nonzero(slam.known_mask() & observable)) \
            / max(1, int(np.count_nonzero(observable)))
        truth_tol = _dilate(truth, 2) & visible
        observed_tol = _dilate(observed, 2) & visible
        precision = float(np.count_nonzero(observed & truth_tol)) \
            / max(1, int(np.count_nonzero(observed)))
        recall = float(np.count_nonzero(observed_tol & truth)) \
            / max(1, int(np.count_nonzero(truth)))
        f1 = 2.0 * precision * recall / max(1e-9, precision + recall)
        union = np.count_nonzero(truth_tol | observed_tol)
        iou_tol = float(np.count_nonzero(truth_tol & observed_tol)) \
            / max(1, int(union))
        print("[mapping] %-6s: coverage %.1f %%, occupied precision %.3f / "
              "recall %.3f / F1 %.3f (tolerance IoU %.3f at 0.02 m; "
              "truth cells %d, observed cells %d)"
              % (name, coverage, precision, recall, f1, iou_tol,
                 int(np.count_nonzero(truth)),
                 int(np.count_nonzero(observed))))
        assert coverage > 95.0, \
            "%s observable coverage %.1f %% too low" % (name, coverage)
        assert f1 > 0.80, "%s obstacle F1 %.3f too low" % (name, f1)
    print("[slam] PASS (workspace coverage / obstacle F1 within sensor error)")


def main() -> int:
    print("== test_slam ==")
    test_occupancy_grid_map()
    test_depth_decoding()
    test_workspace_mapping()
    print("[slam] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
